"""
advisor_ui.py — Gradio web interface for the Personal Evacuation Advisor

Usage:
    python advisor_ui.py
    Then open http://localhost:7860
"""

import base64
import os
import random

import folium
import gradio as gr
import pyproj

from scenario_loader import load_scenario, temporary_config, _apply_disaster_rules
from evac_env import EvacEnv
from personal_advisor import (
    _validate_profile,
    _profile_from_description,
    _generate_recommendation,
    _fallback_recommendation,
    _run_drqn_route,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

CHECKPOINT = "logs/drqn_llm_persona/drqn_torch_best.pt"
SCENARIO_PATH = "scenarios/enterprise_blizzard.json"

# Named campus starting locations: label -> OSM walk node
CAMPUS_LOCATIONS = {
    "🎲 Random location":          None,
    "📚 Marriott Library":          10206645945,
    "🏢 Student Union Building":    6861237919,
    "🏟 Rice-Eccles Stadium":       10171975035,
    "🎭 Kingsbury Hall":            4119764241,
    "⚙️ Merrill Engineering Bldg":  10550594426,
    "🏥 University Hospital":       11552825228,
    "🏋️ HPER Building":             83542422,
    "🚇 South Campus (Trax)":       1520496421,
}

# Shelter labels built dynamically after env loads (see below)

# Coordinate transform (UTM 12N ↔ WGS84)
_TO_LATLON = pyproj.Transformer.from_crs("EPSG:32612", "EPSG:4326", always_xy=True)
_TO_UTM    = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:32612", always_xy=True)

# ---------------------------------------------------------------------------
# Load environment once
# ---------------------------------------------------------------------------

_scenario = load_scenario(SCENARIO_PATH)
_scenario["disaster_severity"] = "moderate"
_scenario = _apply_disaster_rules(_scenario)
_ctx = temporary_config(_scenario.get("config_overrides", {}))
_ctx.__enter__()
_env = EvacEnv()
_ALL_NODES = list(_env.G_walk.nodes())
_DISASTER_TYPE = _scenario.get("disaster_type", "blizzard")

# Build shelter labels dynamically: sort by latitude (north=A)
_shelter_list_sorted = sorted(
    _env.shelters,
    key=lambda s: _TO_LATLON.transform(*_env.pos[s])[1],  # sort by lat descending
    reverse=True,
)
SHELTER_LABELS = {node: f"Shelter {chr(65+i)}" for i, node in enumerate(_shelter_list_sorted)}

print(f"[ui] loaded — {len(_ALL_NODES):,} walk nodes, {len(_env.shelters)} shelters")
for node, label in SHELTER_LABELS.items():
    lon, lat = _TO_LATLON.transform(*_env.pos[node])
    print(f"  {label}: node={node}, lat={lat:.5f}, lon={lon:.5f}")


def _utm_to_latlon(x, y):
    lon, lat = _TO_LATLON.transform(x, y)
    return lat, lon


def _nearest_walk_node(lat, lon):
    x, y = _TO_UTM.transform(lon, lat)
    best, best_d = None, float("inf")
    for n in _ALL_NODES:
        nx_, ny_ = _env.pos[n]
        d = (nx_ - x) ** 2 + (ny_ - y) ** 2
        if d < best_d:
            best_d, best = d, n
    return best


# ---------------------------------------------------------------------------
# Map builder
# ---------------------------------------------------------------------------

def _build_base_map(start_node=None):
    """Return a folium map with shelters marked and optionally a start pin."""
    # Map center = centroid of all shelter positions
    s_latlons = [_utm_to_latlon(*_env.pos[s]) for s in _env.shelters]
    center_lat = sum(ll[0] for ll in s_latlons) / len(s_latlons)
    center_lon = sum(ll[1] for ll in s_latlons) / len(s_latlons)

    m = folium.Map(location=[center_lat, center_lon], zoom_start=15,
                   tiles="CartoDB positron")

    # Shelter markers (red)
    for s in _env.shelters:
        lat, lon = _utm_to_latlon(*_env.pos[s])
        label = SHELTER_LABELS.get(s, f"Shelter {s}")
        folium.Marker(
            location=[lat, lon],
            popup=folium.Popup(f"<b>{label}</b><br>Node: {s}", max_width=200),
            tooltip=label,
            icon=folium.Icon(color="red", icon="home", prefix="fa"),
        ).add_to(m)

    # Start node marker (blue)
    if start_node is not None and start_node in _env.G_walk:
        lat, lon = _utm_to_latlon(*_env.pos[start_node])
        folium.Marker(
            location=[lat, lon],
            popup=folium.Popup(f"<b>Your Location</b><br>Node: {start_node}", max_width=200),
            tooltip="Your Location",
            icon=folium.Icon(color="blue", icon="user", prefix="fa"),
        ).add_to(m)

    return m


def _add_route_to_map(m, route, start_node):
    """Overlay the DRQN route path on an existing folium map."""
    path_nodes = route.get("path_nodes", [])
    if len(path_nodes) < 2:
        return m

    coords = []
    for n in path_nodes:
        if n in _env.pos:
            lat, lon = _utm_to_latlon(*_env.pos[n])
            coords.append([lat, lon])

    if coords:
        folium.PolyLine(
            coords,
            color="#2563eb",
            weight=4,
            opacity=0.8,
            tooltip="Evacuation route",
        ).add_to(m)

        # Mark shelter reached
        end_node = path_nodes[-1]
        if end_node in _env.shelters:
            lat, lon = _utm_to_latlon(*_env.pos[end_node])
            label = SHELTER_LABELS.get(end_node, "Destination Shelter")
            folium.Marker(
                location=[lat, lon],
                popup=folium.Popup(f"<b>✅ {label}</b><br>Destination", max_width=200),
                tooltip=f"Destination: {label}",
                icon=folium.Icon(color="green", icon="flag", prefix="fa"),
            ).add_to(m)

    return m


def _map_html(m):
    """Render folium map as a base64 iframe for Gradio gr.HTML."""
    html_bytes = m.get_root().render().encode("utf-8")
    b64 = base64.b64encode(html_bytes).decode("utf-8")
    return (
        f'<iframe src="data:text/html;base64,{b64}" '
        f'width="100%" height="500px" frameborder="0"></iframe>'
    )


# ---------------------------------------------------------------------------
# Core pipeline
# ---------------------------------------------------------------------------

def advise(description, severity, location_label, api_key_field):
    # Resolve start node
    node_id = CAMPUS_LOCATIONS.get(location_label)
    if node_id is None:
        node_id = random.choice(_ALL_NODES)

    start_node = node_id

    # Resolve API key
    key = api_key_field.strip() or os.environ.get("GROQ_API_KEY", "")

    if not description.strip():
        m = _build_base_map(start_node)
        return "", "", "⚠️ Please enter a description first.", _map_html(m)

    # --- Layer 1: Behavior profile ---
    profile_source = "rule-based defaults (no API key)"
    client = None
    if key:
        try:
            from groq import Groq
            client = Groq(api_key=key)
            profile = _validate_profile(_profile_from_description(client, description))
            profile_source = "LLM-inferred (Llama 3.3 70B)"
        except Exception as e:
            profile = _validate_profile({})
            profile_source = f"LLM failed ({e}), using defaults"
    else:
        profile = _validate_profile({})

    # --- Layer 3: DRQN route ---
    try:
        route = _run_drqn_route(
            env=_env,
            checkpoint_path=CHECKPOINT,
            start_node=start_node,
            profile=profile,
            seed=42,
            device="auto",
            max_neighbors=None,
        )
    except Exception as e:
        m = _build_base_map(start_node)
        return "", "", f"DRQN route error: {e}", _map_html(m)

    # --- Output: LLM recommendation ---
    if client is not None and description.strip():
        try:
            recommendation = _generate_recommendation(
                client, description, profile, route, _DISASTER_TYPE, severity
            )
        except Exception:
            recommendation = _fallback_recommendation(profile, route, _DISASTER_TYPE, severity)
    else:
        recommendation = _fallback_recommendation(profile, route, _DISASTER_TYPE, severity)

    # --- Format profile table ---
    panic = profile["panic_level"]
    speed = profile["walk_speed_multiplier"]
    familiarity = profile["shelter_familiarity"]
    speed_label = "very slow" if speed <= 0.4 else "slow" if speed <= 0.7 else "normal" if speed <= 1.1 else "fast"
    panic_label = "very high" if panic >= 0.75 else "high" if panic >= 0.5 else "moderate" if panic >= 0.25 else "low"
    fam_label = "none" if familiarity <= 0.15 else "low" if familiarity <= 0.35 else "moderate" if familiarity <= 0.65 else "high"

    profile_md = (
        f"**Source**: {profile_source}\n\n"
        f"| Parameter | Value | Label |\n"
        f"|-----------|-------|-------|\n"
        f"| Walk speed | {speed:.2f}× | {speed_label} |\n"
        f"| Compliance rate | {profile['compliance_rate']:.2f} | — |\n"
        f"| Panic level | {panic:.2f} | {panic_label} |\n"
        f"| Observation error | {profile['observation_error_multiplier']:.2f}× | — |\n"
        f"| Decision delay | {profile['decision_delay_steps']} steps | — |\n"
        f"| Shelter familiarity | {familiarity:.2f} | {fam_label} |"
    )

    # --- Format route table ---
    minutes = round(route["steps"] * 5 / 60, 1)
    shelter_node = route.get("recommended_shelter")
    shelter_label = SHELTER_LABELS.get(shelter_node, f"node {shelter_node}") if shelter_node else "unknown"

    route_md = (
        f"| Metric | Value |\n"
        f"|--------|-------|\n"
        f"| **Recommended shelter** | **{shelter_label}** |\n"
        f"| Steps | {route['steps']} |\n"
        f"| Est. time | {minutes} min |\n"
        f"| Hazard exposure | {route['exposure']:.2f} |\n"
        f"| Reroutes due to blockages | {route['replan_count']} |\n"
        f"| Path length | {route['path_length_nodes']} nodes |"
    )

    # --- Build map with route ---
    m = _build_base_map(start_node)
    m = _add_route_to_map(m, route, start_node)

    shelter_node = route.get("recommended_shelter")
    shelter_label = SHELTER_LABELS.get(shelter_node, f"node {shelter_node}") if shelter_node else "unknown"
    reroutes = route['replan_count']
    reroute_str = f", {reroutes} reroute(s)" if reroutes > 0 else ""
    banner = (
        f"### 🏠 Recommended: **{shelter_label}**"
        f" — ~{round(route['steps'] * 5 / 60, 1)} min{reroute_str}\n\n"
    )

    return profile_md, route_md, banner + recommendation, _map_html(m)


def on_location_change(location_label):
    """Update the map preview when user changes location."""
    node_id = CAMPUS_LOCATIONS.get(location_label)
    if node_id is None:
        node_id = random.choice(_ALL_NODES)
    m = _build_base_map(node_id)
    return _map_html(m)


# ---------------------------------------------------------------------------
# Gradio UI
# ---------------------------------------------------------------------------

_initial_node = CAMPUS_LOCATIONS["📚 Marriott Library"]
_initial_map = _map_html(_build_base_map(_initial_node))

with gr.Blocks(title="Campus Evacuation Advisor") as demo:
    gr.Markdown("""
    # 🏫 Campus Evacuation Personal Advisor
    **University of Utah — Three-Layer LLM + DRQN Evacuation System**

    Describe yourself in natural language. The system will:
    1. **Layer 1 (LLM)** — Infer your behavioral profile (speed, panic, campus familiarity…)
    2. **Layer 3 (DRQN)** — Compute the optimal evacuation route from your location
    3. **Output (LLM)** — Generate a personalized plain-language recommendation
    """)

    with gr.Row():
        # ── Left column: inputs ──────────────────────────────────────────────
        with gr.Column(scale=1):
            gr.Markdown("### Your Information")

            description_input = gr.Textbox(
                label="Describe yourself",
                placeholder=(
                    "e.g. I am a first-year international student. "
                    "This is my first week on campus and I have no idea where any shelters are.\n\n"
                    "e.g. I am a senior faculty member and have worked here for 15 years.\n\n"
                    "e.g. I use a wheelchair and I'm feeling very anxious."
                ),
                lines=5,
            )

            location_input = gr.Dropdown(
                label="Your current location",
                choices=list(CAMPUS_LOCATIONS.keys()),
                value="📚 Marriott Library",
            )

            severity_input = gr.Dropdown(
                label="Disaster severity",
                choices=["light", "moderate", "severe", "extreme"],
                value="moderate",
            )

            api_key_input = gr.Textbox(
                label="Groq API Key (leave blank to use environment variable)",
                placeholder="gsk_...",
                type="password",
            )

            submit_btn = gr.Button("🚨 Get Evacuation Advice", variant="primary", size="lg")

        # ── Right column: outputs ─────────────────────────────────────────────
        with gr.Column(scale=1):
            gr.Markdown("### Results")

            recommendation_output = gr.Markdown(label="Personalized Recommendation")

            with gr.Accordion("Behavioral Profile (Layer 1)", open=False):
                profile_output = gr.Markdown()

            with gr.Accordion("Route Summary (Layer 3)", open=False):
                route_output = gr.Markdown()

    # ── Map (full width) ──────────────────────────────────────────────────────
    gr.Markdown("### Campus Map")
    gr.Markdown(
        "🔴 Red = Shelters &nbsp;&nbsp; 🔵 Blue = Your location &nbsp;&nbsp; "
        "🟢 Green = Destination shelter &nbsp;&nbsp; 🔷 Blue line = Evacuation route"
    )
    map_output = gr.HTML(value=_initial_map)

    # ── Shelter reference table ───────────────────────────────────────────────
    with gr.Accordion("Shelter Locations Reference", open=False):
        _shelter_table = "| Shelter | Node ID | Lat | Lon |\n|---------|---------|-----|-----|\n"
        for node, label in SHELTER_LABELS.items():
            lon, lat = _TO_LATLON.transform(*_env.pos[node])
            _shelter_table += f"| {label} | {node} | {lat:.5f} | {lon:.5f} |\n"
        gr.Markdown(_shelter_table)

    # ── Events ───────────────────────────────────────────────────────────────
    location_input.change(
        fn=on_location_change,
        inputs=location_input,
        outputs=map_output,
    )

    submit_btn.click(
        fn=advise,
        inputs=[description_input, severity_input, location_input, api_key_input],
        outputs=[profile_output, route_output, recommendation_output, map_output],
    )


if __name__ == "__main__":
    demo.launch(share=False, inbrowser=True)
