"""
visualize_map.py

Interactive HTML map visualization using Folium.

Two modes:

  1. Route mode (--mode route):
     Visualize a single Personal Advisor route for one agent.
     Shows: start point, shelter, path, blocked edges, agent profile.

  2. Simulation mode (--mode simulation):
     Visualize a full batch simulation run.
     Shows: all agent paths color-coded by persona, shelters, bottleneck edges.

Usage:
    # Single route (Personal Advisor output)
    python visualize_map.py --mode route \
        --route-json logs/personal_advisor/enterprise_blizzard_<node>_advice.json \
        --checkpoint logs/drqn_torch_best.pt \
        --scenario scenarios/enterprise_blizzard.json \
        --output logs/maps/route_map.html

    # Simulation run
    python visualize_map.py --mode simulation \
        --scenario scenarios/enterprise_blizzard.json \
        --checkpoint logs/drqn_torch_best.pt \
        --severity moderate \
        --output logs/maps/simulation_map.html
"""

import argparse
import json
import os
import random

import folium
from folium.plugins import AntPath, MeasureControl, MiniMap

import config
from evac_env import EvacEnv
from scenario_loader import load_scenario, temporary_config, _apply_disaster_rules

# ---------------------------------------------------------------------------
# Color palette per persona
# ---------------------------------------------------------------------------

PERSONA_COLORS = {
    # Students — blue family
    "young_student":              "#4A90D9",
    "freshman_student":           "#1A5276",
    "graduate_student":           "#85C1E9",
    "international_student":      "#2874A6",
    "student_athlete":            "#AED6F1",
    "student_with_anxiety":       "#1B4F72",
    "part_time_student":          "#5DADE2",
    # Faculty — green family
    "senior_faculty":             "#27AE60",
    "junior_faculty":             "#52BE80",
    "adjunct_instructor":         "#A9DFBF",
    # Staff — orange family
    "staff_admin":                "#E67E22",
    "facilities_staff":           "#CA6F1E",
    "campus_security":            "#F39C12",
    "healthcare_staff":           "#FAD7A0",
    "research_scientist":         "#D35400",
    "it_staff":                   "#F0B27A",
    # Visitors — red/purple family
    "visitor":                    "#E74C3C",
    "mobility_impaired":          "#7D3C98",
    "conference_attendee":        "#F1948A",
    "prospective_student_with_parent": "#C39BD3",
    # Fallback
    "user_generated":             "#2ECC71",
    "unknown":                    "#95A5A6",
}

ROLE_COLORS = {
    "student": "#4A90D9",
    "faculty": "#27AE60",
    "staff":   "#E67E22",
    "visitor": "#E74C3C",
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _node_latlon(env, node):
    """Return (lat, lon) for a node. OSM pos is stored as (x=lon, y=lat)."""
    if node not in env.pos:
        return None
    x, y = env.pos[node]
    return (y, x)  # folium uses (lat, lon)


def _edge_latlon(env, u, v):
    a = _node_latlon(env, u)
    b = _node_latlon(env, v)
    if a and b:
        return [a, b]
    return None


def _base_map(env):
    """Create a base folium map centered on the campus."""
    lats = [y for x, y in env.pos.values()]
    lons = [x for x, y in env.pos.values()]
    center = (sum(lats) / len(lats), sum(lons) / len(lons))
    m = folium.Map(location=center, zoom_start=16, tiles="CartoDB positron")
    MiniMap(toggle_display=True).add_to(m)
    MeasureControl(primary_length_unit="meters").add_to(m)
    return m


def _add_shelters(m, env):
    for s in env.shelters:
        ll = _node_latlon(env, s)
        if not ll:
            continue
        folium.Marker(
            location=ll,
            tooltip=f"Shelter {s}",
            icon=folium.Icon(color="green", icon="home", prefix="fa"),
        ).add_to(m)


def _add_blocked_edges(m, env):
    blocked = getattr(env, "blocked_edges_walk", set())
    for u, v in blocked:
        coords = _edge_latlon(env, u, v)
        if coords:
            folium.PolyLine(
                coords, color="red", weight=3, opacity=0.6,
                tooltip="Blocked road",
            ).add_to(m)


# ---------------------------------------------------------------------------
# Mode 1: Route visualization
# ---------------------------------------------------------------------------

def _run_route_sim(env, checkpoint_path, start_node, profile, seed, device, max_neighbors):
    """Re-run DRQN route and return path_nodes + metadata."""
    from personal_advisor import _run_drqn_route
    return _run_drqn_route(env, checkpoint_path, start_node, profile, seed, device, max_neighbors)


def visualize_route(env, route_data, profile, output_path):
    """Build a route map from Personal Advisor output + env."""
    m = _base_map(env)
    _add_shelters(m, env)
    _add_blocked_edges(m, env)

    path_nodes = route_data.get("path_nodes", [])
    coords = [_node_latlon(env, n) for n in path_nodes]
    coords = [c for c in coords if c]

    if len(coords) >= 2:
        # Animated path line
        AntPath(
            locations=coords,
            color="#2471A3",
            weight=4,
            opacity=0.8,
            tooltip="Evacuation route",
        ).add_to(m)

    # Start marker
    start_ll = _node_latlon(env, route_data["start_node"])
    if start_ll:
        panic = float(profile.get("panic_level", 0.0))
        familiarity = float(profile.get("shelter_familiarity", 0.5))
        folium.Marker(
            location=start_ll,
            tooltip=(
                f"START<br>"
                f"panic: {panic:.2f}<br>"
                f"familiarity: {familiarity:.2f}<br>"
                f"speed: {profile.get('walk_speed_multiplier', 1.0):.2f}x"
            ),
            icon=folium.Icon(color="blue", icon="user", prefix="fa"),
        ).add_to(m)

    # Goal marker (if reached)
    goal_ll = _node_latlon(env, route_data["recommended_shelter"])
    if goal_ll:
        reached = route_data.get("reached", False)
        folium.Marker(
            location=goal_ll,
            tooltip=f"Target shelter — {'REACHED' if reached else 'NOT REACHED'}",
            icon=folium.Icon(color="green" if reached else "orange", icon="flag", prefix="fa"),
        ).add_to(m)

    # Info box
    steps = route_data.get("steps", 0)
    replans = route_data.get("replan_count", 0)
    exposure = route_data.get("exposure", 0.0)
    reached = route_data.get("reached", False)
    html = f"""
    <div style="font-family:sans-serif;font-size:13px;padding:8px;background:white;
                border-radius:6px;box-shadow:2px 2px 6px rgba(0,0,0,0.2);max-width:240px;">
      <b>Personal Advisor Route</b><br>
      <hr style="margin:4px 0">
      Reached shelter: <b>{'Yes' if reached else 'No'}</b><br>
      Steps: {steps} (~{round(steps*5/60,1)} min)<br>
      Exposure: {exposure:.2f}<br>
      Replans due to blockage: {replans}<br>
      <hr style="margin:4px 0">
      <b>Profile</b><br>
      Speed: {profile.get('walk_speed_multiplier',1.0):.2f}x &nbsp;
      Panic: {float(profile.get('panic_level',0)):.2f}<br>
      Familiarity: {float(profile.get('shelter_familiarity',0.5)):.2f} &nbsp;
      Delay: {profile.get('decision_delay_steps',0)}s
    </div>
    """
    folium.Marker(
        location=_base_map_center(env),
        icon=folium.DivIcon(html=html, icon_size=(250, 160), icon_anchor=(0, 0)),
    ).add_to(m)

    os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else ".", exist_ok=True)
    m.save(output_path)
    print(f"[map] route map saved → {output_path}")
    return m


def _base_map_center(env):
    lats = [y for x, y in env.pos.values()]
    lons = [x for x, y in env.pos.values()]
    return (sum(lats) / len(lats), sum(lons) / len(lons))


# ---------------------------------------------------------------------------
# Mode 2: Simulation visualization
# ---------------------------------------------------------------------------

def visualize_simulation(env, peds, ped_goals, edge_counts, output_path, title="Simulation"):
    """Build a simulation map showing all agent paths and personas."""
    m = _base_map(env)
    _add_shelters(m, env)
    _add_blocked_edges(m, env)

    # Bottleneck edges (top 15)
    top_edges = sorted(edge_counts.items(), key=lambda kv: kv[1], reverse=True)[:15]
    max_cnt = max((cnt for _, cnt in top_edges), default=1)
    for (u, v), cnt in top_edges:
        coords = _edge_latlon(env, u, v)
        if not coords:
            continue
        weight = 2 + 6 * (cnt / max_cnt)
        folium.PolyLine(
            coords, color="#E74C3C", weight=weight, opacity=0.5,
            tooltip=f"Bottleneck: {cnt} agent passes",
        ).add_to(m)

    # Agent markers
    persona_groups = {}
    for ped in peds:
        persona = getattr(ped, "persona", None) or "unknown"
        color = PERSONA_COLORS.get(persona, "#95A5A6")
        role = getattr(ped, "role", "unknown")

        start_ll = _node_latlon(env, ped.node)
        if not start_ll:
            continue

        goal = ped_goals.get(ped.id)
        goal_ll = _node_latlon(env, goal) if goal else None

        reached = ped.reached
        marker_color = "green" if reached else "red"

        tooltip_html = (
            f"Agent {ped.id} | {persona}<br>"
            f"Role: {role}<br>"
            f"Reached: {'Yes' if reached else 'No'}<br>"
            f"Steps: {ped.steps}<br>"
            f"Exposure: {ped.exposure:.2f}<br>"
            f"Panic: {getattr(ped,'panic_level',0):.2f} | "
            f"Familiarity: {getattr(ped,'shelter_familiarity',1.0):.2f}"
        )

        # Group layer per persona
        if persona not in persona_groups:
            persona_groups[persona] = folium.FeatureGroup(
                name=f"{persona} ({role})", show=True
            )

        folium.CircleMarker(
            location=start_ll,
            radius=5,
            color=color,
            fill=True,
            fill_color=marker_color,
            fill_opacity=0.8,
            tooltip=tooltip_html,
        ).add_to(persona_groups[persona])

    for group in persona_groups.values():
        group.add_to(m)

    folium.LayerControl(collapsed=False).add_to(m)

    # Title box
    title_html = f"""
    <div style="position:fixed;top:10px;left:60px;z-index:1000;font-family:sans-serif;
                font-size:14px;padding:8px 12px;background:white;border-radius:6px;
                box-shadow:2px 2px 6px rgba(0,0,0,0.25);">
      <b>{title}</b><br>
      Agents: {len(peds)} &nbsp; Reached: {sum(1 for p in peds if p.reached)}<br>
      Reached rate: {sum(1 for p in peds if p.reached)/max(1,len(peds)):.1%}
    </div>
    """
    m.get_root().html.add_child(folium.Element(title_html))

    os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else ".", exist_ok=True)
    m.save(output_path)
    print(f"[map] simulation map saved → {output_path}")
    return m


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Interactive map visualization")
    parser.add_argument("--mode", choices=["route", "simulation"], default="simulation")
    parser.add_argument("--scenario", default="scenarios/enterprise_blizzard.json")
    parser.add_argument("--checkpoint", default="logs/drqn_torch_best.pt")
    parser.add_argument("--severity", default="moderate",
                        choices=["light", "moderate", "severe", "extreme"])
    parser.add_argument("--seed", type=int, default=20260301)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--max-neighbors", type=int, default=None)
    parser.add_argument("--output", default="logs/maps/map.html")
    # Route mode only
    parser.add_argument("--route-json", default="",
                        help="Path to Personal Advisor JSON output (route mode)")
    parser.add_argument("--start-node", type=str, default="",
                        help="OSM node ID for route mode (overrides route-json start)")
    parser.add_argument("--description", default="",
                        help="User description for route mode (generates profile via LLM if API key set)")
    parser.add_argument("--api-key", default="")
    args = parser.parse_args()

    scenario = load_scenario(args.scenario)
    scenario["disaster_severity"] = args.severity
    scenario = _apply_disaster_rules(scenario)

    os.makedirs(os.path.dirname(args.output) if os.path.dirname(args.output) else ".", exist_ok=True)

    with temporary_config(scenario.get("config_overrides", {})):
        random.seed(args.seed)
        import numpy as np
        np.random.seed(args.seed)

        env = EvacEnv()

        if args.mode == "route":
            # Load route from JSON or run advisor
            if args.route_json and os.path.exists(args.route_json):
                with open(args.route_json) as f:
                    advice = json.load(f)
                profile = advice["profile"]
                route = advice["route"]
                # route in saved JSON omits path_nodes — re-run to get them
                start_node = advice["start_node"]
                if isinstance(start_node, str):
                    try:
                        start_node = int(start_node)
                    except ValueError:
                        pass
                print(f"[map] re-running DRQN route for node {start_node}...")
                from personal_advisor import _run_drqn_route
                route = _run_drqn_route(env, args.checkpoint, start_node, profile,
                                        args.seed, args.device, args.max_neighbors)
            elif args.start_node:
                try:
                    start_node = int(args.start_node)
                except ValueError:
                    start_node = args.start_node
                from personal_advisor import PersonalAdvisor, _validate_profile
                advisor = PersonalAdvisor(
                    api_key=args.api_key or os.environ.get("GROQ_API_KEY", ""),
                    checkpoint_path=args.checkpoint,
                    device=args.device,
                    max_neighbors=args.max_neighbors,
                    verbose=True,
                )
                result = advisor.advise(
                    description=args.description,
                    env=env,
                    start_node=start_node,
                    seed=args.seed,
                )
                profile = result["profile"]
                route = result["route"]
            else:
                parser.error("Route mode requires --route-json or --start-node")

            visualize_route(env, route, profile, args.output)

        else:  # simulation mode
            from batch_runner import _build_agents, _capacity_aware_goal_map
            from collections import defaultdict

            peds, cars, _ = _build_agents(env)
            shelters_walk = list(env.shelters)

            from batch_runner import _DRQNPedController, _build_drive_shelter_mapping
            shelters_drive, drive_goal_to_shelter = _build_drive_shelter_mapping(env, shelters_walk)

            ped_goals = _capacity_aware_goal_map(peds, env.G_walk, shelters_walk, "drqn", env)
            for a in peds:
                a.target_shelter = ped_goals.get(a.id)

            ctrl = _DRQNPedController(
                env=env,
                checkpoint_path=args.checkpoint,
                device=args.device,
                max_neighbors=args.max_neighbors,
                max_steps=config.EVAC_STEP_LIMIT,
            )
            ctrl.reset_agents(peds)
            for a in peds:
                ctrl.init_goal(a.id, a.node, ped_goals[a.id])

            edge_counts = defaultdict(int)
            import networkx as nx

            for step in range(config.EVAC_STEP_LIMIT):
                env.step_hazards()
                ctrl.update_interaction_state(peds, cars)
                ctrl.reset_decision_step()
                for a in peds:
                    if getattr(a, "_delay_remaining", 0) > 0:
                        a._delay_remaining -= 1
                        continue
                    if not a.reached and a.alive:
                        goal = ped_goals[a.id]
                        if a.node == goal:
                            a.reached = True
                            continue
                        committed = ctrl.committed_next(a.id)
                        if committed is not None and a.node == committed:
                            ctrl.clear_committed_next(a.id)
                            committed = None
                        if committed is None:
                            nxt = ctrl.select_next_node(a, goal, step)
                            if nxt != a.node:
                                ctrl.set_committed_next(a.id, nxt)
                                committed = nxt
                        if committed and committed != a.node:
                            if not env.is_blocked(a.node, committed, mode="walk", belief=None):
                                prev = a.node
                                a.node = committed
                                ctrl.clear_committed_next(a.id)
                                edge_counts[(prev, a.node)] += 1
                                if a.node == goal:
                                    a.reached = True

            title = f"{scenario.get('name','scenario')} | {args.severity} | DRQN | seed={args.seed}"
            visualize_simulation(env, peds, ped_goals, edge_counts, args.output, title=title)


if __name__ == "__main__":
    main()
