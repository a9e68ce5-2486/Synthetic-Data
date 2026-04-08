"""
llm_scenario_generator.py

Use an LLM (Llama 3.3 70B via Groq) to generate physically-grounded simulation
parameters for each disaster type × severity combination.

The LLM uses its own knowledge of real-world disasters to decide what values
are physically reasonable — no numeric ranges are provided in the prompt.
A safety clamp is applied after generation as a hard guardrail only.

Generated parameters replace the hard-coded severity override tables in
scenario_loader.py.  The output JSON is saved to:

    scenarios/llm_severity_presets.json

scenario_loader.py automatically loads this file at startup if it exists,
so no further changes are needed to run sweeps.

Usage:
    python llm_scenario_generator.py --api-key YOUR_GROQ_KEY
    python llm_scenario_generator.py --api-key YOUR_GROQ_KEY --verbose --compare
    python llm_scenario_generator.py --api-key YOUR_GROQ_KEY --output scenarios/llm_severity_presets.json
"""

import argparse
import json
import os
import re

# ---------------------------------------------------------------------------
# Parameter descriptions (physical meaning only — no numeric ranges given to LLM)
# Safety clamp bounds are kept here as a post-generation guardrail only.
# ---------------------------------------------------------------------------

PARAM_SCHEMA = {
    "EVAC_BLOCK_PROB": {
        "clamp": [0.0, 0.6],
        "desc": (
            "Fraction of walkable road segments randomly blocked per simulation step. "
            "Represents ongoing hazard progression — roads being cut off as the disaster evolves. "
            "A light blizzard might see 3–5% of roads affected per step; "
            "an extreme earthquake could block 30%+ per step due to aftershocks and cascading damage."
        ),
    },
    "EVAC_BLOCK_INIT_PROB": {
        "clamp": [0.0, 0.9],
        "desc": (
            "Fraction of road segments already blocked when the simulation starts (t=0). "
            "For blizzard: near zero — snow damage accumulates over time, not instantly. "
            "For earthquake: high — structural collapse and debris create immediate blockages at onset. "
            "For compound: maximum of the two component disasters."
        ),
    },
    "EVAC_SNOW_MIN": {
        "clamp": [0.0, 1.0],
        "desc": (
            "Minimum snow or debris accumulation level on road surfaces (0 = bare, 1 = fully impassable). "
            "For blizzard: how much snow has already fallen before the simulation begins. "
            "For earthquake: represents rubble/debris level rather than snow (typically low). "
            "For compound: reflects both snow and rubble."
        ),
    },
    "EVAC_SNOW_MAX": {
        "clamp": [0.0, 1.0],
        "desc": (
            "Maximum snow or debris accumulation level (must be >= SNOW_MIN). "
            "The worst-case surface condition reached during the simulation. "
            "Extreme blizzard could reach near 1.0 (roads completely snowbound); "
            "earthquake debris is lower since rubble does not keep accumulating like snowfall."
        ),
    },
    "EVAC_SNOW_ALPHA": {
        "clamp": [0.0, 3.0],
        "desc": (
            "Friction penalty multiplier applied to travel time due to snow or debris. "
            "A value of 1.0 means no added friction; 2.0 means travel takes twice as long. "
            "Heavy snow or rubble fields significantly slow pedestrian movement."
        ),
    },
    "EVAC_SNOW_ACCUM_PER_STEP": {
        "clamp": [0.0, 0.015],
        "desc": (
            "Mean amount of snow or debris accumulation added per simulation step (each step ≈ 5 seconds). "
            "A full simulation runs 600 steps (~50 minutes). "
            "For a severe blizzard, consider how much a road surface degrades over 50 minutes of heavy snowfall. "
            "For earthquake, this represents aftershock-induced rubble increments (typically very small)."
        ),
    },
    "EVAC_SNOW_ACCUM_NOISE": {
        "clamp": [0.0, 0.005],
        "desc": (
            "Standard deviation of per-step accumulation noise — captures spatial variability "
            "(some roads accumulate faster than others). Should be a fraction of ACCUM_PER_STEP."
        ),
    },
    "EVAC_BLOCK_FROM_SNOW_THRESHOLD": {
        "clamp": [0.3, 1.0],
        "desc": (
            "Snow or debris level above which a road segment becomes eligible to be blocked. "
            "Higher threshold means roads stay open longer despite accumulation. "
            "A light blizzard may only block roads when snow is near-maximum; "
            "an extreme event blocks roads at much lower accumulation levels."
        ),
    },
    "EVAC_BLOCK_FROM_SNOW_PROB": {
        "clamp": [0.0, 0.03],
        "desc": (
            "Per-step probability that a road segment above the snow/debris threshold becomes permanently blocked. "
            "Controls how quickly accumulation converts to actual road closures."
        ),
    },
    "EVAC_OBS_ERROR_WALK": {
        "clamp": [0.0, 0.5],
        "desc": (
            "Pedestrian observation noise multiplier (0 = perfect perception, higher = more confused). "
            "Blizzard whiteout severely impairs visibility; earthquake causes disorientation from fallen structures. "
            "Represents how accurately evacuees perceive their surroundings and navigate."
        ),
    },
    "EVAC_OBS_ERROR_DRIVE": {
        "clamp": [0.0, 0.5],
        "desc": (
            "Vehicle observation noise multiplier. Typically lower than pedestrian noise "
            "since vehicles have better vantage points, but still affected by smoke, debris, and poor visibility."
        ),
    },
}

DISASTER_CONTEXTS = {
    "blizzard": (
        "A winter blizzard on a university campus. "
        "Effects: heavy snowfall accumulates on roads, reducing visibility and mobility. "
        "Roads are blocked progressively by snow accumulation (not sudden structural damage). "
        "Initial road damage is minimal — most disruption builds up over time. "
        "Pedestrian visibility and navigation are heavily impaired in severe/extreme cases."
    ),
    "earthquake": (
        "A seismic earthquake event on a university campus. "
        "Effects: structural damage blocks roads immediately at disaster onset (high BLOCK_INIT_PROB). "
        "Snow/debris accumulation is minimal (earthquake does not produce snow, but rubble is modeled "
        "as a small debris accumulation value for friction effects). "
        "Observation error is moderate — fallen structures obstruct sight lines but don't cause "
        "the same white-out confusion as a blizzard. "
        "Magnitude increases with severity: light ~M5.8, moderate ~M6.8, severe ~M7.8, extreme ~M8.5+."
    ),
    "compound": (
        "A compound disaster: simultaneous earthquake and blizzard on a university campus. "
        "Effects combine both: immediate structural road damage (earthquake) PLUS progressive "
        "snow accumulation (blizzard). Initial damage is the maximum of the two disasters at the same "
        "severity level. Snow and friction effects follow blizzard dynamics. "
        "Observation error is the worst of both: structural obstructions AND low visibility. "
        "This is the most severe scenario type — parameters should reflect compounding risk."
    ),
}

SEVERITY_DESCRIPTIONS = {
    "light": (
        "Minor/light event. Minimal disruption. Most roads remain passable. "
        "Evacuation is inconvenient but manageable without major delays."
    ),
    "moderate": (
        "Moderate event. Notable disruption. Some roads blocked or impaired. "
        "Evacuation requires rerouting for some agents."
    ),
    "severe": (
        "Severe event. Significant disruption. Many roads affected. "
        "Evacuation is difficult, especially for mobility-impaired or unfamiliar populations."
    ),
    "extreme": (
        "Extreme / worst-case event. Major disruption across most of the campus. "
        "High initial damage, continued degradation. Evacuation success rates are expected to drop "
        "significantly even for capable agents."
    ),
}

SYSTEM_PROMPT = """\
You are an emergency management expert calibrating a campus evacuation simulation.

The simulation models pedestrian and vehicle evacuation on the University of Utah campus
road network. Each simulation step represents approximately 5 seconds; a full run lasts
up to 600 steps (~50 minutes).

You will be given a disaster type and severity level, along with physical descriptions
of what each parameter controls in the simulation. Use your knowledge of real-world
disasters to choose values that are physically realistic and internally consistent.

Guidelines:
- Parameters should show a clear monotonic progression across severities:
  light → moderate → severe → extreme (more disruption each level)
- For blizzard: roads degrade gradually from snow accumulation; initial damage is minimal
- For earthquake: structural collapse causes immediate road damage at onset;
  snow/debris from rubble is secondary
- For compound: combines both — maximum initial damage plus progressive snow accumulation
- All values are floats between 0 and 1 (or small decimals for accumulation rates);
  use your physical judgement to decide what is realistic

Respond ONLY with a valid JSON object — no explanation text, no markdown fences.
Include a "reasoning" field (1–2 sentences) explaining your key calibration decisions.
"""


# ---------------------------------------------------------------------------
# LLM call
# ---------------------------------------------------------------------------

def _generate_one(client, disaster_type: str, severity: str, verbose: bool) -> dict:
    """Ask LLM to generate parameters for one (disaster_type, severity) pair."""
    # Build parameter descriptions with no numeric ranges
    param_lines = []
    for param, info in PARAM_SCHEMA.items():
        param_lines.append(f"  {param}:\n    {info['desc']}")
    param_str = "\n".join(param_lines)

    user_prompt = (
        f"Disaster type: {disaster_type}\n"
        f"Severity: {severity}\n\n"
        f"Disaster context:\n{DISASTER_CONTEXTS[disaster_type]}\n\n"
        f"Severity description:\n{SEVERITY_DESCRIPTIONS[severity]}\n\n"
        f"Parameters to generate (physical descriptions only — choose values based on "
        f"your knowledge of real disasters):\n{param_str}\n\n"
        "Generate the simulation parameters for this disaster type and severity."
    )

    if verbose:
        print(f"  [llm] calling for {disaster_type}/{severity} ...", end=" ", flush=True)

    response = client.chat.completions.create(
        model="llama-3.3-70b-versatile",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": user_prompt},
        ],
        temperature=0.1,
        max_tokens=512,
    )
    raw = response.choices[0].message.content.strip()
    raw = re.sub(r"^```[a-zA-Z]*\n?", "", raw)
    raw = re.sub(r"\n?```$", "", raw)

    params = json.loads(raw)
    if verbose:
        print("done")
    return params


# ---------------------------------------------------------------------------
# Validation and clamping
# ---------------------------------------------------------------------------

def _clamp(val, lo, hi):
    return max(lo, min(hi, val))


def _validate_and_clamp(params: dict, disaster_type: str, severity: str) -> dict:
    """Clamp all numeric params to safety bounds; enforce SNOW_MAX >= SNOW_MIN."""
    out = {}
    for param, info in PARAM_SCHEMA.items():
        lo, hi = info["clamp"]
        v = params.get(param)
        if v is None:
            v = (lo + hi) / 2
        out[param] = round(_clamp(float(v), lo, hi), 6)

    # Enforce SNOW_MAX >= SNOW_MIN
    if out["EVAC_SNOW_MAX"] < out["EVAC_SNOW_MIN"]:
        out["EVAC_SNOW_MAX"] = out["EVAC_SNOW_MIN"]

    # Preserve reasoning if present
    if "reasoning" in params:
        out["reasoning"] = str(params["reasoning"])[:300]

    return out


# ---------------------------------------------------------------------------
# Main generation loop
# ---------------------------------------------------------------------------

DISASTER_TYPES = ["blizzard", "earthquake", "compound"]
SEVERITIES = ["light", "moderate", "severe", "extreme"]


def generate_all_presets(api_key: str, verbose: bool = False) -> dict:
    """Generate LLM parameters for all 3 × 4 = 12 combinations."""
    from groq import Groq
    client = Groq(api_key=api_key)

    presets = {}
    for disaster in DISASTER_TYPES:
        presets[disaster] = {}
        print(f"\n[generator] {disaster}:")
        for severity in SEVERITIES:
            params = _generate_one(client, disaster, severity, verbose)
            validated = _validate_and_clamp(params, disaster, severity)
            presets[disaster][severity] = validated
            if verbose:
                print(f"    {severity}: BLOCK_PROB={validated['EVAC_BLOCK_PROB']:.3f}  "
                      f"BLOCK_INIT={validated['EVAC_BLOCK_INIT_PROB']:.3f}  "
                      f"SNOW={validated['EVAC_SNOW_MIN']:.2f}–{validated['EVAC_SNOW_MAX']:.2f}  "
                      f"OBS_ERR={validated['EVAC_OBS_ERROR_WALK']:.2f}")
    return presets


# ---------------------------------------------------------------------------
# Print comparison table
# ---------------------------------------------------------------------------

def print_comparison(presets: dict):
    """Print side-by-side LLM vs original hardcoded values."""
    # Import original hardcoded tables for comparison
    from scenario_loader import (
        BLIZZARD_SEVERITY_OVERRIDES,
        EARTHQUAKE_SEVERITY_OVERRIDES,
        COMPOUND_SEVERITY_OVERRIDES,
    )
    originals = {
        "blizzard": BLIZZARD_SEVERITY_OVERRIDES,
        "earthquake": EARTHQUAKE_SEVERITY_OVERRIDES,
        "compound": COMPOUND_SEVERITY_OVERRIDES,
    }

    key_params = [
        "EVAC_BLOCK_PROB", "EVAC_BLOCK_INIT_PROB",
        "EVAC_SNOW_MIN", "EVAC_SNOW_MAX",
        "EVAC_OBS_ERROR_WALK",
    ]

    for disaster in DISASTER_TYPES:
        print(f"\n## {disaster.upper()}")
        print(f"{'Severity':<10} {'Param':<30} {'Original':>10} {'LLM':>10}")
        print("-" * 62)
        for severity in SEVERITIES:
            orig = originals[disaster].get(severity, {})
            llm = presets[disaster].get(severity, {})
            for i, param in enumerate(key_params):
                o_val = orig.get(param, "—")
                l_val = llm.get(param, "—")
                sev_label = severity if i == 0 else ""
                o_str = f"{o_val:.4f}" if isinstance(o_val, float) else str(o_val)
                l_str = f"{l_val:.4f}" if isinstance(l_val, float) else str(l_val)
                print(f"  {sev_label:<8} {param:<30} {o_str:>10} {l_str:>10}")


# ---------------------------------------------------------------------------
# Save
# ---------------------------------------------------------------------------

def save_presets(presets: dict, output_path: str):
    """Save presets JSON (without reasoning fields for cleanliness in the output)."""
    clean = {}
    reasoning_log = {}
    for disaster, severities in presets.items():
        clean[disaster] = {}
        reasoning_log[disaster] = {}
        for severity, params in severities.items():
            clean[disaster][severity] = {
                k: v for k, v in params.items() if k != "reasoning"
            }
            if "reasoning" in params:
                reasoning_log[disaster][severity] = params["reasoning"]

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(clean, f, indent=2)
    print(f"\n[generator] saved → {output_path}")

    # Save reasoning separately
    reasoning_path = output_path.replace(".json", "_reasoning.json")
    with open(reasoning_path, "w", encoding="utf-8") as f:
        json.dump(reasoning_log, f, indent=2, ensure_ascii=False)
    print(f"[generator] reasoning → {reasoning_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LLM Disaster Scenario Generator")
    parser.add_argument("--api-key", default="",
                        help="Groq API key (or set GROQ_API_KEY env var)")
    parser.add_argument("--output", default="scenarios/llm_severity_presets.json",
                        help="Output path for generated presets JSON")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--compare", action="store_true",
                        help="Print comparison table vs original hardcoded values")
    args = parser.parse_args()

    api_key = args.api_key or os.environ.get("GROQ_API_KEY", "")
    if not api_key:
        print("[generator] ERROR: no API key. Pass --api-key or set GROQ_API_KEY.")
        return

    print(f"[generator] Generating parameters for 3 disasters × 4 severities "
          f"using Llama 3.3 70B via Groq ...")

    presets = generate_all_presets(api_key, verbose=args.verbose)
    save_presets(presets, args.output)

    if args.compare:
        print_comparison(presets)

    print("\n[generator] Done. scenario_loader.py will automatically use these "
          "parameters if the file exists at the output path.")


if __name__ == "__main__":
    main()
