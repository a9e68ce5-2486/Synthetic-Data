import sys, os; sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
"""
demo_pipeline.py — End-to-end three-layer pipeline demo

Demonstrates the full LLM-DRQN evacuation system in sequence:

  Layer 2 — LLM Zone Coordinator
    Divides campus into zones, uses LLM ReAct loop to assign each zone
    to a primary + backup shelter (capacity-aware, road-condition-aware).

  Layer 1 — LLM Behavior Profiler
    Parses a natural language user description into a quantitative
    behavior profile (panic, familiarity, speed, compliance, delay).

  Layer 3 — DRQN Navigation
    Runs the DRQN model to find the best route from the user's location
    to their assigned shelter, incorporating all persona parameters.

  Output — LLM Recommendation
    Translates route + profile into a plain-language personalized
    evacuation recommendation calibrated to the user's familiarity level.

Usage:
    python demo_pipeline.py \\
        --checkpoint logs/drqn_progressive_severity_v3_extreme/drqn_torch_best.pt \\
        --scenario scenarios/enterprise_blizzard.json \\
        --severity moderate \\
        --description "I am a first-year international student. \\
                       This is my second week on campus." \\
        --start-node 1638160433 \\
        --api-key YOUR_GROQ_KEY \\
        --verbose

    # Without LLM (algorithmic fallback for zone coordinator, default profile):
    python demo_pipeline.py \\
        --checkpoint logs/drqn_progressive_severity_v3_extreme/drqn_torch_best.pt \\
        --scenario scenarios/enterprise_blizzard.json \\
        --start-node 1638160433
"""

import argparse
import json
import os
import random
import time

import numpy as np

import config
from evac_env import EvacEnv
from scenario_loader import load_scenario, temporary_config, _apply_disaster_rules
from zone_assignment import _build_zones, _build_ped_population

# ---------------------------------------------------------------------------
# Banner helpers
# ---------------------------------------------------------------------------

def _banner(title, width=62):
    print("\n" + "═" * width)
    pad = (width - len(title) - 2) // 2
    print(" " * pad + f"  {title}")
    print("═" * width)


def _section(title):
    print(f"\n── {title} {'─' * max(0, 56 - len(title))}")


def _kv(key, value, indent=2):
    print(" " * indent + f"{key:<32s} {value}")


# ---------------------------------------------------------------------------
# Step 1: Environment setup
# ---------------------------------------------------------------------------

def setup_env(scenario, seed, verbose=False):
    _section("Environment Setup")
    random.seed(seed)
    np.random.seed(seed)
    env = EvacEnv()
    shelters = list(env.shelters)
    nodes = list(env.G_walk.nodes())
    _kv("Walk graph nodes", f"{len(nodes):,}")
    _kv("Walk graph edges", f"{env.G_walk.number_of_edges():,}")
    _kv("Shelters", len(shelters))
    _kv("Blocked edges (walk)", len(getattr(env, 'blocked_edges_walk', set())))
    if verbose:
        _kv("Shelter IDs", str([int(s) if hasattr(s,'__int__') else s for s in shelters[:4]]) + ("..." if len(shelters) > 4 else ""))
    return env


# ---------------------------------------------------------------------------
# Step 2: Layer 2 — Zone Coordinator
# ---------------------------------------------------------------------------

def run_zone_coordinator(env, scenario, api_key, seed, num_zones, verbose):
    _section("Layer 2 — LLM Zone Coordinator")
    disaster_type = scenario.get("disaster_type", "unknown")
    severity = scenario.get("disaster_severity", "unknown")

    peds = _build_ped_population(env, seed=seed)
    zones = _build_zones(env, peds, num_zones=num_zones, seed=seed)

    from llm_zone_coordinator import LLMZoneCoordinator, build_coordinator_state
    state = build_coordinator_state(zones, env, disaster_type=disaster_type, severity=severity)

    coordinator = LLMZoneCoordinator(
        api_key=api_key,
        verbose=verbose,
        max_iterations=20,
    )
    t0 = time.time()
    zones_annotated, source = coordinator.assign(state, env, zones)
    elapsed = time.time() - t0

    _kv("Assignment source", source)
    _kv("Zones assigned", len(zones_annotated))
    _kv("Time elapsed", f"{elapsed:.1f}s")
    print()
    print(f"  {'Zone':>4}  {'Demand':>6}  {'Primary Shelter':<20}  {'Backup Shelter':<20}  Reasoning")
    print(f"  {'----':>4}  {'------':>6}  {'-'*20}  {'-'*20}  ---------")
    for z in zones_annotated:
        ps = str(z.get('primary_shelter', '—'))[:18]
        bs = str(z.get('backup_shelter', '—'))[:18]
        reason = (z.get('llm_reasoning') or '')[:45]
        print(f"  {z['zone_id']:>4}  {z['demand']:>6}  {ps:<20}  {bs:<20}  {reason}")

    return zones_annotated, source


# ---------------------------------------------------------------------------
# Step 3: Layer 1 — Behavior Profiling
# ---------------------------------------------------------------------------

def run_behavior_profiler(description, api_key, verbose):
    _section("Layer 1 — LLM Behavior Profiler")

    if not description:
        _kv("Mode", "default profile (no description provided)")
        profile = {
            "walk_speed_multiplier": 1.0,
            "compliance_rate": 0.8,
            "panic_level": 0.3,
            "observation_error_multiplier": 1.0,
            "decision_delay_steps": 1,
            "shelter_familiarity": 0.5,
        }
        return profile

    _kv("Description", f'"{description[:80]}{"..." if len(description) > 80 else ""}"')

    key = api_key or os.environ.get("GROQ_API_KEY", "")
    if not key:
        _kv("Mode", "no API key — using default profile")
        return {
            "walk_speed_multiplier": 1.0,
            "compliance_rate": 0.8,
            "panic_level": 0.3,
            "observation_error_multiplier": 1.0,
            "decision_delay_steps": 1,
            "shelter_familiarity": 0.5,
        }

    from groq import Groq
    from personal_advisor import _profile_from_description, _validate_profile
    client = Groq(api_key=key)

    t0 = time.time()
    try:
        profile = _validate_profile(_profile_from_description(client, description))
    except Exception as e:
        _kv("WARNING", f"Profile generation failed ({e}), using defaults")
        profile = _validate_profile({})
    elapsed = time.time() - t0

    _kv("Time elapsed", f"{elapsed:.1f}s")
    print()

    # Compute effective values after panic modulation
    panic = profile["panic_level"]
    eff_compliance = profile["compliance_rate"] * (1 - 0.5 * panic)
    eff_obs_error  = profile["observation_error_multiplier"] * (1 + panic)

    _kv("walk_speed_multiplier", f"{profile['walk_speed_multiplier']:.2f}x")
    _kv("panic_level", f"{panic:.2f}")
    _kv("shelter_familiarity", f"{profile['shelter_familiarity']:.2f}")
    _kv("decision_delay_steps", str(profile["decision_delay_steps"]))
    _kv("compliance_rate (raw → effective)", f"{profile['compliance_rate']:.2f} → {eff_compliance:.2f}")
    _kv("obs_error_multiplier (raw → effective)", f"{profile['observation_error_multiplier']:.2f} → {eff_obs_error:.2f}")

    return profile


# ---------------------------------------------------------------------------
# Step 4: Layer 3 — DRQN Navigation
# ---------------------------------------------------------------------------

def run_drqn_navigation(env, checkpoint_path, start_node, profile, seed, device, max_neighbors, verbose):
    _section("Layer 3 — DRQN Navigation")

    if start_node not in env.G_walk:
        print(f"  WARNING: node {start_node} not in walk graph — picking random node")
        start_node = random.choice(list(env.G_walk.nodes()))

    _kv("Start node", str(start_node))

    from personal_advisor import _run_drqn_route
    t0 = time.time()
    route = _run_drqn_route(env, checkpoint_path, start_node, profile, seed, device, max_neighbors)
    elapsed = time.time() - t0

    _kv("Recommended shelter", str(route["recommended_shelter"]))
    _kv("Reached shelter", str(route["reached"]))
    _kv("Steps taken", f"{route['steps']} (~{route['steps']*5/60:.1f} min)")
    _kv("Exposure accumulated", f"{route['exposure']:.3f}")
    _kv("Replans (blockage detours)", str(route["replan_count"]))
    _kv("Path length (nodes)", str(route["path_length_nodes"]))
    _kv("Time elapsed", f"{elapsed:.1f}s")

    return route


# ---------------------------------------------------------------------------
# Step 5: Output — LLM Recommendation
# ---------------------------------------------------------------------------

def run_recommendation(description, profile, route, scenario, api_key, verbose):
    _section("Output — Personalized Evacuation Recommendation")

    disaster_type = scenario.get("disaster_type", "blizzard")
    severity = scenario.get("disaster_severity", "moderate")
    key = api_key or os.environ.get("GROQ_API_KEY", "")

    if key and description:
        from groq import Groq
        from personal_advisor import _generate_recommendation
        client = Groq(api_key=key)
        t0 = time.time()
        try:
            recommendation = _generate_recommendation(
                client, description, profile, route, disaster_type, severity
            )
        except Exception as e:
            _kv("WARNING", f"Recommendation failed ({e}), using fallback")
            from personal_advisor import _fallback_recommendation
            recommendation = _fallback_recommendation(profile, route, disaster_type, severity)
        elapsed = time.time() - t0
        _kv("Time elapsed", f"{elapsed:.1f}s")
    else:
        from personal_advisor import _fallback_recommendation
        recommendation = _fallback_recommendation(profile, route, disaster_type, severity)

    print()
    print("  " + "─" * 56)
    for line in recommendation.split("\n"):
        print(f"  {line}")
    print("  " + "─" * 56)

    return recommendation


# ---------------------------------------------------------------------------
# Save results
# ---------------------------------------------------------------------------

def save_results(output_dir, scenario, start_node, profile, route,
                 zones_annotated, zone_source, recommendation, description):
    os.makedirs(output_dir, exist_ok=True)
    stem = f"{scenario.get('name','scenario')}_{start_node}"
    path = os.path.join(output_dir, f"{stem}_demo.json")

    def _ser(v):
        if isinstance(v, (np.integer,)):
            return int(v)
        if isinstance(v, (np.floating,)):
            return float(v)
        return v

    result = {
        "scenario": scenario.get("name"),
        "disaster_type": scenario.get("disaster_type"),
        "severity": scenario.get("disaster_severity"),
        "start_node": _ser(start_node),
        "description": description,
        "behavior_profile": profile,
        "route": {k: _ser(v) for k, v in route.items()
                  if k not in ("path_nodes", "traversed_edges", "target_history")},
        "zone_assignment_source": zone_source,
        "zone_assignments": [
            {
                "zone_id": z["zone_id"],
                "demand": z["demand"],
                "primary_shelter": _ser(z.get("primary_shelter")),
                "backup_shelter": _ser(z.get("backup_shelter")),
                "reasoning": z.get("llm_reasoning", ""),
            }
            for z in zones_annotated
        ],
        "recommendation": recommendation,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    return path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="End-to-end LLM-DRQN evacuation pipeline demo")
    parser.add_argument("--scenario", default="scenarios/enterprise_blizzard.json")
    parser.add_argument("--checkpoint",
                        default="logs/drqn_progressive_severity_v3_extreme/drqn_torch_best.pt")
    parser.add_argument("--severity", default="moderate",
                        choices=["light", "moderate", "severe", "extreme"])
    parser.add_argument("--description", default="",
                        help="Natural language description of the evacuating person")
    parser.add_argument("--start-node", type=str, required=True,
                        help="OSM node ID for the person's current location")
    parser.add_argument("--api-key", default="",
                        help="Groq API key (or set GROQ_API_KEY env var)")
    parser.add_argument("--seed", type=int, default=20260323)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--max-neighbors", type=int, default=None)
    parser.add_argument("--num-zones", type=int, default=6)
    parser.add_argument("--output-dir", default="logs/demo_pipeline")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    api_key = args.api_key or os.environ.get("GROQ_API_KEY", "")

    try:
        start_node = int(args.start_node)
    except ValueError:
        start_node = args.start_node

    # Load and prepare scenario
    scenario = load_scenario(args.scenario)
    scenario["disaster_severity"] = args.severity
    scenario = _apply_disaster_rules(scenario)

    _banner("LLM-DRQN Campus Evacuation System — End-to-End Demo")
    _kv("Scenario", scenario.get("name", args.scenario))
    _kv("Disaster", f"{scenario.get('disaster_type','?')} / {args.severity}")
    _kv("Checkpoint", args.checkpoint)
    _kv("LLM", "Llama 3.3 70B via Groq" if api_key else "offline (no API key)")

    t_total = time.time()

    with temporary_config(scenario.get("config_overrides", {})):
        # Step 1: Environment
        env = setup_env(scenario, seed=args.seed, verbose=args.verbose)

        # Step 2: Layer 2 — Zone Coordinator
        zones_annotated, zone_source = run_zone_coordinator(
            env, scenario, api_key, seed=args.seed,
            num_zones=args.num_zones, verbose=args.verbose,
        )

        # Step 3: Layer 1 — Behavior Profiling
        profile = run_behavior_profiler(args.description, api_key, verbose=args.verbose)

        # Step 4: Layer 3 — DRQN Navigation
        route = run_drqn_navigation(
            env, args.checkpoint, start_node, profile,
            seed=args.seed, device=args.device,
            max_neighbors=args.max_neighbors, verbose=args.verbose,
        )

        # Step 5: Output — Recommendation
        recommendation = run_recommendation(
            args.description, profile, route, scenario, api_key, verbose=args.verbose,
        )

    # Save
    out_path = save_results(
        args.output_dir, scenario, start_node, profile, route,
        zones_annotated, zone_source, recommendation, args.description,
    )

    elapsed_total = time.time() - t_total
    _section("Summary")
    _kv("Total pipeline time", f"{elapsed_total:.1f}s")
    _kv("Zone assignment source", zone_source)
    _kv("Route reached shelter", str(route["reached"]))
    _kv("Output saved", out_path)
    print()


if __name__ == "__main__":
    main()
