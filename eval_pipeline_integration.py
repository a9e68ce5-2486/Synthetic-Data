"""
eval_pipeline_integration.py

Step 22: End-to-end 3-layer pipeline integration experiment.

Compares two zone assignment strategies using the persona-aware DRQN checkpoint:
  B) Algorithmic zone assignment + persona-aware DRQN (Layer 1 + 3)
  C) LLM zone assignment + persona-aware DRQN (Layer 1 + 2 + 3)

Without an API key, Config C falls back to algorithmic zone assignment (= Config B).
Results contextualized against existing sweep results for interpretation.

Note: We do not compare against the old 37-dim checkpoint here because
batch_runner.py has been updated to 42-dim obs. The baseline vs persona-aware
DRQN comparison is already documented in the sweep results (0.724 vs 0.544).

Usage:
    # Offline (quantifies architecture, Layer 2 = algo fallback):
    python eval_pipeline_integration.py \\
        --checkpoint logs/drqn_llm_persona/drqn_torch_best.pt \\
        --scenario scenarios/enterprise_blizzard.json \\
        --severity moderate \\
        --seeds 42 43 44 45 46 \\
        --output-dir logs/pipeline_integration

    # With LLM key (quantifies Layer 2 contribution):
    python eval_pipeline_integration.py \\
        --checkpoint logs/drqn_llm_persona/drqn_torch_best.pt \\
        --scenario scenarios/enterprise_blizzard.json \\
        --severity moderate \\
        --api-key $GROQ_API_KEY \\
        --seeds 42 43 44 45 46 \\
        --output-dir logs/pipeline_integration
"""

import argparse
import json
import os
import random

import numpy as np

import config
from batch_runner import (
    _build_agents,
    _capacity_aware_goal_map,
    _build_drive_shelter_mapping,
    _DRQNPedController,
    run_single_simulation,
)
from evac_env import EvacEnv
from scenario_loader import load_scenario, temporary_config, _apply_disaster_rules
from zone_assignment import _build_zones, _assign_zone_shelters
from llm_zone_coordinator import LLMZoneCoordinator, build_coordinator_state


# ---------------------------------------------------------------------------
# Config B: standard run_single_simulation (algo zone + persona obs)
# ---------------------------------------------------------------------------

def _run_config_b(checkpoint, scenario_name, scenario_overrides, seed):
    _, summary = run_single_simulation(
        scenario_name=scenario_name,
        config_overrides=scenario_overrides,
        seed=seed,
        policy_name="drqn",
        draw=False,
        drqn_checkpoint=checkpoint,
    )
    return _extract_metrics(summary)


def _extract_metrics(summary):
    return {
        "reached_rate": float(summary.get("avg_reached_rate", summary.get("reached_rate", 0.0))),
        "exposure_total": float(summary.get("avg_exposure_total", summary.get("exposure_total", 0.0))),
        "ped_reached_rate": float(summary.get("avg_ped_reached_rate", summary.get("ped_reached_rate", 0.0))),
        "staff_reached_rate": float(summary.get("avg_staff_reached_rate", summary.get("staff_reached_rate", 0.0))),
        "faculty_reached_rate": float(summary.get("avg_faculty_reached_rate", summary.get("faculty_reached_rate", 0.0))),
    }


# ---------------------------------------------------------------------------
# Config C: LLM zone assignment → override ped goals → DRQN routing
# ---------------------------------------------------------------------------

def _zone_goal_map_from_llm(peds, zones_assigned, shelters_walk):
    node_to_shelter = {}
    for z in zones_assigned:
        primary = z.get("primary_shelter")
        if primary is None:
            continue
        for node in z.get("member_start_nodes", []):
            node_to_shelter[node] = primary
    return {a.id: (node_to_shelter.get(a.node) if node_to_shelter.get(a.node) in shelters_walk
                   else shelters_walk[0])
            for a in peds}


def _run_config_c(checkpoint, scenario_name, scenario_overrides, seed, api_key, num_zones):
    random.seed(seed)
    np.random.seed(seed)

    with temporary_config(scenario_overrides):
        env = EvacEnv()
        peds, cars, _ = _build_agents(env)
        shelters_walk = list(env.shelters)
        shelters_drive, drive_goal_to_shelter = _build_drive_shelter_mapping(env, shelters_walk)

        # Layer 2: LLM zone assignment
        zones = _build_zones(env, peds, num_zones=num_zones, seed=seed)
        state = build_coordinator_state(zones, env)
        coordinator = LLMZoneCoordinator(api_key=api_key, verbose=False)
        zones_assigned, llm_source = coordinator.assign(state, env, zones)

        ped_goals = _zone_goal_map_from_llm(peds, zones_assigned, shelters_walk)
        car_goals = _capacity_aware_goal_map(
            cars, env.G_drive, shelters_drive, "drqn", env,
            goal_to_shelter=drive_goal_to_shelter,
        )
        for a in peds:
            a.target_shelter = ped_goals.get(a.id)
        for a in cars:
            gn = car_goals.get(a.id)
            a.target_shelter = drive_goal_to_shelter.get(gn, gn)

        drqn_ctrl = _DRQNPedController(
            env=env, checkpoint_path=checkpoint, device="auto",
            max_neighbors=None, max_steps=config.EVAC_STEP_LIMIT,
        )
        drqn_ctrl.reset_agents(peds)
        for a in peds:
            if ped_goals.get(a.id):
                drqn_ctrl.init_goal(a.id, a.node, ped_goals[a.id])
        drqn_ctrl.update_interaction_state(peds, cars)

        for step in range(config.EVAC_STEP_LIMIT):
            drqn_ctrl.update_interaction_state(peds, cars)
            drqn_ctrl.reset_decision_step()
            for a in peds:
                if not a.alive or a.reached:
                    continue
                goal = ped_goals.get(a.id)
                if goal is None:
                    continue
                if getattr(a, "decision_delay_steps", 0) > 0:
                    a.decision_delay_steps -= 1
                    continue
                curr_target = drqn_ctrl.current_target(a.id, goal)
                nxt = drqn_ctrl.select_next_node(a, goal, step)
                if nxt is not None and nxt != a.node:
                    a.steps = getattr(a, "steps", 0) + 1
                    a.update_belief()
                    a.exposure += float(env.snow_depth_walk.get((a.node, nxt), 0.0))
                    a.node = nxt
                    # Advance checkpoint when current intermediate target is reached
                    curr_target = drqn_ctrl.current_target(a.id, goal)
                    if a.node == curr_target and curr_target != goal:
                        drqn_ctrl.advance_target(a.id, goal)
                    if a.node == goal:
                        a.reached = True
            env.step_hazards()
            if all(not ag.alive or ag.reached for ag in peds + cars):
                break

        total = len(peds) + len(cars)
        reached = sum(1 for a in peds + cars if a.reached)
        peds_r = sum(1 for a in peds if a.reached)
        staff_t = sum(1 for a in peds if getattr(a, "role", "") == "staff")
        fac_t = sum(1 for a in peds if getattr(a, "role", "") == "faculty")
        return {
            "reached_rate": reached / total if total > 0 else 0.0,
            "exposure_total": sum(a.exposure for a in peds + cars),
            "ped_reached_rate": peds_r / len(peds) if peds else 0.0,
            "staff_reached_rate": sum(1 for a in peds if a.reached and getattr(a, "role", "") == "staff") / staff_t if staff_t > 0 else 0.0,
            "faculty_reached_rate": sum(1 for a in peds if a.reached and getattr(a, "role", "") == "faculty") / fac_t if fac_t > 0 else 0.0,
            "llm_source": llm_source,
        }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _agg(rows, key):
    vals = [r[key] for r in rows if key in r]
    return float(np.mean(vals)) if vals else 0.0


def run_experiment(args):
    os.makedirs(args.output_dir, exist_ok=True)

    scenario = load_scenario(args.scenario)
    if args.severity:
        scenario["disaster_severity"] = args.severity
        scenario = _apply_disaster_rules(scenario)
    scenario_name = scenario.get("name", os.path.basename(args.scenario))
    scenario_overrides = scenario.get("config_overrides", {})

    print(f"[pipeline] scenario      : {args.scenario}")
    print(f"[pipeline] severity      : {args.severity or 'default'}")
    print(f"[pipeline] checkpoint    : {args.checkpoint}")
    print(f"[pipeline] seeds         : {args.seeds}")
    print(f"[pipeline] api_key       : {'set' if args.api_key else 'not set (C = B, algo fallback)'}")
    print()

    rows_b, rows_c = [], []

    for seed in args.seeds:
        print(f"  [seed {seed}] Config B (algo zone + persona-aware DRQN)...")
        r = _run_config_b(args.checkpoint, scenario_name, scenario_overrides, seed)
        rows_b.append(r)
        print(f"  [seed {seed}] B: reached={r['reached_rate']:.3f} exp={r['exposure_total']:.1f}")

        print(f"  [seed {seed}] Config C (LLM zone + persona-aware DRQN)...")
        r = _run_config_c(args.checkpoint, scenario_name, scenario_overrides, seed,
                          args.api_key, args.num_zones)
        rows_c.append(r)
        src = r.get("llm_source", "?")
        print(f"  [seed {seed}] C: reached={r['reached_rate']:.3f} exp={r['exposure_total']:.1f} [{src}]")

    keys = ["reached_rate", "exposure_total", "staff_reached_rate", "faculty_reached_rate"]
    agg_b = {k: _agg(rows_b, k) for k in keys}
    agg_c = {k: _agg(rows_c, k) for k in keys}
    llm_used = any(r.get("llm_source") == "llm" for r in rows_c)

    delta = agg_c["reached_rate"] - agg_b["reached_rate"]

    print()
    print("=" * 75)
    print("[pipeline] RESULTS SUMMARY")
    print("=" * 75)
    # Context from existing sweep results
    print("  [context] From llm_persona_sweep (blizzard moderate, 20 runs):")
    print("            reached_rate = 0.724 (this is the B-equivalent full sweep)")
    print()
    configs = [
        ("B", "Persona-aware DRQN + algo zone (Layer 1+3)", agg_b),
        ("C", f"Persona-aware DRQN + {'LLM' if llm_used else 'algo'} zone (Layer 1+2+3)", agg_c),
    ]
    print(f"{'Cfg':<3} {'Description':<46} {'Reached':>8} {'Exposure':>9} {'Staff':>7} {'Faculty':>8}")
    print("-" * 84)
    for cfg, desc, agg in configs:
        print(f"{cfg:<3} {desc:<46} {agg['reached_rate']:>8.3f} {agg['exposure_total']:>9.1f} {agg['staff_reached_rate']:>7.3f} {agg['faculty_reached_rate']:>8.3f}")

    print()
    print(f"  Layer 2 contribution (C vs B): Δreached = {delta:+.3f}")
    if not llm_used:
        print("  (offline: no API key — Layer 2 fell back to algorithmic zone assignment)")
        print("  To quantify Layer 2, rerun with: --api-key $GROQ_API_KEY")

    # Save
    output = {
        "scenario": args.scenario,
        "severity": args.severity,
        "checkpoint": args.checkpoint,
        "seeds": args.seeds,
        "llm_api_used": llm_used,
        "context_from_full_sweep": {
            "blizzard_moderate_20runs": 0.724,
            "source": "logs/llm_persona_sweep_blizzard/disaster_severity_sweep.json",
        },
        "configs": {
            "B": {"desc": "Algo zone + persona obs (Layer 1+3)", "rows": rows_b, "agg": agg_b},
            "C": {"desc": f"{'LLM' if llm_used else 'Algo fallback'} zone + persona obs (Layer 1+2+3)", "rows": rows_c, "agg": agg_c},
        },
        "deltas": {
            "layer_2_marginal_c_vs_b": delta,
        },
    }
    out_json = os.path.join(args.output_dir, "pipeline_integration_results.json")
    with open(out_json, "w") as f:
        json.dump(output, f, indent=2)

    md_path = os.path.join(args.output_dir, "pipeline_integration_summary.md")
    with open(md_path, "w") as f:
        f.write("# 3-Layer Pipeline Integration Results\n\n")
        f.write(f"**Scenario**: {args.scenario}  \n")
        f.write(f"**Severity**: {args.severity or 'default'}  \n")
        f.write(f"**Seeds**: {args.seeds}  \n")
        f.write(f"**Layer 2 LLM used**: {llm_used}  \n\n")
        f.write("## Context\n\n")
        f.write("From the full 20-run llm_persona_sweep (blizzard moderate): **reached_rate = 0.724**  \n")
        f.write("This represents the Layer 1+3 baseline (algo zone, persona-aware DRQN).  \n\n")
        f.write("## Results\n\n")
        f.write("| Config | Description | Reached Rate | Exposure | Staff RR | Faculty RR |\n")
        f.write("|--------|-------------|-------------|---------|---------|----------|\n")
        for cfg, desc, agg in configs:
            f.write(f"| {cfg} | {desc} | {agg['reached_rate']:.3f} | {agg['exposure_total']:.1f} | {agg['staff_reached_rate']:.3f} | {agg['faculty_reached_rate']:.3f} |\n")
        f.write(f"\n**Layer 2 contribution (C vs B)**: Δreached = {delta:+.3f}  \n")
        if not llm_used:
            f.write("\n> ⚠ Offline run: no API key — Layer 2 used algorithmic fallback (C ≈ B).  \n")
            f.write("> To quantify true Layer 2 contribution, rerun with `--api-key $GROQ_API_KEY`.  \n")

    print(f"\n[pipeline] JSON : {out_json}")
    print(f"[pipeline] MD   : {md_path}")


def main():
    parser = argparse.ArgumentParser(description="Step 22: 3-layer pipeline integration")
    parser.add_argument("--checkpoint", default="logs/drqn_llm_persona/drqn_torch_best.pt")
    parser.add_argument("--scenario", default="scenarios/enterprise_blizzard.json")
    parser.add_argument("--severity", default="moderate")
    parser.add_argument("--seeds", nargs="+", type=int, default=[42, 43, 44])
    parser.add_argument("--num-zones", type=int, default=6)
    parser.add_argument("--api-key", default=os.environ.get("GROQ_API_KEY", ""))
    parser.add_argument("--output-dir", default="logs/pipeline_integration")
    args = parser.parse_args()
    run_experiment(args)


if __name__ == "__main__":
    main()
