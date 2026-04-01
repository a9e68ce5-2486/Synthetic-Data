"""
analyze_reachability.py

Diagnose WHY reached_rate drops under high disaster severity.

For each severity level, this script:
  1. Builds the environment (with the severity's hazard config applied)
  2. Samples agent starting positions (same logic as batch_runner)
  3. For each agent start, checks:
       - UNREACHABLE : no path exists from start to any shelter (graph disconnected)
       - BUDGET_LIMITED : path exists but shortest_dist > step_budget_max / step_budget_scale
                          (agent would run out of steps even with a perfect policy)
       - POLICY_SOLVABLE: path exists and is within step budget (policy is the bottleneck)
  4. Reports per-severity breakdown and path-length percentiles

Usage:
    venv/bin/python analyze_reachability.py \
        --scenario scenarios/enterprise_blizzard.json \
        --checkpoint logs/drqn_progressive_severity/drqn_torch_best.pt \
        --num-seeds 10 \
        --output-dir logs/reachability_analysis
"""
import argparse
import csv
import json
import os
import random

import networkx as nx
import numpy as np

import config as cfg_module
from evac_env import EvacEnv
from scenario_loader import _apply_disaster_rules, load_scenario, temporary_config

SEVERITY_LEVELS = ["light", "moderate", "severe", "extreme"]


def _load_step_budget_params(checkpoint_path):
    """Extract step-budget parameters from a DRQN checkpoint."""
    defaults = {
        "dynamic_step_budget": True,
        "step_budget_scale": 0.08,
        "step_budget_min": 100,
        "step_budget_max": 1000,
        "step_budget_slack": 20,
    }
    if not checkpoint_path or not os.path.exists(checkpoint_path):
        return defaults
    try:
        import torch
        payload = torch.load(checkpoint_path, map_location="cpu")
        return {
            "dynamic_step_budget": bool(payload.get("dynamic_step_budget", defaults["dynamic_step_budget"])),
            "step_budget_scale": float(payload.get("step_budget_scale", defaults["step_budget_scale"])),
            "step_budget_min": int(payload.get("step_budget_min", defaults["step_budget_min"])),
            "step_budget_max": int(payload.get("step_budget_max", defaults["step_budget_max"])),
            "step_budget_slack": float(payload.get("step_budget_slack", defaults["step_budget_slack"])),
        }
    except Exception as e:
        print(f"[warn] could not load checkpoint params: {e}, using defaults")
        return defaults


def _effective_step_budget(dist, params):
    """Return the step budget an agent at 'dist' from shelter would receive."""
    if not params["dynamic_step_budget"]:
        return params["step_budget_max"]
    raw = params["step_budget_scale"] * dist + params["step_budget_slack"]
    return int(max(params["step_budget_min"], min(params["step_budget_max"], raw)))


def _walk_graph_unblocked(env):
    """Return the sub-graph with all initially-blocked edges removed."""
    blocked = set(env.blocked_edges_walk)
    if not blocked:
        return env.G_walk
    keep = [(u, v) for u, v in env.G_walk.edges() if (u, v) not in blocked]
    return env.G_walk.edge_subgraph(keep).copy()


def analyze_one_seed(env, budget_params, num_peds, seed):
    """
    Sample 'num_peds' pedestrian start positions (same as batch_runner),
    then classify each start as UNREACHABLE / BUDGET_LIMITED / POLICY_SOLVABLE.
    Returns a list of per-agent dicts.
    """
    random.seed(seed)
    np.random.seed(seed)

    shelters = list(env.shelters)
    if not shelters:
        return []

    graph = _walk_graph_unblocked(env)
    nodes_walk = list(env.G_walk.nodes())
    records = []

    for _ in range(num_peds):
        start = random.choice(nodes_walk)

        # Find shortest distance to any shelter (on unblocked graph)
        best_dist = float("inf")
        best_shelter = None
        reachable = False
        for s in shelters:
            try:
                d = nx.shortest_path_length(graph, start, s, weight="weight")
                if d < best_dist:
                    best_dist = d
                    best_shelter = s
                    reachable = True
            except nx.NetworkXNoPath:
                pass
            except nx.NodeNotFound:
                pass

        if not reachable:
            category = "unreachable"
            budget = None
            dist = None
        else:
            dist = best_dist
            budget = _effective_step_budget(dist, budget_params)
            # Rough heuristic: minimum steps needed ≈ dist / EVAC_SPEED_WALK
            # (each step the agent moves ~EVAC_SPEED_WALK metres along the graph)
            min_steps_needed = dist / max(1.0, getattr(cfg_module, "EVAC_SPEED_WALK", 1.4))
            if min_steps_needed > budget:
                category = "budget_limited"
            else:
                category = "policy_solvable"

        records.append({
            "start": start,
            "best_shelter": best_shelter,
            "dist_to_shelter": round(dist, 1) if dist is not None else None,
            "step_budget": budget,
            "category": category,
        })

    return records


def analyze_severity(scenario_base, severity, budget_params, num_seeds, num_peds_per_seed, ped_count):
    scenario = dict(scenario_base)
    scenario["disaster_severity"] = severity
    scenario["name"] = f"{scenario_base.get('name', 'scenario')}_{severity}"
    scenario = _apply_disaster_rules(scenario)
    overrides = dict(scenario.get("config_overrides", {}))

    all_records = []
    for seed in range(num_seeds):
        with temporary_config({**overrides, "EVAC_USE_OSM": True}):
            random.seed(seed * 7 + 13)
            env = EvacEnv()
        records = analyze_one_seed(env, budget_params, ped_count, seed=seed * 100 + 42)
        all_records.extend(records)

    total = len(all_records)
    if total == 0:
        return {}

    unreachable = [r for r in all_records if r["category"] == "unreachable"]
    budget_lim  = [r for r in all_records if r["category"] == "budget_limited"]
    solvable    = [r for r in all_records if r["category"] == "policy_solvable"]

    dists = [r["dist_to_shelter"] for r in all_records if r["dist_to_shelter"] is not None]
    budgets = [r["step_budget"] for r in all_records if r["step_budget"] is not None]

    return {
        "severity": severity,
        "total_agents": total,
        "unreachable_n": len(unreachable),
        "budget_limited_n": len(budget_lim),
        "policy_solvable_n": len(solvable),
        "unreachable_pct": round(100 * len(unreachable) / total, 2),
        "budget_limited_pct": round(100 * len(budget_lim) / total, 2),
        "policy_solvable_pct": round(100 * len(solvable) / total, 2),
        "dist_p50": round(float(np.percentile(dists, 50)), 1) if dists else None,
        "dist_p75": round(float(np.percentile(dists, 75)), 1) if dists else None,
        "dist_p90": round(float(np.percentile(dists, 90)), 1) if dists else None,
        "dist_p95": round(float(np.percentile(dists, 95)), 1) if dists else None,
        "dist_max":  round(float(np.max(dists)), 1) if dists else None,
        "budget_p50": round(float(np.percentile(budgets, 50)), 1) if budgets else None,
        "budget_p95": round(float(np.percentile(budgets, 95)), 1) if budgets else None,
        "num_seeds": num_seeds,
    }


def print_table(results):
    header = (
        f"{'severity':10} {'unreachable':>13} {'budget_lim':>12} {'solvable':>10} "
        f"{'dist_p50':>10} {'dist_p90':>10} {'dist_p95':>10} {'budget_p50':>12}"
    )
    print()
    print(header)
    print("-" * len(header))
    for r in results:
        print(
            f"{r['severity']:10} "
            f"{r['unreachable_pct']:>11.1f}%  "
            f"{r['budget_limited_pct']:>10.1f}%  "
            f"{r['policy_solvable_pct']:>8.1f}%  "
            f"{str(r['dist_p50']):>10}  "
            f"{str(r['dist_p90']):>10}  "
            f"{str(r['dist_p95']):>10}  "
            f"{str(r['budget_p50']):>12}"
        )
    print()


def main():
    parser = argparse.ArgumentParser(description="Reachability analysis across disaster severity levels")
    parser.add_argument("--scenario", required=True, help="Base scenario JSON, e.g. scenarios/enterprise_blizzard.json")
    parser.add_argument("--checkpoint", default=None, help="DRQN checkpoint to read step-budget params from")
    parser.add_argument("--num-seeds", type=int, default=10, help="Number of environment seeds per severity level")
    parser.add_argument("--ped-count", type=int, default=None,
                        help="Pedestrians to sample per seed (default: EVAC_PED_COUNT from config)")
    parser.add_argument("--severities", nargs="+", default=SEVERITY_LEVELS,
                        choices=SEVERITY_LEVELS, help="Severity levels to analyse")
    parser.add_argument("--output-dir", default="logs/reachability_analysis")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    ped_count = args.ped_count or getattr(cfg_module, "EVAC_PED_COUNT", 40)
    budget_params = _load_step_budget_params(args.checkpoint)

    print(f"[reachability] scenario  : {args.scenario}")
    print(f"[reachability] checkpoint: {args.checkpoint}")
    print(f"[reachability] seeds/severity: {args.num_seeds}  peds/seed: {ped_count}")
    print(f"[reachability] step_budget params: {budget_params}")

    base_scenario = load_scenario(args.scenario)
    results = []

    for severity in args.severities:
        print(f"\n[reachability] analysing severity={severity} ...", flush=True)
        stats = analyze_severity(
            base_scenario, severity, budget_params,
            num_seeds=args.num_seeds,
            num_peds_per_seed=ped_count,
            ped_count=ped_count,
        )
        if stats:
            results.append(stats)
            print(
                f"  unreachable={stats['unreachable_pct']:.1f}%  "
                f"budget_limited={stats['budget_limited_pct']:.1f}%  "
                f"policy_solvable={stats['policy_solvable_pct']:.1f}%  "
                f"dist_p90={stats['dist_p90']}m  budget_p50={stats['budget_p50']} steps"
            )

    print_table(results)

    csv_path  = os.path.join(args.output_dir, "reachability_analysis.csv")
    json_path = os.path.join(args.output_dir, "reachability_analysis.json")
    md_path   = os.path.join(args.output_dir, "reachability_analysis.md")

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        if results:
            writer = csv.DictWriter(f, fieldnames=list(results[0].keys()))
            writer.writeheader()
            writer.writerows(results)

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump({
            "scenario": args.scenario,
            "checkpoint": args.checkpoint,
            "budget_params": budget_params,
            "ped_count": ped_count,
            "num_seeds": args.num_seeds,
            "results": results,
        }, f, indent=2)

    with open(md_path, "w", encoding="utf-8") as f:
        f.write("# Reachability Analysis\n\n")
        f.write(f"- Scenario: `{args.scenario}`\n")
        f.write(f"- Checkpoint: `{args.checkpoint}`\n")
        f.write(f"- Seeds per severity: {args.num_seeds}, peds per seed: {ped_count}\n")
        f.write(f"- Step-budget params: `{budget_params}`\n\n")
        f.write("## Agent Classification per Severity\n\n")
        f.write("| Severity | Unreachable | Budget-limited | Policy-solvable | dist_p50 | dist_p90 | dist_p95 | budget_p50 |\n")
        f.write("|----------|------------|----------------|-----------------|----------|----------|----------|------------|\n")
        for r in results:
            f.write(
                f"| {r['severity']} "
                f"| {r['unreachable_pct']:.1f}% "
                f"| {r['budget_limited_pct']:.1f}% "
                f"| {r['policy_solvable_pct']:.1f}% "
                f"| {r['dist_p50']}m "
                f"| {r['dist_p90']}m "
                f"| {r['dist_p95']}m "
                f"| {r['budget_p50']} steps |\n"
            )
        f.write("\n## Interpretation Guide\n\n")
        f.write("- **Unreachable**: No path from start to any shelter on the (initially) unblocked graph.\n")
        f.write("  → Fix: reduce `EVAC_BLOCK_INIT_PROB` or review shelter placement.\n\n")
        f.write("- **Budget-limited**: Path exists, but minimum steps needed exceed `step_budget_max`.\n")
        f.write("  → Fix: increase `step_budget_max` or `step_budget_scale` in DRQN training.\n\n")
        f.write("- **Policy-solvable**: Path exists and is within budget — DRQN policy is the bottleneck.\n")
        f.write("  → Fix: improve DRQN policy (longer training, HER, better obs features).\n\n")

    print(f"[reachability] csv : {csv_path}")
    print(f"[reachability] json: {json_path}")
    print(f"[reachability] md  : {md_path}")


if __name__ == "__main__":
    main()
