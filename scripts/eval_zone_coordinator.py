import sys, os; sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
"""
eval_zone_coordinator.py

Evaluate LLM Zone Coordinator (Layer 2) vs algorithmic assignment.

Metrics compared:
  - avg_primary_distance   : average travel distance from zone members to assigned shelter
  - load_balance_std       : std of demand assigned per shelter (lower = better balanced)
  - shelter_diversity      : number of distinct shelters used as primary
  - backup_coverage        : fraction of zones with a valid backup shelter
  - invalid_assignments    : zones where LLM assigned a non-existent shelter (fallback count)
  - reasoning_quality      : (LLM only) fraction of zones with non-empty reasoning

Runs multiple seeds for statistical reliability.

Usage:
    python eval_zone_coordinator.py \\
        --scenario scenarios/enterprise_blizzard.json \\
        --severity moderate \\
        --api-key YOUR_GROQ_KEY \\
        --seeds 42 43 44 45 46 \\
        --num-zones 6 \\
        --output-dir logs/zone_eval
"""

import argparse
import json
import os
import random

import networkx as nx
import numpy as np

import config
from evac_env import EvacEnv
from scenario_loader import load_scenario, temporary_config, _apply_disaster_rules
from zone_assignment import _build_zones, _build_ped_population, _assign_zone_shelters
from llm_zone_coordinator import LLMZoneCoordinator, build_coordinator_state


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def _avg_primary_distance(zones, env):
    """Average graph distance from each zone member to its assigned primary shelter."""
    distances = []
    for z in zones:
        primary = z.get("primary_shelter")
        if primary is None:
            continue
        for node in z.get("member_start_nodes", []):
            try:
                d = nx.shortest_path_length(env.G_walk, node, primary, weight="weight")
                distances.append(d)
            except Exception:
                pass
    return float(np.mean(distances)) if distances else float("inf")


def _load_balance_std(zones):
    """Std of demand assigned to each primary shelter (lower = better balanced)."""
    from collections import defaultdict
    shelter_demand = defaultdict(int)
    for z in zones:
        primary = z.get("primary_shelter")
        if primary is not None:
            shelter_demand[primary] += z["demand"]
    if not shelter_demand:
        return 0.0
    vals = list(shelter_demand.values())
    return float(np.std(vals))


def _shelter_diversity(zones):
    """Number of distinct primary shelters used."""
    return len(set(z.get("primary_shelter") for z in zones if z.get("primary_shelter") is not None))


def _backup_coverage(zones):
    """Fraction of zones with a valid backup shelter different from primary."""
    if not zones:
        return 0.0
    covered = sum(
        1 for z in zones
        if z.get("backup_shelter") is not None
        and z.get("backup_shelter") != z.get("primary_shelter")
    )
    return covered / len(zones)


def _invalid_assignments(zones, env):
    """Fraction of zones where the assigned shelter is not in env.shelters."""
    shelters = set(env.shelters)
    invalid = sum(1 for z in zones if z.get("primary_shelter") not in shelters)
    return invalid / len(zones) if zones else 0.0


def _reasoning_quality(zones):
    """Fraction of zones with non-empty, non-fallback reasoning (LLM only)."""
    meaningful = sum(
        1 for z in zones
        if z.get("llm_reasoning")
        and "fallback" not in str(z.get("llm_reasoning", "")).lower()
        and len(str(z.get("llm_reasoning", ""))) > 10
    )
    return meaningful / len(zones) if zones else 0.0


def compute_metrics(zones, env):
    return {
        "avg_primary_distance_m": round(_avg_primary_distance(zones, env), 1),
        "load_balance_std":        round(_load_balance_std(zones), 2),
        "shelter_diversity":       _shelter_diversity(zones),
        "backup_coverage":         round(_backup_coverage(zones), 3),
        "invalid_assignments":     round(_invalid_assignments(zones, env), 3),
        "reasoning_quality":       round(_reasoning_quality(zones), 3),
    }


# ---------------------------------------------------------------------------
# Single run
# ---------------------------------------------------------------------------

def run_one_seed(env, scenario, seed, num_zones, api_key, verbose):
    random.seed(seed)
    np.random.seed(seed)

    peds = _build_ped_population(env, seed=seed)
    zones_base = _build_zones(env, peds, num_zones=num_zones, seed=seed)

    # --- Algorithmic ---
    import copy
    zones_algo = copy.deepcopy(zones_base)
    _assign_zone_shelters(zones_algo, list(env.shelters), env)
    for z in zones_algo:
        z.setdefault("llm_reasoning", "algorithmic")

    # --- LLM ---
    zones_llm = copy.deepcopy(zones_base)
    state = build_coordinator_state(
        zones_llm, env,
        disaster_type=scenario.get("disaster_type", "unknown"),
        severity=scenario.get("disaster_severity", "unknown"),
    )
    coordinator = LLMZoneCoordinator(api_key=api_key, verbose=verbose)
    zones_llm, source = coordinator.assign(state, env, zones_llm)

    metrics_algo = compute_metrics(zones_algo, env)
    metrics_llm  = compute_metrics(zones_llm, env)
    metrics_llm["assignment_source"] = source

    return metrics_algo, metrics_llm, zones_algo, zones_llm


# ---------------------------------------------------------------------------
# Aggregate across seeds
# ---------------------------------------------------------------------------

def aggregate(records):
    if not records:
        return {}
    keys = [k for k in records[0] if k != "assignment_source"]
    out = {}
    for k in keys:
        vals = [r[k] for r in records if r.get(k) is not None]
        out[f"avg_{k}"] = round(float(np.mean(vals)), 3) if vals else None
        out[f"std_{k}"] = round(float(np.std(vals)), 3) if vals else None
    sources = [r.get("assignment_source", "?") for r in records]
    llm_count = sum(1 for s in sources if s == "llm")
    out["llm_rate"] = round(llm_count / len(sources), 2) if sources else 0.0
    return out


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def print_report(algo_agg, llm_agg, seeds, output_dir):
    lines = []
    lines.append("# Layer 2 Evaluation: LLM Zone Coordinator vs Algorithmic Assignment\n")
    lines.append(f"Seeds: {seeds}  |  Runs per method: {len(seeds)}\n")

    if llm_agg.get("llm_rate") is not None:
        llm_pct = llm_agg['llm_rate'] * 100
        algo_pct = 100 - llm_pct
        lines.append(f"LLM coordinator success rate: **{llm_pct:.0f}%** "
                     f"(fell back to algorithmic {algo_pct:.0f}% of runs)\n")

    lines.append("## Metrics Comparison\n")
    lines.append("| Metric | Algorithmic | LLM Coordinator | Winner |")
    lines.append("|--------|-------------|-----------------|--------|")

    metrics_info = [
        ("avg_primary_distance_m", "Avg distance to shelter (m)", "lower"),
        ("load_balance_std",       "Load balance std (demand/shelter)", "lower"),
        ("shelter_diversity",      "Distinct shelters used",       "higher"),
        ("backup_coverage",        "Backup shelter coverage",      "higher"),
        ("invalid_assignments",    "Invalid assignments (fraction)", "lower"),
        ("reasoning_quality",      "Reasoning quality (LLM only)", "higher"),
    ]

    for key, label, better in metrics_info:
        a_val = algo_agg.get(f"avg_{key}")
        l_val = llm_agg.get(f"avg_{key}")
        a_std = algo_agg.get(f"std_{key}", 0.0) or 0.0
        l_std = llm_agg.get(f"std_{key}", 0.0) or 0.0

        fmt = lambda v, s: f"{v:.1f} ±{s:.1f}" if v is not None else "—"
        a_str = fmt(a_val, a_std)
        l_str = fmt(l_val, l_std)

        # Determine winner
        if a_val is None or l_val is None:
            winner = "—"
        elif better == "lower":
            if abs(a_val - l_val) < 0.01 * max(abs(a_val), abs(l_val), 1):
                winner = "tie"
            else:
                winner = "Algo" if a_val < l_val else "LLM"
        else:
            if abs(a_val - l_val) < 0.01 * max(abs(a_val), abs(l_val), 1):
                winner = "tie"
            else:
                winner = "LLM" if l_val > a_val else "Algo"

        lines.append(f"| {label} | {a_str} | {l_str} | {winner} |")

    lines.append("")
    lines.append("## Interpretation\n")
    lines.append("- **avg_primary_distance_m**: lower means agents walk less to reach their shelter")
    lines.append("- **load_balance_std**: lower means demand is more evenly spread across shelters")
    lines.append("- **shelter_diversity**: higher means the LLM uses more shelters (avoids overcrowding)")
    lines.append("- **backup_coverage**: higher means more zones have a fallback if primary is full/blocked")
    lines.append("- **invalid_assignments**: higher means LLM hallucinated shelter IDs (0 = perfect)")
    lines.append("- **reasoning_quality**: fraction of zones where LLM provided meaningful reasoning")
    lines.append("")
    lines.append("## Notes\n")
    lines.append("- Algorithmic method uses demand-balanced greedy assignment with capacity awareness")
    lines.append("- LLM method uses ReAct loop with 4 tools; falls back to algorithmic if API unavailable")
    lines.append("- Both methods start from the same zone clustering (k-means on agent positions)")

    md = "\n".join(lines)
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, "zone_eval_report.md")
    with open(path, "w") as f:
        f.write(md)
    print(f"\n[eval] report saved → {path}")
    print("\n" + md)
    return md


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Evaluate LLM Zone Coordinator vs algorithmic")
    parser.add_argument("--scenario", default="scenarios/enterprise_blizzard.json")
    parser.add_argument("--severity", default="moderate",
                        choices=["light", "moderate", "severe", "extreme"])
    parser.add_argument("--api-key", default="")
    parser.add_argument("--seeds", nargs="+", type=int, default=[42, 43, 44, 45, 46])
    parser.add_argument("--num-zones", type=int, default=6)
    parser.add_argument("--output-dir", default="logs/zone_eval")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    api_key = args.api_key or os.environ.get("GROQ_API_KEY", "")

    scenario = load_scenario(args.scenario)
    scenario["disaster_severity"] = args.severity
    scenario = _apply_disaster_rules(scenario)

    print(f"[eval] scenario: {scenario.get('name')}  severity: {args.severity}")
    print(f"[eval] LLM: {'Llama 3.3 70B via Groq' if api_key else 'offline (algorithmic fallback)'}")
    print(f"[eval] seeds: {args.seeds}  num_zones: {args.num_zones}")

    algo_records = []
    llm_records  = []
    all_results  = []

    with temporary_config(scenario.get("config_overrides", {})):
        env = EvacEnv()
        for seed in args.seeds:
            print(f"\n[eval] seed {seed} ...", end=" ", flush=True)
            m_algo, m_llm, zones_algo, zones_llm = run_one_seed(
                env, scenario, seed, args.num_zones, api_key, args.verbose
            )
            algo_records.append(m_algo)
            llm_records.append(m_llm)
            print(f"algo_dist={m_algo['avg_primary_distance_m']:.0f}m  "
                  f"llm_dist={m_llm['avg_primary_distance_m']:.0f}m  "
                  f"source={m_llm.get('assignment_source','?')}")
            all_results.append({
                "seed": seed,
                "algorithmic": m_algo,
                "llm": m_llm,
            })

    algo_agg = aggregate(algo_records)
    llm_agg  = aggregate(llm_records)

    # Save JSON
    os.makedirs(args.output_dir, exist_ok=True)
    json_path = os.path.join(args.output_dir, "zone_eval_results.json")
    with open(json_path, "w") as f:
        json.dump({
            "scenario": scenario.get("name"),
            "severity": args.severity,
            "seeds": args.seeds,
            "num_zones": args.num_zones,
            "algorithmic_aggregate": algo_agg,
            "llm_aggregate": llm_agg,
            "per_seed": all_results,
        }, f, indent=2)
    print(f"\n[eval] json saved → {json_path}")

    print_report(algo_agg, llm_agg, args.seeds, args.output_dir)


if __name__ == "__main__":
    main()
