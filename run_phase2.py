import argparse
import csv
import json
import os

from batch_runner import run_batch
from scenario_loader import load_scenario


PHASE2_SCENARIOS = [
    "scenarios/enterprise_baseline.json",
    "scenarios/enterprise_blizzard_mid.json",
    "scenarios/enterprise_earthquake.json",
    "scenarios/enterprise_compound.json",
]


def _safe_improvement(new_value, baseline_value):
    if baseline_value in (None, 0):
        return None
    return (new_value - baseline_value) / baseline_value


def _choose_best_policy(policy_summary):
    best_name = None
    best_score = None
    for name, stats in policy_summary.items():
        score = (
            stats.get("avg_reached_rate", 0.0),
            -stats.get("avg_exposure_total", 0.0),
        )
        if best_score is None or score > best_score:
            best_score = score
            best_name = name
    return best_name


def run_phase2(output_root, scenarios, dry_run=False):
    os.makedirs(output_root, exist_ok=True)

    rows = []
    for scenario_path in scenarios:
        scenario = load_scenario(scenario_path)
        scenario_name = scenario["name"]
        scenario_out = os.path.join(output_root, scenario_name)
        os.makedirs(scenario_out, exist_ok=True)

        print(f"[phase2] scenario={scenario_name}")
        if dry_run:
            print(f"[phase2] dry-run skip execution: {scenario_path}")
            continue

        _, _, _, policy_summary = run_batch(scenario, scenario_out)

        baseline_name = scenario.get("baseline_policy", "round_robin")
        if baseline_name not in policy_summary:
            baseline_name = next(iter(policy_summary.keys()))
        baseline = policy_summary[baseline_name]

        best_policy = _choose_best_policy(policy_summary)
        for policy_name, stats in policy_summary.items():
            rows.append(
                {
                    "scenario": scenario_name,
                    "disaster_type": scenario.get("disaster_type", "custom"),
                    "policy": policy_name,
                    "is_best_policy": policy_name == best_policy,
                    "runs": stats.get("runs"),
                    "avg_reached_rate": stats.get("avg_reached_rate"),
                    "avg_alive_rate": stats.get("avg_alive_rate"),
                    "avg_exposure_total": stats.get("avg_exposure_total"),
                    "avg_t95_step": stats.get("avg_t95_step"),
                    "baseline_policy": baseline_name,
                    "reached_rate_improve_vs_baseline": _safe_improvement(
                        stats.get("avg_reached_rate"), baseline.get("avg_reached_rate")
                    ),
                    "exposure_improve_vs_baseline": _safe_improvement(
                        baseline.get("avg_exposure_total"), stats.get("avg_exposure_total")
                    ),
                }
            )

    csv_path = os.path.join(output_root, "phase2_policy_comparison.csv")
    json_path = os.path.join(output_root, "phase2_policy_comparison.json")
    md_path = os.path.join(output_root, "phase2_policy_comparison.md")

    if rows:
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(rows, f, indent=2)
        with open(md_path, "w", encoding="utf-8") as f:
            f.write("# Phase 2 Policy Comparison\n\n")
            for scenario_name in sorted(set(r["scenario"] for r in rows)):
                f.write(f"## {scenario_name}\n\n")
                subset = [r for r in rows if r["scenario"] == scenario_name]
                best = [r for r in subset if r["is_best_policy"]]
                if best:
                    f.write(
                        f"- Best policy: `{best[0]['policy']}` "
                        f"(avg_reached_rate={best[0]['avg_reached_rate']}, "
                        f"avg_exposure_total={best[0]['avg_exposure_total']})\n"
                    )
                for r in subset:
                    f.write(
                        f"- {r['policy']}: reached={r['avg_reached_rate']}, alive={r['avg_alive_rate']}, "
                        f"exposure={r['avg_exposure_total']}, t95={r['avg_t95_step']}, "
                        f"reach_improve_vs_baseline={r['reached_rate_improve_vs_baseline']}\n"
                    )
                f.write("\n")

        print(f"[phase2] comparison csv: {csv_path}")
        print(f"[phase2] comparison json: {json_path}")
        print(f"[phase2] comparison md: {md_path}")
    else:
        print("[phase2] no completed scenario results; comparison files were not generated.")


def main():
    parser = argparse.ArgumentParser(description="Run Phase 2 frozen scenario suite")
    parser.add_argument("--output-root", default="logs/phase2")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--scenarios", nargs="*", default=PHASE2_SCENARIOS)
    args = parser.parse_args()
    run_phase2(args.output_root, args.scenarios, dry_run=args.dry_run)


if __name__ == "__main__":
    main()
