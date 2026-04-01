import argparse
import csv
import json
import os

from batch_runner import run_batch
from management_report import build_management_summary
from scenario_loader import load_scenario


def _safe_improvement(new_value, baseline_value):
    if baseline_value in (None, 0):
        return None
    return (new_value - baseline_value) / baseline_value


def _score_policy(summary):
    return (
        summary.get("avg_reached_rate", 0.0),
        -summary.get("avg_exposure_total", float("inf")),
    )


def main():
    parser = argparse.ArgumentParser(description="Compare heuristic baselines vs DRQN under one scenario.")
    parser.add_argument("--scenario", default="scenarios/enterprise_baseline.json")
    parser.add_argument("--output-dir", default="logs/baselines_vs_drqn")
    parser.add_argument("--drqn-checkpoint", default="logs/drqn_blocked_finetune/drqn_torch_best.pt")
    parser.add_argument("--baseline-policy", default="round_robin")
    parser.add_argument("--num-runs", type=int, default=None)
    parser.add_argument("--base-seed", type=int, default=None)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    scenario = load_scenario(args.scenario)
    policies = [p for p in scenario.get("policies", []) if p != "priority_faculty"]
    if "drqn" not in policies:
        policies.append("drqn")
    scenario["policies"] = policies
    if args.num_runs is not None:
        scenario["num_runs"] = int(args.num_runs)
    if args.base_seed is not None:
        scenario["base_seed"] = int(args.base_seed)

    _, _, _, policy_summary = run_batch(
        scenario=scenario,
        output_dir=args.output_dir,
        drqn_checkpoint=args.drqn_checkpoint,
    )

    baseline_name = args.baseline_policy
    if baseline_name not in policy_summary:
        baseline_name = next(iter(policy_summary.keys()))
    baseline = policy_summary[baseline_name]
    best_policy = max(policy_summary.items(), key=lambda kv: _score_policy(kv[1]))[0]

    rows = []
    for policy_name, stats in policy_summary.items():
        rows.append(
            {
                "scenario": scenario.get("name", "scenario"),
                "policy": policy_name,
                "is_best_policy": policy_name == best_policy,
                "runs": stats.get("runs"),
                "avg_reached_rate": stats.get("avg_reached_rate"),
                "avg_alive_rate": stats.get("avg_alive_rate"),
                "avg_exposure_total": stats.get("avg_exposure_total"),
                "avg_t90_step": stats.get("avg_t90_step"),
                "avg_t95_step": stats.get("avg_t95_step"),
                "avg_faculty_reached_rate": stats.get("avg_faculty_reached_rate"),
                "avg_staff_reached_rate": stats.get("avg_staff_reached_rate"),
                "avg_reach_rate_gap": stats.get("avg_reach_rate_gap"),
                "baseline_policy": baseline_name,
                "reached_rate_improve_vs_baseline": _safe_improvement(
                    stats.get("avg_reached_rate"), baseline.get("avg_reached_rate")
                ),
                "exposure_improve_vs_baseline": _safe_improvement(
                    baseline.get("avg_exposure_total"), stats.get("avg_exposure_total")
                ),
            }
        )

    csv_path = os.path.join(args.output_dir, "baselines_vs_drqn_comparison.csv")
    json_path = os.path.join(args.output_dir, "baselines_vs_drqn_comparison.json")
    md_path = os.path.join(args.output_dir, "baselines_vs_drqn_comparison.md")
    mgmt_path = os.path.join(args.output_dir, "baselines_vs_drqn_management_summary.txt")

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(
            {
                "scenario": scenario.get("name", "scenario"),
                "drqn_checkpoint": args.drqn_checkpoint,
                "baseline_policy": baseline_name,
                "best_policy": best_policy,
                "policy_summary": policy_summary,
                "rows": rows,
            },
            f,
            indent=2,
        )

    with open(md_path, "w", encoding="utf-8") as f:
        f.write("# Baselines vs DRQN Comparison\n\n")
        f.write(f"- Scenario: `{scenario.get('name', 'scenario')}`\n")
        f.write(f"- DRQN checkpoint: `{args.drqn_checkpoint}`\n")
        f.write(f"- Baseline policy: `{baseline_name}`\n")
        f.write(f"- Best policy: `{best_policy}`\n\n")
        for row in rows:
            f.write(
                f"- `{row['policy']}`: reached={row['avg_reached_rate']}, "
                f"alive={row['avg_alive_rate']}, exposure={row['avg_exposure_total']}, "
                f"t90={row['avg_t90_step']}, t95={row['avg_t95_step']}, "
                f"gap={row['avg_reach_rate_gap']}, "
                f"reach_improve_vs_baseline={row['reached_rate_improve_vs_baseline']}\n"
            )

    management_summary = build_management_summary(
        policy_summary=policy_summary,
        scenario_name=scenario.get("name", "scenario"),
        baseline_policy=baseline_name,
    )
    with open(mgmt_path, "w", encoding="utf-8") as f:
        f.write(management_summary)
        f.write("\n")

    print(f"[compare] csv: {csv_path}")
    print(f"[compare] json: {json_path}")
    print(f"[compare] md: {md_path}")
    print(f"[compare] management: {mgmt_path}")
    print(f"[compare] best_policy: {best_policy}")


if __name__ == "__main__":
    main()
