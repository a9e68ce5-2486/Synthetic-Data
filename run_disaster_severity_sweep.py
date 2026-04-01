import argparse
import csv
import json
import os

from batch_runner import run_batch
from scenario_loader import _apply_disaster_rules, load_scenario


SEVERITY_LEVELS = ["light", "moderate", "severe", "extreme"]


def main():
    parser = argparse.ArgumentParser(
        description="Sweep disaster severity levels for one disaster type and summarize KPI changes."
    )
    parser.add_argument("--scenario", required=True, help="Base scenario path, e.g. scenarios/enterprise_blizzard.json")
    parser.add_argument("--output-dir", default="logs/disaster_severity_sweep")
    parser.add_argument("--drqn-checkpoint", default="logs/drqn_blocked_finetune/drqn_torch_best.pt")
    parser.add_argument("--policies", nargs="+", default=["drqn"])
    parser.add_argument("--num-runs", type=int, default=None)
    parser.add_argument("--base-seed", type=int, default=None)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    base_scenario = load_scenario(args.scenario)
    rows = []

    for severity in SEVERITY_LEVELS:
        scenario = dict(base_scenario)
        scenario["disaster_severity"] = severity
        scenario["policies"] = list(args.policies)
        scenario["config_overrides"] = dict(base_scenario.get("config_overrides", {}))
        scenario["name"] = f"{base_scenario.get('name', 'scenario')}_{severity}"
        if args.num_runs is not None:
            scenario["num_runs"] = int(args.num_runs)
        if args.base_seed is not None:
            scenario["base_seed"] = int(args.base_seed)
        scenario = _apply_disaster_rules(scenario)

        scenario_out_dir = os.path.join(args.output_dir, scenario["name"])
        _, _, _, policy_summary = run_batch(
            scenario=scenario,
            output_dir=scenario_out_dir,
            drqn_checkpoint=args.drqn_checkpoint,
        )

        for policy_name, stats in policy_summary.items():
            rows.append(
                {
                    "scenario": base_scenario.get("name", "scenario"),
                    "disaster_type": scenario.get("disaster_type"),
                    "severity": severity,
                    "policy": policy_name,
                    "runs": stats.get("runs"),
                    "avg_reached_rate": stats.get("avg_reached_rate"),
                    "avg_alive_rate": stats.get("avg_alive_rate"),
                    "avg_exposure_total": stats.get("avg_exposure_total"),
                    "avg_t90_step": stats.get("avg_t90_step"),
                    "avg_t95_step": stats.get("avg_t95_step"),
                    "avg_reach_rate_gap": stats.get("avg_reach_rate_gap"),
                    "avg_shelter_reassignments": stats.get("avg_shelter_reassignments"),
                    "avg_faculty_reached_rate": stats.get("avg_faculty_reached_rate"),
                    "avg_staff_reached_rate": stats.get("avg_staff_reached_rate"),
                }
            )

    csv_path = os.path.join(args.output_dir, "disaster_severity_sweep.csv")
    json_path = os.path.join(args.output_dir, "disaster_severity_sweep.json")
    md_path = os.path.join(args.output_dir, "disaster_severity_sweep.md")
    txt_path = os.path.join(args.output_dir, "disaster_severity_sweep_management_summary.txt")

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(
            {
                "base_scenario": args.scenario,
                "drqn_checkpoint": args.drqn_checkpoint,
                "severity_levels": SEVERITY_LEVELS,
                "policies": args.policies,
                "rows": rows,
            },
            f,
            indent=2,
        )

    with open(md_path, "w", encoding="utf-8") as f:
        f.write("# Disaster Severity Sweep\n\n")
        f.write(f"- Base scenario: `{args.scenario}`\n")
        f.write(f"- DRQN checkpoint: `{args.drqn_checkpoint}`\n")
        f.write(f"- Policies: `{args.policies}`\n\n")
        for row in rows:
            f.write(
                f"- `{row['severity']}` / `{row['policy']}`: "
                f"reached={row['avg_reached_rate']}, alive={row['avg_alive_rate']}, "
                f"exposure={row['avg_exposure_total']}, t90={row['avg_t90_step']}, "
                f"t95={row['avg_t95_step']}, gap={row['avg_reach_rate_gap']}, "
                f"reassignments={row['avg_shelter_reassignments']}\n"
            )

    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("Disaster Severity Sweep Management Summary\n")
        f.write(f"Base scenario: {args.scenario}\n")
        f.write(f"Policies: {args.policies}\n\n")
        f.write("Ordered by severity:\n")
        for row in rows:
            f.write(
                f"- {row['severity']} / {row['policy']}: reached={row['avg_reached_rate']}, "
                f"exposure={row['avg_exposure_total']}, t95={row['avg_t95_step']}, "
                f"reassignments={row['avg_shelter_reassignments']}\n"
            )

    print(f"[severity] csv: {csv_path}")
    print(f"[severity] json: {json_path}")
    print(f"[severity] md: {md_path}")
    print(f"[severity] management: {txt_path}")


if __name__ == "__main__":
    main()
