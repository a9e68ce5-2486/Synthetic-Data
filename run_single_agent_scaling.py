import argparse
import csv
import json
import os

from batch_runner import run_batch
from scenario_loader import load_scenario


def main():
    parser = argparse.ArgumentParser(description="Run a single-agent scaling sweep with the current DRQN checkpoint.")
    parser.add_argument(
        "--scenarios",
        nargs="+",
        default=[
            "scenarios/enterprise_scaling_a60_c20.json",
            "scenarios/enterprise_scaling_a80_c30.json",
            "scenarios/enterprise_scaling_a100_c40.json",
            "scenarios/enterprise_scaling_a120_c50.json",
            "scenarios/enterprise_scaling_a150_c60.json",
            "scenarios/enterprise_scaling_a200_c80.json",
            "scenarios/enterprise_scaling_a300_c120.json",
            "scenarios/enterprise_scaling_a400_c160.json",
            "scenarios/enterprise_scaling_a500_c200.json",
        ],
    )
    parser.add_argument("--output-dir", default="logs/single_agent_scaling")
    parser.add_argument("--drqn-checkpoint", default="logs/drqn_blocked_finetune/drqn_torch_best.pt")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    rows = []
    scenario_summaries = {}

    for scenario_path in args.scenarios:
        scenario = load_scenario(scenario_path)
        scenario_name = scenario.get("name", os.path.splitext(os.path.basename(scenario_path))[0])
        scenario_out_dir = os.path.join(args.output_dir, scenario_name)
        _, _, _, policy_summary = run_batch(
            scenario=scenario,
            output_dir=scenario_out_dir,
            drqn_checkpoint=args.drqn_checkpoint,
        )
        drqn_stats = policy_summary.get("drqn", {})
        config_overrides = scenario.get("config_overrides", {})
        row = {
            "scenario": scenario_name,
            "ped_count": config_overrides.get("EVAC_PED_COUNT"),
            "car_count": config_overrides.get("EVAC_CAR_COUNT"),
            "bus_count": config_overrides.get("EVAC_BUS_COUNT"),
            "step_limit": config_overrides.get("EVAC_STEP_LIMIT"),
            "runs": drqn_stats.get("runs"),
            "avg_reached_rate": drqn_stats.get("avg_reached_rate"),
            "avg_alive_rate": drqn_stats.get("avg_alive_rate"),
            "avg_exposure_total": drqn_stats.get("avg_exposure_total"),
            "avg_t90_step": drqn_stats.get("avg_t90_step"),
            "avg_t95_step": drqn_stats.get("avg_t95_step"),
            "avg_faculty_reached_rate": drqn_stats.get("avg_faculty_reached_rate"),
            "avg_staff_reached_rate": drqn_stats.get("avg_staff_reached_rate"),
            "avg_reach_rate_gap": drqn_stats.get("avg_reach_rate_gap"),
        }
        rows.append(row)
        scenario_summaries[scenario_name] = row

    csv_path = os.path.join(args.output_dir, "single_agent_scaling_summary.csv")
    json_path = os.path.join(args.output_dir, "single_agent_scaling_summary.json")
    md_path = os.path.join(args.output_dir, "single_agent_scaling_summary.md")

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(
            {
                "drqn_checkpoint": args.drqn_checkpoint,
                "scenarios": args.scenarios,
                "rows": rows,
            },
            f,
            indent=2,
        )

    with open(md_path, "w", encoding="utf-8") as f:
        f.write("# Single-Agent Scaling Summary\n\n")
        f.write(f"- DRQN checkpoint: `{args.drqn_checkpoint}`\n\n")
        for row in rows:
            f.write(
                f"- `{row['scenario']}`: ped={row['ped_count']}, car={row['car_count']}, "
                f"reached={row['avg_reached_rate']}, alive={row['avg_alive_rate']}, "
                f"exposure={row['avg_exposure_total']}, t90={row['avg_t90_step']}, "
                f"t95={row['avg_t95_step']}, gap={row['avg_reach_rate_gap']}\n"
            )

    print(f"[scaling] csv: {csv_path}")
    print(f"[scaling] json: {json_path}")
    print(f"[scaling] md: {md_path}")


if __name__ == "__main__":
    main()
