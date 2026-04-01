import argparse
import csv
import json
import os

from batch_runner import run_batch
from scenario_loader import load_scenario


def main():
    parser = argparse.ArgumentParser(description="Run shelter-capacity scaling sweep with the current DRQN checkpoint.")
    parser.add_argument(
        "--scenarios",
        nargs="+",
        default=[
            "scenarios/enterprise_capacity_a200_c80_cap50.json",
            "scenarios/enterprise_capacity_a300_c120_cap70.json",
            "scenarios/enterprise_capacity_a500_c200_cap100.json",
        ],
    )
    parser.add_argument("--interaction-aware", action="store_true")
    parser.add_argument("--output-dir", default="logs/capacity_scaling")
    parser.add_argument("--drqn-checkpoint", default="logs/drqn_blocked_finetune/drqn_torch_best.pt")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    rows = []

    scenario_paths = list(args.scenarios)
    if args.interaction_aware:
        scenario_paths = [
            "scenarios/enterprise_capacity_interaction_a200_c80_cap50.json",
            "scenarios/enterprise_capacity_interaction_a300_c120_cap70.json",
            "scenarios/enterprise_capacity_interaction_a500_c200_cap100.json",
        ]

    for scenario_path in scenario_paths:
        scenario = load_scenario(scenario_path)
        scenario_name = scenario.get("name", os.path.splitext(os.path.basename(scenario_path))[0])
        scenario_out_dir = os.path.join(args.output_dir, scenario_name)
        _, _, _, policy_summary = run_batch(
            scenario=scenario,
            output_dir=scenario_out_dir,
            drqn_checkpoint=args.drqn_checkpoint,
        )
        stats = policy_summary.get("drqn", {})
        cfg = scenario.get("config_overrides", {})
        rows.append(
            {
                "scenario": scenario_name,
                "ped_count": cfg.get("EVAC_PED_COUNT"),
                "car_count": cfg.get("EVAC_CAR_COUNT"),
                "shelter_capacity_enabled": cfg.get("EVAC_SHELTER_CAPACITY_ENABLED"),
                "shelter_capacity_per_site": cfg.get("EVAC_SHELTER_CAPACITY_PER_SITE"),
                "interaction_density_enabled": cfg.get("EVAC_INTERACTION_DENSITY_ENABLED", False),
                "step_limit": cfg.get("EVAC_STEP_LIMIT"),
                "runs": stats.get("runs"),
                "avg_reached_rate": stats.get("avg_reached_rate"),
                "avg_alive_rate": stats.get("avg_alive_rate"),
                "avg_exposure_total": stats.get("avg_exposure_total"),
                "avg_t90_step": stats.get("avg_t90_step"),
                "avg_t95_step": stats.get("avg_t95_step"),
                "avg_reach_rate_gap": stats.get("avg_reach_rate_gap"),
                "avg_shelter_reassignments": stats.get("avg_shelter_reassignments"),
            }
        )

    csv_path = os.path.join(args.output_dir, "capacity_scaling_summary.csv")
    json_path = os.path.join(args.output_dir, "capacity_scaling_summary.json")
    md_path = os.path.join(args.output_dir, "capacity_scaling_summary.md")

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(
            {
                "drqn_checkpoint": args.drqn_checkpoint,
                "scenarios": scenario_paths,
                "interaction_aware": bool(args.interaction_aware),
                "rows": rows,
            },
            f,
            indent=2,
        )

    with open(md_path, "w", encoding="utf-8") as f:
        f.write("# Capacity Scaling Summary\n\n")
        f.write(f"- DRQN checkpoint: `{args.drqn_checkpoint}`\n\n")
        for row in rows:
            f.write(
                f"- `{row['scenario']}`: ped={row['ped_count']}, car={row['car_count']}, "
                f"cap_per_shelter={row['shelter_capacity_per_site']}, interaction={row['interaction_density_enabled']}, reached={row['avg_reached_rate']}, "
                f"alive={row['avg_alive_rate']}, exposure={row['avg_exposure_total']}, "
                f"t95={row['avg_t95_step']}, reassignments={row['avg_shelter_reassignments']}, "
                f"gap={row['avg_reach_rate_gap']}\n"
            )

    print(f"[capacity] csv: {csv_path}")
    print(f"[capacity] json: {json_path}")
    print(f"[capacity] md: {md_path}")


if __name__ == "__main__":
    main()
