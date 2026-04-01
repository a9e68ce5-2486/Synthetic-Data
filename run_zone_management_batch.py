import argparse
import csv
import json
import os
import subprocess
import sys

from scenario_loader import load_scenario


def _scenario_name(path):
    scenario = load_scenario(path)
    return scenario.get("name", os.path.splitext(os.path.basename(path))[0]), scenario


def _run_zone_route(python_bin, scenario_path, checkpoint, seed, num_zones, output_dir):
    cmd = [
        python_bin,
        "zone_route_recommendation.py",
        "--scenario",
        scenario_path,
        "--checkpoint",
        checkpoint,
        "--seed",
        str(seed),
        "--num-zones",
        str(num_zones),
        "--output-dir",
        output_dir,
    ]
    subprocess.run(cmd, check=True)


def main():
    parser = argparse.ArgumentParser(
        description="Batch-run zone route recommendation and aggregate management-facing risk summaries."
    )
    parser.add_argument(
        "--scenarios",
        nargs="+",
        default=[
            "scenarios/enterprise_capacity_a200_c80_cap50.json",
            "scenarios/enterprise_capacity_a300_c120_cap70.json",
            "scenarios/enterprise_capacity_a500_c200_cap100.json",
        ],
    )
    parser.add_argument("--python-bin", default=sys.executable)
    parser.add_argument("--checkpoint", default="logs/drqn_blocked_finetune/drqn_torch_best.pt")
    parser.add_argument("--seed", type=int, default=20260323)
    parser.add_argument("--num-zones", type=int, default=6)
    parser.add_argument("--output-dir", default="logs/zone_management_batch")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    rows = []

    for scenario_path in args.scenarios:
        scenario_name, scenario = _scenario_name(scenario_path)
        scenario_out_dir = os.path.join(args.output_dir, scenario_name)
        os.makedirs(scenario_out_dir, exist_ok=True)

        _run_zone_route(
            python_bin=args.python_bin,
            scenario_path=scenario_path,
            checkpoint=args.checkpoint,
            seed=args.seed,
            num_zones=args.num_zones,
            output_dir=scenario_out_dir,
        )

        json_path = os.path.join(scenario_out_dir, f"{scenario_name}_zone_routes.json")
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        zones = data.get("zones", [])
        weak_backup_zones = [int(z["zone_id"]) for z in zones if bool(z.get("backup_weak"))]
        changed_primary_zones = [
            int(z["zone_id"])
            for z in zones
            if z.get("assigned_primary_shelter") != z.get("recommended_primary_shelter")
        ]
        changed_backup_zones = [
            int(z["zone_id"])
            for z in zones
            if z.get("assigned_backup_shelter") != z.get("recommended_backup_shelter")
        ]
        cfg = scenario.get("config_overrides", {})
        rows.append(
            {
                "scenario": scenario_name,
                "ped_count": cfg.get("EVAC_PED_COUNT"),
                "car_count": cfg.get("EVAC_CAR_COUNT"),
                "shelter_capacity_per_site": cfg.get("EVAC_SHELTER_CAPACITY_PER_SITE"),
                "num_zones": len(zones),
                "weak_backup_zone_count": len(weak_backup_zones),
                "weak_backup_zones": weak_backup_zones,
                "changed_primary_zone_count": len(changed_primary_zones),
                "changed_primary_zones": changed_primary_zones,
                "changed_backup_zone_count": len(changed_backup_zones),
                "changed_backup_zones": changed_backup_zones,
                "all_zones_assigned": all(int(z.get("unassigned_demand", 0) or 0) == 0 for z in zones),
            }
        )

    csv_path = os.path.join(args.output_dir, "zone_management_batch_summary.csv")
    json_path = os.path.join(args.output_dir, "zone_management_batch_summary.json")
    md_path = os.path.join(args.output_dir, "zone_management_batch_summary.md")
    txt_path = os.path.join(args.output_dir, "zone_management_batch_summary.txt")

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(
            {
                "checkpoint": args.checkpoint,
                "seed": args.seed,
                "num_zones": args.num_zones,
                "scenarios": args.scenarios,
                "rows": rows,
            },
            f,
            indent=2,
        )

    with open(md_path, "w", encoding="utf-8") as f:
        f.write("# Zone Management Batch Summary\n\n")
        f.write(f"- Checkpoint: `{args.checkpoint}`\n")
        f.write(f"- Seed: `{args.seed}`\n")
        f.write(f"- Num zones: `{args.num_zones}`\n\n")
        for row in rows:
            f.write(
                f"- `{row['scenario']}`: ped={row['ped_count']}, car={row['car_count']}, "
                f"cap={row['shelter_capacity_per_site']}, weak_backup_count={row['weak_backup_zone_count']}, "
                f"weak_backup_zones={row['weak_backup_zones']}, "
                f"changed_primary={row['changed_primary_zones']}, "
                f"changed_backup={row['changed_backup_zones']}, "
                f"all_assigned={row['all_zones_assigned']}\n"
            )

    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("Zone Management Batch Summary\n")
        f.write(f"Checkpoint: {args.checkpoint}\n")
        f.write(f"Seed: {args.seed}\n\n")
        f.write("Priority scenarios:\n")
        for row in sorted(rows, key=lambda r: (r["weak_backup_zone_count"], r["changed_primary_zone_count"]), reverse=True):
            f.write(
                f"- {row['scenario']}: weak_backup={row['weak_backup_zones']}, "
                f"changed_primary={row['changed_primary_zones']}, "
                f"changed_backup={row['changed_backup_zones']}\n"
            )

    print(f"[zone-batch] csv: {csv_path}")
    print(f"[zone-batch] json: {json_path}")
    print(f"[zone-batch] md: {md_path}")
    print(f"[zone-batch] management: {txt_path}")


if __name__ == "__main__":
    main()
