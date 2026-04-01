import argparse
import csv
import json
import os

import numpy as np
import torch

from eval_best_model import evaluate_checkpoint


def main():
    parser = argparse.ArgumentParser(description="Evaluate one checkpoint across multiple seeds.")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--device", default="cpu", choices=["cpu", "cuda", "mps"])
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--seeds", type=int, nargs="+", default=[2026, 2027, 2028, 2029, 2030])
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    device = torch.device(args.device)

    per_seed_rows = []
    reached_rates = []
    return_means = []
    steps_means = []

    for seed in args.seeds:
        _, summary = evaluate_checkpoint(
            ckpt_path=args.checkpoint,
            episodes=args.episodes,
            max_steps=args.max_steps,
            seed=seed,
            device=device,
        )
        per_seed_rows.append(summary)
        reached_rates.append(summary["reached_rate"])
        return_means.append(summary["return_mean"])
        steps_means.append(summary["steps_mean"])

    aggregate = {
        "checkpoint": args.checkpoint,
        "episodes_per_seed": args.episodes,
        "seeds": args.seeds,
        "num_seeds": len(args.seeds),
        "reached_rate_mean": float(np.mean(reached_rates)),
        "reached_rate_std": float(np.std(reached_rates)),
        "return_mean_mean": float(np.mean(return_means)),
        "return_mean_std": float(np.std(return_means)),
        "steps_mean_mean": float(np.mean(steps_means)),
        "steps_mean_std": float(np.std(steps_means)),
    }

    per_seed_csv = os.path.join(args.output_dir, "multiseed_eval_per_seed.csv")
    with open(per_seed_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "seed",
                "episodes",
                "checkpoint",
                "return_mean",
                "return_std",
                "reached_rate",
                "steps_mean",
                "steps_std",
            ],
        )
        writer.writeheader()
        writer.writerows(per_seed_rows)

    agg_json = os.path.join(args.output_dir, "multiseed_eval_summary.json")
    with open(agg_json, "w", encoding="utf-8") as f:
        json.dump(aggregate, f, indent=2)

    agg_txt = os.path.join(args.output_dir, "multiseed_eval_summary.txt")
    with open(agg_txt, "w", encoding="utf-8") as f:
        f.write("Multiseed Evaluation Summary\n")
        for k, v in aggregate.items():
            f.write(f"{k}: {v}\n")

    print(f"[eval-multiseed] per-seed csv: {per_seed_csv}")
    print(f"[eval-multiseed] summary json: {agg_json}")
    print(f"[eval-multiseed] summary txt: {agg_txt}")
    print(f"[eval-multiseed] aggregate: {aggregate}")


if __name__ == "__main__":
    main()
