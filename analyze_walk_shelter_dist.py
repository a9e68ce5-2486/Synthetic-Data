import argparse
import json
import math
import os

import networkx as nx
import numpy as np

from evac_env import EvacEnv
from scenario_loader import load_scenario, temporary_config


def compute_distribution(scenario_path):
    scenario = load_scenario(scenario_path)
    overrides = dict(scenario.get("config_overrides", {}))
    with temporary_config(overrides):
        env = EvacEnv()

    shelters = list(env.shelters)
    distances = []
    for node in env.G_walk.nodes():
        if node in shelters:
            distances.append(0.0)
            continue
        best = math.inf
        for shelter in shelters:
            try:
                d = float(nx.shortest_path_length(env.G_walk, node, shelter, weight="weight"))
            except Exception:
                continue
            if d < best:
                best = d
        if math.isfinite(best):
            distances.append(best)

    if not distances:
        raise RuntimeError("No reachable walk-node to shelter distances were found.")

    arr = np.array(distances, dtype=np.float64)
    return {
        "scenario": scenario.get("name", os.path.basename(scenario_path)),
        "count": int(arr.size),
        "min": float(np.min(arr)),
        "p50": float(np.percentile(arr, 50)),
        "p90": float(np.percentile(arr, 90)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(np.max(arr)),
        "mean": float(np.mean(arr)),
    }


def main():
    parser = argparse.ArgumentParser(description="Analyze walk-node shortest-path distance to nearest shelter.")
    parser.add_argument("--scenario", default="scenarios/enterprise_blizzard_quick.json")
    parser.add_argument("--output", default=None)
    args = parser.parse_args()

    summary = compute_distribution(args.scenario)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
