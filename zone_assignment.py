import argparse
import json
import os
import random

import networkx as nx
import numpy as np

import config
from agents.ped_agent import PedAgent
from evac_env import EvacEnv
from scenario_loader import load_scenario, temporary_config


def _serialize_node(node):
    if isinstance(node, (np.integer, int)):
        return int(node)
    return str(node)


def _build_ped_population(env, seed):
    random.seed(seed)
    np.random.seed(seed)
    nodes_walk = list(env.G_walk.nodes())
    peds = []
    for i in range(config.EVAC_PED_COUNT):
        start = random.choice(nodes_walk)
        ped = PedAgent(i + 1, start, env)
        ped.role = "faculty" if random.random() < config.EVAC_FACULTY_RATIO else "staff"
        peds.append(ped)
    return peds


def _kmeans(points, k, seed, max_iter=25):
    rng = np.random.default_rng(seed)
    if len(points) <= k:
        labels = np.arange(len(points), dtype=int)
        centers = points.copy()
        return labels, centers
    init_idx = rng.choice(len(points), size=k, replace=False)
    centers = points[init_idx].copy()
    labels = np.zeros(len(points), dtype=int)
    for _ in range(max_iter):
        dists = ((points[:, None, :] - centers[None, :, :]) ** 2).sum(axis=2)
        new_labels = np.argmin(dists, axis=1)
        if np.array_equal(new_labels, labels):
            break
        labels = new_labels
        for j in range(k):
            mask = labels == j
            if np.any(mask):
                centers[j] = points[mask].mean(axis=0)
            else:
                centers[j] = points[rng.integers(0, len(points))]
    return labels, centers


def _avg_distance_to_shelter(graph, member_nodes, shelter_node):
    vals = []
    for n in member_nodes:
        try:
            vals.append(float(nx.shortest_path_length(graph, n, shelter_node, weight="weight")))
        except Exception:
            vals.append(float("inf"))
    if not vals:
        return float("inf")
    return float(sum(vals) / len(vals))


def _pick_representative(member_nodes, center, pos):
    best = None
    best_d = float("inf")
    cx, cy = center
    for n in member_nodes:
        x, y = pos[n]
        d = (x - cx) ** 2 + (y - cy) ** 2
        if d < best_d:
            best_d = d
            best = n
    return best


def _capacity_adjusted_score(distance_score, demand, remaining, capacity_enabled):
    if not np.isfinite(distance_score):
        return float("inf")
    if not capacity_enabled:
        return float(distance_score)

    rem = float(remaining)
    dem = float(max(1, demand))
    if rem <= 0.0:
        # Hard discourage already-full shelters.
        return float(distance_score + 1e6)

    load_pressure = dem / max(1.0, rem)
    overload = max(0.0, dem - rem) / dem
    # Distance stays primary, but remaining capacity matters before overload happens.
    return float(distance_score * (1.0 + 0.35 * load_pressure) + 500.0 * overload)


def _pick_backup_shelter(ranked, primary, demand, remaining, capacity_enabled):
    backup = None
    best_score = float("inf")
    for shelter_node, base_score in ranked:
        if shelter_node == primary:
            continue
        adjusted = _capacity_adjusted_score(
            distance_score=base_score,
            demand=demand,
            remaining=remaining.get(shelter_node, 0.0),
            capacity_enabled=capacity_enabled,
        )
        if adjusted < best_score:
            best_score = adjusted
            backup = shelter_node
    return backup


def _assign_zone_shelters(zones, shelters, env):
    capacity_enabled = bool(getattr(config, "EVAC_SHELTER_CAPACITY_ENABLED", False))
    remaining = {s: env.shelter_capacity.get(s, 999999) - env.shelter_occupancy.get(s, 0) for s in shelters}
    for zone in sorted(zones, key=lambda z: z["demand"], reverse=True):
        ranked = sorted(zone["shelter_scores"], key=lambda kv: kv[1])
        primary = None
        best_score = float("inf")
        for shelter_node, base_score in ranked:
            adjusted = _capacity_adjusted_score(
                distance_score=base_score,
                demand=zone["demand"],
                remaining=remaining.get(shelter_node, 0.0),
                capacity_enabled=capacity_enabled,
            )
            if adjusted < best_score:
                best_score = adjusted
                primary = shelter_node
        primary_available = max(0, int(remaining.get(primary, 0))) if primary is not None else 0
        if capacity_enabled:
            primary_assigned_demand = min(int(zone["demand"]), primary_available)
        else:
            primary_assigned_demand = int(zone["demand"])
        overflow_demand = int(zone["demand"]) - int(primary_assigned_demand)
        if primary is not None:
            remaining[primary] = remaining.get(primary, 0) - primary_assigned_demand

        backup = _pick_backup_shelter(
            ranked=ranked,
            primary=primary,
            demand=max(1, overflow_demand) if overflow_demand > 0 else zone["demand"],
            remaining=remaining,
            capacity_enabled=capacity_enabled,
        )
        backup_available = max(0, int(remaining.get(backup, 0))) if backup is not None else 0
        if capacity_enabled and overflow_demand > 0 and backup is not None:
            backup_assigned_demand = min(int(overflow_demand), backup_available)
            remaining[backup] = remaining.get(backup, 0) - backup_assigned_demand
        else:
            backup_assigned_demand = 0
        unassigned_demand = int(zone["demand"]) - int(primary_assigned_demand) - int(backup_assigned_demand)
        zone["primary_shelter"] = primary
        zone["backup_shelter"] = backup
        zone["primary_remaining_after_assignment"] = remaining.get(primary) if primary is not None else None
        zone["primary_assigned_demand"] = int(primary_assigned_demand)
        zone["backup_assigned_demand"] = int(backup_assigned_demand)
        zone["overflow_demand"] = int(overflow_demand)
        zone["unassigned_demand"] = int(unassigned_demand)
        zone["primary_assignment_score"] = best_score if primary is not None else None


def _build_zones(env, peds, num_zones, seed):
    ped_nodes = [p.node for p in peds]
    coords = np.array([env.pos[n] for n in ped_nodes], dtype=np.float64)
    k = max(1, min(int(num_zones), len(coords)))
    labels, centers = _kmeans(coords, k, seed=seed)
    zones = []
    shelters = list(env.shelters)
    for zone_id in range(k):
        members = [p for idx, p in enumerate(peds) if labels[idx] == zone_id]
        member_nodes = [p.node for p in members]
        faculty_count = sum(1 for p in members if p.role == "faculty")
        staff_count = sum(1 for p in members if p.role == "staff")
        center = centers[zone_id]
        rep_start = _pick_representative(member_nodes, center, env.pos)
        shelter_scores = [(s, _avg_distance_to_shelter(env.G_walk, member_nodes, s)) for s in shelters]
        zones.append(
            {
                "zone_id": zone_id,
                "demand": len(members),
                "faculty_count": faculty_count,
                "staff_count": staff_count,
                "center_x": float(center[0]),
                "center_y": float(center[1]),
                "representative_start_node": rep_start,
                "member_start_nodes": member_nodes,
                "shelter_scores": shelter_scores,
            }
        )
    _assign_zone_shelters(zones, shelters, env)
    return zones


def main():
    parser = argparse.ArgumentParser(description="Build zone-level shelter assignments for one scenario.")
    parser.add_argument("--scenario", default="scenarios/enterprise_baseline.json")
    parser.add_argument("--seed", type=int, default=20260323)
    parser.add_argument("--num-zones", type=int, default=6)
    parser.add_argument("--output-dir", default="logs/zone_assignment")
    args = parser.parse_args()

    scenario = load_scenario(args.scenario)
    os.makedirs(args.output_dir, exist_ok=True)

    with temporary_config(scenario.get("config_overrides", {})):
        env = EvacEnv()
        peds = _build_ped_population(env, seed=args.seed)
        zones = _build_zones(env, peds, num_zones=args.num_zones, seed=args.seed)
        shelter_capacity_enabled = bool(getattr(config, "EVAC_SHELTER_CAPACITY_ENABLED", False))
        shelter_capacity_per_site = int(getattr(config, "EVAC_SHELTER_CAPACITY_PER_SITE", 999999))

    result = {
        "scenario": scenario.get("name", "scenario"),
        "seed": int(args.seed),
        "num_zones": int(args.num_zones),
        "ped_count": int(len(peds)),
        "assignment_mode": "capacity_aware_demand_balancing",
        "shelter_capacity_enabled": shelter_capacity_enabled,
        "shelter_capacity_per_site": shelter_capacity_per_site,
        "zones": [
            {
                "zone_id": int(z["zone_id"]),
                "demand": int(z["demand"]),
                "faculty_count": int(z["faculty_count"]),
                "staff_count": int(z["staff_count"]),
                "center_x": z["center_x"],
                "center_y": z["center_y"],
                "representative_start_node": _serialize_node(z["representative_start_node"]),
                "primary_shelter": _serialize_node(z["primary_shelter"]) if z["primary_shelter"] is not None else None,
                "backup_shelter": _serialize_node(z["backup_shelter"]) if z["backup_shelter"] is not None else None,
                "primary_remaining_after_assignment": z["primary_remaining_after_assignment"],
                "primary_assigned_demand": z["primary_assigned_demand"],
                "backup_assigned_demand": z["backup_assigned_demand"],
                "overflow_demand": z["overflow_demand"],
                "unassigned_demand": z["unassigned_demand"],
                "primary_assignment_score": z["primary_assignment_score"],
                "member_start_nodes": [_serialize_node(n) for n in z["member_start_nodes"]],
                "shelter_scores": [[_serialize_node(s), float(score)] for s, score in z["shelter_scores"]],
            }
            for z in zones
        ],
    }

    stem = f"{result['scenario']}_zones"
    json_path = os.path.join(args.output_dir, f"{stem}.json")
    md_path = os.path.join(args.output_dir, f"{stem}.md")

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)

    with open(md_path, "w", encoding="utf-8") as f:
        f.write("# Zone Assignment\n\n")
        f.write(f"- Scenario: `{result['scenario']}`\n")
        f.write(f"- Seed: `{result['seed']}`\n")
        f.write(f"- Num zones: `{result['num_zones']}`\n")
        f.write(f"- Ped count: `{result['ped_count']}`\n")
        f.write(f"- Assignment mode: `{result['assignment_mode']}`\n")
        f.write(f"- Shelter capacity enabled: `{result['shelter_capacity_enabled']}`\n")
        f.write(f"- Shelter capacity per site: `{result['shelter_capacity_per_site']}`\n\n")
        for z in result["zones"]:
            f.write(
                f"- `zone_{z['zone_id']}`: demand={z['demand']}, faculty={z['faculty_count']}, "
                f"staff={z['staff_count']}, rep_start={z['representative_start_node']}, "
                f"primary={z['primary_shelter']}, backup={z['backup_shelter']}, "
                f"remaining_after_primary={z['primary_remaining_after_assignment']}, "
                f"primary_assigned={z['primary_assigned_demand']}, "
                f"backup_assigned={z['backup_assigned_demand']}, "
                f"unassigned={z['unassigned_demand']}, "
                f"assignment_score={z['primary_assignment_score']}\n"
            )

    print(f"[zone] json: {json_path}")
    print(f"[zone] md: {md_path}")


if __name__ == "__main__":
    main()
