import argparse
import json
import os

from evac_env import EvacEnv
from route_recommendation import extract_route
from scenario_loader import load_scenario, temporary_config
from zone_assignment import _build_ped_population, _build_zones, _serialize_node


def _coerce_node(node):
    if node is None:
        return None
    try:
        return int(node)
    except (TypeError, ValueError):
        return node


def _route_payload(route):
    if route is None:
        return None
    return {
        "recommended_shelter": route["recommended_shelter"],
        "reached": route["reached"],
        "steps": route["steps"],
        "exposure": route["exposure"],
        "path_length_nodes": route["path_length_nodes"],
        "path_nodes": route["path_nodes"],
        "traversed_edges": route["traversed_edges"],
        "replan_count": route["replan_count"],
        "target_history": route["target_history"],
    }


def _ordered_candidate_shelters(zone):
    ordered = []
    seen = set()
    for shelter in [zone.get("primary_shelter"), zone.get("backup_shelter")]:
        if shelter is not None and shelter not in seen:
            ordered.append(shelter)
            seen.add(shelter)
    for shelter, _score in sorted(zone.get("shelter_scores", []), key=lambda kv: kv[1]):
        if shelter not in seen:
            ordered.append(shelter)
            seen.add(shelter)
    return ordered


def _extract_candidate_routes(checkpoint, rep_start, role, seed, device, max_neighbors, candidate_shelters):
    routes = []
    for shelter in candidate_shelters:
        route_env = EvacEnv()
        route = extract_route(
            env=route_env,
            checkpoint_path=checkpoint,
            start_node=rep_start,
            role=role,
            seed=seed,
            device=device,
            max_neighbors=max_neighbors,
            goal_override=shelter,
        )
        routes.append((shelter, route))
    return routes


def _route_quality_ok(primary_route, backup_route, max_step_ratio, max_exposure_ratio):
    if primary_route is None or backup_route is None:
        return False
    if not backup_route.get("reached"):
        return False
    primary_steps = max(1, int(primary_route.get("steps", 0) or 0))
    backup_steps = int(backup_route.get("steps", 0) or 0)
    primary_exposure = max(1e-6, float(primary_route.get("exposure", 0.0) or 0.0))
    backup_exposure = float(backup_route.get("exposure", 0.0) or 0.0)
    return (
        backup_steps <= primary_steps * float(max_step_ratio)
        and backup_exposure <= primary_exposure * float(max_exposure_ratio)
    )


def _route_quality_score(route, step_weight, exposure_weight):
    if route is None:
        return float("inf")
    if not route.get("reached"):
        return float("inf")
    return float(
        float(route.get("steps", 0) or 0) * float(step_weight)
        + float(route.get("exposure", 0.0) or 0.0) * float(exposure_weight)
    )


def _backup_status_label(quality_ok, backup_route):
    if backup_route is None:
        return "missing"
    if not backup_route.get("reached"):
        return "unreachable"
    if quality_ok:
        return "usable"
    return "weak"


def _pick_feasible_routes(candidate_routes, max_step_ratio, max_exposure_ratio):
    feasible = [(shelter, route) for shelter, route in candidate_routes if route is not None and route.get("reached")]
    primary = feasible[0] if feasible else (candidate_routes[0] if candidate_routes else (None, None))
    backup = None
    for shelter, route in feasible[1:]:
        if shelter != primary[0] and _route_quality_ok(primary[1], route, max_step_ratio, max_exposure_ratio):
            backup = (shelter, route)
            break
    if backup is None:
        for shelter, route in candidate_routes:
            if shelter != primary[0]:
                backup = (shelter, route)
                break
    return primary, backup, feasible


def _pick_quality_ranked_routes(feasible_routes, fallback_primary, fallback_backup, max_step_ratio, max_exposure_ratio, step_weight, exposure_weight):
    ranked = sorted(
        feasible_routes,
        key=lambda item: _route_quality_score(item[1], step_weight=step_weight, exposure_weight=exposure_weight),
    )
    if ranked:
        primary = ranked[0]
    else:
        primary = fallback_primary

    backup = None
    for shelter, route in ranked[1:]:
        if shelter != primary[0] and _route_quality_ok(primary[1], route, max_step_ratio, max_exposure_ratio):
            backup = (shelter, route)
            break
    if backup is None:
        for shelter, route in ranked[1:]:
            if shelter != primary[0]:
                backup = (shelter, route)
                break
    if backup is None:
        backup = fallback_backup
    return primary, backup


def main():
    parser = argparse.ArgumentParser(
        description="Generate zone-level shelter assignment with primary and backup DRQN routes."
    )
    parser.add_argument("--scenario", default="scenarios/enterprise_baseline.json")
    parser.add_argument("--checkpoint", default="logs/drqn_blocked_finetune/drqn_torch_best.pt")
    parser.add_argument("--seed", type=int, default=20260323)
    parser.add_argument("--num-zones", type=int, default=6)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--max-neighbors", type=int, default=None)
    parser.add_argument("--output-dir", default="logs/zone_route_recommendation")
    args = parser.parse_args()

    scenario = load_scenario(args.scenario)
    os.makedirs(args.output_dir, exist_ok=True)

    with temporary_config(scenario.get("config_overrides", {})):
        env = EvacEnv()
        peds = _build_ped_population(env, seed=args.seed)
        zones = _build_zones(env, peds, num_zones=args.num_zones, seed=args.seed)
        import config as runtime_config
        shelter_capacity_enabled = bool(getattr(runtime_config, "EVAC_SHELTER_CAPACITY_ENABLED", False))
        shelter_capacity_per_site = int(getattr(runtime_config, "EVAC_SHELTER_CAPACITY_PER_SITE", 999999))
        backup_max_step_ratio = float(getattr(runtime_config, "EVAC_ZONE_BACKUP_MAX_STEP_RATIO", 1.5))
        backup_max_exposure_ratio = float(getattr(runtime_config, "EVAC_ZONE_BACKUP_MAX_EXPOSURE_RATIO", 2.5))
        route_step_weight = float(getattr(runtime_config, "EVAC_ZONE_ROUTE_STEP_WEIGHT", 1.0))
        route_exposure_weight = float(getattr(runtime_config, "EVAC_ZONE_ROUTE_EXPOSURE_WEIGHT", 12.0))

        zone_rows = []
        for z in zones:
            rep_start = z["representative_start_node"]
            assigned_primary_shelter = _coerce_node(z["primary_shelter"])
            assigned_backup_shelter = _coerce_node(z["backup_shelter"])

            role = "faculty" if z["faculty_count"] >= z["staff_count"] else "staff"
            candidate_shelters = [_coerce_node(s) for s in _ordered_candidate_shelters(z)]
            candidate_routes = _extract_candidate_routes(
                checkpoint=args.checkpoint,
                rep_start=rep_start,
                role=role,
                seed=args.seed,
                device=args.device,
                max_neighbors=args.max_neighbors,
                candidate_shelters=candidate_shelters,
            )
            (primary_shelter, primary_route), (backup_shelter, backup_route), feasible_routes = _pick_feasible_routes(
                candidate_routes,
                max_step_ratio=backup_max_step_ratio,
                max_exposure_ratio=backup_max_exposure_ratio,
            )
            (
                (recommended_primary_shelter, recommended_primary_route),
                (recommended_backup_shelter, recommended_backup_route),
            ) = _pick_quality_ranked_routes(
                feasible_routes=feasible_routes,
                fallback_primary=(primary_shelter, primary_route),
                fallback_backup=(backup_shelter, backup_route),
                max_step_ratio=backup_max_step_ratio,
                max_exposure_ratio=backup_max_exposure_ratio,
                step_weight=route_step_weight,
                exposure_weight=route_exposure_weight,
            )
            backup_quality_ok = _route_quality_ok(
                primary_route,
                backup_route,
                backup_max_step_ratio,
                backup_max_exposure_ratio,
            )
            recommended_backup_quality_ok = _route_quality_ok(
                recommended_primary_route,
                recommended_backup_route,
                backup_max_step_ratio,
                backup_max_exposure_ratio,
            )
            backup_status = _backup_status_label(backup_quality_ok, backup_route)
            recommended_backup_status = _backup_status_label(
                recommended_backup_quality_ok,
                recommended_backup_route,
            )

            zone_rows.append(
                {
                    "zone_id": int(z["zone_id"]),
                    "demand": int(z["demand"]),
                    "faculty_count": int(z["faculty_count"]),
                    "staff_count": int(z["staff_count"]),
                    "representative_role": role,
                    "representative_start_node": _serialize_node(rep_start),
                    "assigned_primary_shelter": _serialize_node(assigned_primary_shelter)
                    if assigned_primary_shelter is not None
                    else None,
                    "assigned_backup_shelter": _serialize_node(assigned_backup_shelter)
                    if assigned_backup_shelter is not None
                    else None,
                    "primary_shelter": _serialize_node(primary_shelter) if primary_shelter is not None else None,
                    "backup_shelter": _serialize_node(backup_shelter) if backup_shelter is not None else None,
                    "recommended_primary_shelter": _serialize_node(recommended_primary_shelter)
                    if recommended_primary_shelter is not None
                    else None,
                    "recommended_backup_shelter": _serialize_node(recommended_backup_shelter)
                    if recommended_backup_shelter is not None
                    else None,
                    "primary_remaining_after_assignment": z["primary_remaining_after_assignment"],
                    "primary_assigned_demand": z.get("primary_assigned_demand"),
                    "backup_assigned_demand": z.get("backup_assigned_demand"),
                    "overflow_demand": z.get("overflow_demand"),
                    "unassigned_demand": z.get("unassigned_demand"),
                    "primary_assignment_score": z.get("primary_assignment_score"),
                    "route_feasibility_filter_applied": True,
                    "backup_quality_filter_applied": True,
                    "feasible_route_count": len(feasible_routes),
                    "candidate_shelters": [_serialize_node(s) for s in candidate_shelters],
                    "backup_quality_ok": bool(backup_quality_ok),
                    "recommended_backup_quality_ok": bool(recommended_backup_quality_ok),
                    "backup_status": backup_status,
                    "recommended_backup_status": recommended_backup_status,
                    "backup_weak": bool(recommended_backup_status == "weak"),
                    "primary_route_quality_score": _route_quality_score(
                        primary_route,
                        step_weight=route_step_weight,
                        exposure_weight=route_exposure_weight,
                    ),
                    "backup_route_quality_score": _route_quality_score(
                        backup_route,
                        step_weight=route_step_weight,
                        exposure_weight=route_exposure_weight,
                    ),
                    "recommended_primary_route_quality_score": _route_quality_score(
                        recommended_primary_route,
                        step_weight=route_step_weight,
                        exposure_weight=route_exposure_weight,
                    ),
                    "recommended_backup_route_quality_score": _route_quality_score(
                        recommended_backup_route,
                        step_weight=route_step_weight,
                        exposure_weight=route_exposure_weight,
                    ),
                    "primary_route": _route_payload(primary_route),
                    "backup_route": _route_payload(backup_route),
                    "recommended_primary_route": _route_payload(recommended_primary_route),
                    "recommended_backup_route": _route_payload(recommended_backup_route),
                }
            )

    result = {
        "scenario": scenario.get("name", "scenario"),
        "checkpoint": args.checkpoint,
        "seed": int(args.seed),
        "num_zones": int(args.num_zones),
        "assignment_mode": "capacity_aware_demand_balancing",
        "route_selection_mode": "route_feasibility_filter",
        "primary_recommendation_mode": "route_quality_rerank",
        "backup_quality_filter_enabled": True,
        "backup_max_step_ratio": backup_max_step_ratio,
        "backup_max_exposure_ratio": backup_max_exposure_ratio,
        "route_step_weight": route_step_weight,
        "route_exposure_weight": route_exposure_weight,
        "zones_with_weak_backup": [
            int(z["zone_id"]) for z in zone_rows if bool(z.get("backup_weak"))
        ],
        "shelter_capacity_enabled": shelter_capacity_enabled,
        "shelter_capacity_per_site": shelter_capacity_per_site,
        "zones": zone_rows,
    }

    stem = f"{result['scenario']}_zone_routes"
    json_path = os.path.join(args.output_dir, f"{stem}.json")
    md_path = os.path.join(args.output_dir, f"{stem}.md")
    mgmt_path = os.path.join(args.output_dir, f"{stem}_management_summary.txt")

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)

    with open(md_path, "w", encoding="utf-8") as f:
        f.write("# Zone Route Recommendation\n\n")
        f.write(f"- Scenario: `{result['scenario']}`\n")
        f.write(f"- Checkpoint: `{result['checkpoint']}`\n")
        f.write(f"- Seed: `{result['seed']}`\n")
        f.write(f"- Num zones: `{result['num_zones']}`\n")
        f.write(f"- Route selection mode: `{result['route_selection_mode']}`\n\n")
        f.write(f"- Primary recommendation mode: `{result['primary_recommendation_mode']}`\n")
        f.write(f"- Backup quality filter enabled: `{result['backup_quality_filter_enabled']}`\n")
        f.write(f"- Backup max step ratio: `{result['backup_max_step_ratio']}`\n")
        f.write(f"- Backup max exposure ratio: `{result['backup_max_exposure_ratio']}`\n\n")
        f.write(f"- Route step weight: `{result['route_step_weight']}`\n")
        f.write(f"- Route exposure weight: `{result['route_exposure_weight']}`\n\n")
        f.write(f"- Zones with weak backup: `{result['zones_with_weak_backup']}`\n\n")
        f.write(f"- Shelter capacity enabled: `{result['shelter_capacity_enabled']}`\n")
        f.write(f"- Shelter capacity per site: `{result['shelter_capacity_per_site']}`\n\n")
        for z in result["zones"]:
            f.write(
                f"## zone_{z['zone_id']}\n\n"
                f"- Demand: `{z['demand']}`\n"
                f"- Faculty: `{z['faculty_count']}`\n"
                f"- Staff: `{z['staff_count']}`\n"
                f"- Representative role: `{z['representative_role']}`\n"
                f"- Representative start node: `{z['representative_start_node']}`\n"
                f"- Assigned primary shelter: `{z['assigned_primary_shelter']}`\n"
                f"- Assigned backup shelter: `{z['assigned_backup_shelter']}`\n"
                f"- Primary shelter: `{z['primary_shelter']}`\n"
                f"- Backup shelter: `{z['backup_shelter']}`\n"
                f"- Recommended primary shelter: `{z['recommended_primary_shelter']}`\n"
                f"- Recommended backup shelter: `{z['recommended_backup_shelter']}`\n"
                f"- Remaining after primary assignment: `{z['primary_remaining_after_assignment']}`\n"
                f"- Primary assigned demand: `{z['primary_assigned_demand']}`\n"
                f"- Backup assigned demand: `{z['backup_assigned_demand']}`\n"
                f"- Overflow demand: `{z['overflow_demand']}`\n"
                f"- Unassigned demand: `{z['unassigned_demand']}`\n"
                f"- Primary assignment score: `{z['primary_assignment_score']}`\n"
                f"- Feasible route count: `{z['feasible_route_count']}`\n"
                f"- Backup quality ok: `{z['backup_quality_ok']}`\n"
                f"- Recommended backup quality ok: `{z['recommended_backup_quality_ok']}`\n"
                f"- Backup status: `{z['backup_status']}`\n"
                f"- Recommended backup status: `{z['recommended_backup_status']}`\n"
                f"- Backup weak: `{z['backup_weak']}`\n"
                f"- Primary route quality score: `{z['primary_route_quality_score']}`\n"
                f"- Backup route quality score: `{z['backup_route_quality_score']}`\n"
                f"- Recommended primary route quality score: `{z['recommended_primary_route_quality_score']}`\n"
                f"- Recommended backup route quality score: `{z['recommended_backup_route_quality_score']}`\n"
            )
            if z["primary_route"] is not None:
                f.write(
                    f"- Primary route reached: `{z['primary_route']['reached']}`\n"
                    f"- Primary route steps: `{z['primary_route']['steps']}`\n"
                    f"- Primary route exposure: `{z['primary_route']['exposure']:.4f}`\n"
                    f"- Primary route path nodes: `{z['primary_route']['path_nodes']}`\n"
                )
            if z["backup_route"] is not None:
                f.write(
                    f"- Backup route reached: `{z['backup_route']['reached']}`\n"
                    f"- Backup route steps: `{z['backup_route']['steps']}`\n"
                    f"- Backup route exposure: `{z['backup_route']['exposure']:.4f}`\n"
                    f"- Backup route path nodes: `{z['backup_route']['path_nodes']}`\n"
                )
            if z["recommended_primary_route"] is not None:
                f.write(
                    f"- Recommended primary route reached: `{z['recommended_primary_route']['reached']}`\n"
                    f"- Recommended primary route steps: `{z['recommended_primary_route']['steps']}`\n"
                    f"- Recommended primary route exposure: `{z['recommended_primary_route']['exposure']:.4f}`\n"
                    f"- Recommended primary route path nodes: `{z['recommended_primary_route']['path_nodes']}`\n"
                )
            if z["recommended_backup_route"] is not None:
                f.write(
                    f"- Recommended backup route reached: `{z['recommended_backup_route']['reached']}`\n"
                    f"- Recommended backup route steps: `{z['recommended_backup_route']['steps']}`\n"
                    f"- Recommended backup route exposure: `{z['recommended_backup_route']['exposure']:.4f}`\n"
                    f"- Recommended backup route path nodes: `{z['recommended_backup_route']['path_nodes']}`\n"
                )
            f.write("\n")

    changed_primary_zones = [
        int(z["zone_id"])
        for z in zone_rows
        if z.get("assigned_primary_shelter") != z.get("recommended_primary_shelter")
    ]
    changed_backup_zones = [
        int(z["zone_id"])
        for z in zone_rows
        if z.get("assigned_backup_shelter") != z.get("recommended_backup_shelter")
    ]
    weak_zones = [int(z["zone_id"]) for z in zone_rows if bool(z.get("backup_weak"))]

    with open(mgmt_path, "w", encoding="utf-8") as f:
        f.write("Zone Route Recommendation Management Summary\n")
        f.write(f"Scenario: {result['scenario']}\n")
        f.write(f"Checkpoint: {result['checkpoint']}\n")
        f.write(f"Zones with weak backup: {weak_zones}\n")
        f.write(f"Zones with changed primary recommendation: {changed_primary_zones}\n")
        f.write(f"Zones with changed backup recommendation: {changed_backup_zones}\n")
        f.write("\n")
        f.write("Priority follow-up zones:\n")
        for z in zone_rows:
            if bool(z.get("backup_weak")) or z.get("assigned_primary_shelter") != z.get("recommended_primary_shelter"):
                f.write(
                    f"- zone_{z['zone_id']}: assigned_primary={z.get('assigned_primary_shelter')}, "
                    f"recommended_primary={z.get('recommended_primary_shelter')}, "
                    f"recommended_backup={z.get('recommended_backup_shelter')}, "
                    f"backup_status={z.get('recommended_backup_status')}, "
                    f"primary_score={z.get('recommended_primary_route_quality_score')}, "
                    f"backup_score={z.get('recommended_backup_route_quality_score')}\n"
                )

    print(f"[zone-route] json: {json_path}")
    print(f"[zone-route] md: {md_path}")
    print(f"[zone-route] management: {mgmt_path}")


if __name__ == "__main__":
    main()
