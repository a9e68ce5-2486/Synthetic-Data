import math
from collections import defaultdict


def _percentile_reach_step(reached_steps, total_agents, q):
    if total_agents <= 0:
        return None
    threshold = math.ceil(total_agents * q)
    if len(reached_steps) < threshold:
        return None
    reached_steps = sorted(reached_steps)
    return reached_steps[threshold - 1]


def _persona_reached_rates(agents):
    """Compute per-persona reached_rate for all personas present in agents.

    Returns a dict: { "persona_<name>_reached_rate": float, ... }
    Only includes personas with at least 1 agent.
    """
    counts = defaultdict(int)
    reached = defaultdict(int)
    for a in agents:
        persona = getattr(a, "persona", None)
        if not persona:
            continue
        counts[persona] += 1
        if a.reached:
            reached[persona] += 1
    result = {}
    for persona, n in counts.items():
        result[f"persona_{persona}_reached_rate"] = round(reached[persona] / n, 4)
        result[f"persona_{persona}_count"] = n
    return result


def _role_reached_rates(agents):
    """Compute per-role reached_rate for all 4 role categories."""
    counts = defaultdict(int)
    reached = defaultdict(int)
    for a in agents:
        role = getattr(a, "role", None)
        if not role:
            continue
        counts[role] += 1
        if a.reached:
            reached[role] += 1
    result = {}
    for role, n in counts.items():
        result[f"role_{role}_reached_rate"] = round(reached[role] / n, 4)
        result[f"role_{role}_count"] = n
    return result


def summarize_run(step_rows, peds, cars, edge_counts, step_limit, scenario_name, seed, policy_name, extra_metrics=None):
    agents = list(peds) + list(cars)
    total = len(agents)
    reached = [a for a in agents if a.reached]
    alive = [a for a in agents if a.alive]
    reached_steps = [a.steps for a in reached]
    avg_exp = (sum(a.exposure for a in agents) / total) if total else 0.0

    # Legacy binary role tracking (backward compatibility)
    faculty = [a for a in agents if a.role == "faculty"]
    staff = [a for a in agents if a.role == "staff"]
    faculty_reached_rate = (sum(1 for a in faculty if a.reached) / len(faculty)) if faculty else 0.0
    staff_reached_rate = (sum(1 for a in staff if a.reached) / len(staff)) if staff else 0.0

    top_edges = sorted(edge_counts.items(), key=lambda kv: kv[1], reverse=True)[:10]
    top_edges_str = ";".join(f"{u}->{v}:{cnt}" for (u, v), cnt in top_edges)

    final_step = step_rows[-1] if step_rows else {"alive": len(alive), "reached": len(reached), "avg_exposure": avg_exp}
    summary = {
        "scenario": scenario_name,
        "seed": seed,
        "policy": policy_name,
        "step_limit": step_limit,
        "final_alive": final_step["alive"],
        "final_reached": final_step["reached"],
        "final_avg_exposure": round(final_step["avg_exposure"], 4),
        "alive_rate": round(len(alive) / total, 4) if total else 0.0,
        "reached_rate": round(len(reached) / total, 4) if total else 0.0,
        "avg_exposure_total": round(avg_exp, 4),
        "t90_step": _percentile_reach_step(reached_steps, total, 0.90),
        "t95_step": _percentile_reach_step(reached_steps, total, 0.95),
        "faculty_reached_rate": round(faculty_reached_rate, 4),
        "staff_reached_rate": round(staff_reached_rate, 4),
        "reach_rate_gap": round(faculty_reached_rate - staff_reached_rate, 4),
        "top_bottlenecks": top_edges_str,
    }
    # Per-persona and per-role breakdowns
    summary.update(_persona_reached_rates(agents))
    summary.update(_role_reached_rates(agents))

    if extra_metrics:
        summary.update(extra_metrics)
    return summary


def aggregate_policy_rows(rows):
    if not rows:
        return {}

    def _avg(field):
        values = [r[field] for r in rows]
        return sum(values) / len(values)

    def _avg_optional(field):
        values = [r[field] for r in rows if r.get(field) is not None]
        return round(sum(values) / len(values), 4) if values else None

    t95_values = [r["t95_step"] for r in rows if r["t95_step"] is not None]
    t90_values = [r["t90_step"] for r in rows if r["t90_step"] is not None]
    reassign_values = [r["shelter_reassignments"] for r in rows if r.get("shelter_reassignments") is not None]

    result = {
        "runs": len(rows),
        "avg_reached_rate": round(_avg("reached_rate"), 4),
        "avg_alive_rate": round(_avg("alive_rate"), 4),
        "avg_exposure_total": round(_avg("avg_exposure_total"), 4),
        "avg_t90_step": round(sum(t90_values) / len(t90_values), 2) if t90_values else None,
        "avg_t95_step": round(sum(t95_values) / len(t95_values), 2) if t95_values else None,
        "avg_faculty_reached_rate": _avg_optional("faculty_reached_rate"),
        "avg_staff_reached_rate": _avg_optional("staff_reached_rate"),
        "avg_reach_rate_gap": _avg_optional("reach_rate_gap"),
        "avg_shelter_reassignments": round(sum(reassign_values) / len(reassign_values), 2) if reassign_values else None,
        "shelter_capacity_enabled": any(bool(r.get("shelter_capacity_enabled", False)) for r in rows),
    }

    # Aggregate per-persona reached rates and counts across all runs
    persona_keys = set()
    persona_count_keys = set()
    role_keys = set()
    for r in rows:
        for k in r:
            if k.startswith("persona_") and k.endswith("_reached_rate"):
                persona_keys.add(k)
            if k.startswith("persona_") and k.endswith("_count"):
                persona_count_keys.add(k)
            if k.startswith("role_") and k.endswith("_reached_rate"):
                role_keys.add(k)
    for k in sorted(persona_keys):
        result[f"avg_{k}"] = _avg_optional(k)
    for k in sorted(persona_count_keys):
        result[k] = _avg_optional(k)   # avg agents/run for this persona
    for k in sorted(role_keys):
        result[f"avg_{k}"] = _avg_optional(k)

    return result
