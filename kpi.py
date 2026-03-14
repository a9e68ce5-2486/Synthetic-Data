import math


def _percentile_reach_step(reached_steps, total_agents, q):
    if total_agents <= 0:
        return None
    threshold = math.ceil(total_agents * q)
    if len(reached_steps) < threshold:
        return None
    reached_steps = sorted(reached_steps)
    return reached_steps[threshold - 1]


def summarize_run(step_rows, peds, cars, edge_counts, step_limit, scenario_name, seed, policy_name):
    agents = list(peds) + list(cars)
    total = len(agents)
    reached = [a for a in agents if a.reached]
    alive = [a for a in agents if a.alive]
    reached_steps = [a.steps for a in reached]
    avg_exp = (sum(a.exposure for a in agents) / total) if total else 0.0

    faculty = [a for a in agents if a.role == "faculty"]
    staff = [a for a in agents if a.role == "staff"]
    faculty_reached_rate = (sum(1 for a in faculty if a.reached) / len(faculty)) if faculty else 0.0
    staff_reached_rate = (sum(1 for a in staff if a.reached) / len(staff)) if staff else 0.0

    top_edges = sorted(edge_counts.items(), key=lambda kv: kv[1], reverse=True)[:10]
    top_edges_str = ";".join(f"{u}->{v}:{cnt}" for (u, v), cnt in top_edges)

    final_step = step_rows[-1] if step_rows else {"alive": len(alive), "reached": len(reached), "avg_exposure": avg_exp}
    return {
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


def aggregate_policy_rows(rows):
    if not rows:
        return {}

    def _avg(field):
        values = [r[field] for r in rows]
        return sum(values) / len(values)

    t95_values = [r["t95_step"] for r in rows if r["t95_step"] is not None]
    return {
        "runs": len(rows),
        "avg_reached_rate": round(_avg("reached_rate"), 4),
        "avg_alive_rate": round(_avg("alive_rate"), 4),
        "avg_exposure_total": round(_avg("avg_exposure_total"), 4),
        "avg_t95_step": round(sum(t95_values) / len(t95_values), 2) if t95_values else None,
    }
