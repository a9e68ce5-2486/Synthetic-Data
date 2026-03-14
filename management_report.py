def _pct_change(new_value, base_value):
    if base_value in (None, 0):
        return None
    return (new_value - base_value) / base_value


def _fmt_pct(value):
    if value is None:
        return "N/A"
    return f"{value * 100:.1f}%"


def _score_policy(summary):
    # Higher reached rate is best; lower exposure is better as tie-breaker.
    reached = summary.get("avg_reached_rate", 0.0)
    exposure = summary.get("avg_exposure_total", float("inf"))
    return (reached, -exposure)


def build_management_summary(policy_summary, scenario_name, baseline_policy="round_robin"):
    if not policy_summary:
        return "No policy results available."

    if baseline_policy not in policy_summary:
        baseline_policy = next(iter(policy_summary.keys()))

    best_policy = max(policy_summary.items(), key=lambda kv: _score_policy(kv[1]))[0]
    best = policy_summary[best_policy]
    base = policy_summary[baseline_policy]

    reached_improve = _pct_change(best.get("avg_reached_rate"), base.get("avg_reached_rate"))
    exposure_improve = _pct_change(base.get("avg_exposure_total"), best.get("avg_exposure_total"))

    t95_best = best.get("avg_t95_step")
    t95_base = base.get("avg_t95_step")
    if t95_best is not None and t95_base is not None and t95_base > 0:
        t95_gain = (t95_base - t95_best) / t95_base
        t95_text = f"T95 reduced by {_fmt_pct(t95_gain)}"
    else:
        t95_text = "T95 not available"

    one_liner = (
        f"For scenario '{scenario_name}', best policy is '{best_policy}': "
        f"reached rate improvement vs '{baseline_policy}' = {_fmt_pct(reached_improve)}, "
        f"exposure improvement = {_fmt_pct(exposure_improve)}, {t95_text}."
    )

    lines = [
        "Management Summary",
        f"Scenario: {scenario_name}",
        f"Baseline Policy: {baseline_policy}",
        "",
        one_liner,
        "",
        "Policy Snapshot:",
    ]
    for name, stats in policy_summary.items():
        lines.append(
            f"- {name}: runs={stats.get('runs')}, "
            f"avg_reached_rate={stats.get('avg_reached_rate')}, "
            f"avg_alive_rate={stats.get('avg_alive_rate')}, "
            f"avg_exposure_total={stats.get('avg_exposure_total')}, "
            f"avg_t95_step={stats.get('avg_t95_step')}"
        )

    return "\n".join(lines)
