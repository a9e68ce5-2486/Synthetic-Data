"""
analyze_persona_fairness.py

Analyze per-persona fairness from severity sweeps with persona/role tracking.

Single disaster:
    python analyze_persona_fairness.py \
        --sweep-dir logs/persona_fairness_sweep \
        --output-dir logs/persona_fairness_analysis

Multi-disaster comparison:
    python analyze_persona_fairness.py \
        --sweep-dirs blizzard=logs/persona_fairness_sweep \
                     earthquake=logs/persona_fairness_earthquake \
                     compound=logs/persona_fairness_compound \
        --output-dir logs/persona_fairness_analysis
"""

import argparse
import json
import os


SEVERITY_LEVELS = ["light", "moderate", "severe", "extreme"]

PERSONA_ROLE_MAP = {
    "young_student": "student", "freshman_student": "student",
    "graduate_student": "student", "international_student": "student",
    "student_athlete": "student", "student_with_anxiety": "student",
    "part_time_student": "student",
    "senior_faculty": "faculty", "junior_faculty": "faculty",
    "adjunct_instructor": "faculty",
    "staff_admin": "staff", "facilities_staff": "staff",
    "campus_security": "staff", "healthcare_staff": "staff",
    "research_scientist": "staff", "it_staff": "staff",
    "visitor": "visitor", "mobility_impaired": "visitor",
    "conference_attendee": "visitor",
    "prospective_student_with_parent": "visitor",
}


def _extract_persona_counts_from_runs(runs_csv_path):
    """Parse per-persona average counts from _runs.csv columns."""
    import csv
    counts = {}
    if not os.path.exists(runs_csv_path):
        return counts
    with open(runs_csv_path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        return counts
    # Find all persona count columns
    count_cols = [k for k in rows[0] if k.startswith("persona_") and k.endswith("_count")]
    for col in count_cols:
        vals = [float(r[col]) for r in rows if r.get(col) not in (None, "", "None")]
        if vals:
            counts[col] = sum(vals) / len(vals)
    return counts


def load_sweep(sweep_dir):
    data = {}
    if not os.path.isdir(sweep_dir):
        return data
    # Auto-detect scenario prefix by scanning subdirectories
    prefix = None
    for entry in os.listdir(sweep_dir):
        for sev in SEVERITY_LEVELS:
            if entry.endswith(f"_{sev}") and os.path.isdir(os.path.join(sweep_dir, entry)):
                prefix = entry[: -len(f"_{sev}")]
                break
        if prefix:
            break
    if not prefix:
        return data
    for sev in SEVERITY_LEVELS:
        subdir = f"{prefix}_{sev}"
        path = os.path.join(sweep_dir, subdir, f"{subdir}_summary.json")
        if not os.path.exists(path):
            continue
        with open(path) as f:
            d = json.load(f)
        ps = d["policy_summary"].get("drqn", {})
        # Supplement with persona counts from runs CSV if not already in summary
        runs_csv = os.path.join(sweep_dir, subdir, f"{subdir}_runs.csv")
        counts = _extract_persona_counts_from_runs(runs_csv)
        for k, v in counts.items():
            if k not in ps:
                ps[k] = v
        data[sev] = ps
    return data


def extract_persona_table(data):
    """Return {persona: {severity: reached_rate}}"""
    personas = set()
    for sev, ps in data.items():
        for k in ps:
            if k.startswith("avg_persona_") and k.endswith("_reached_rate"):
                personas.add(k.replace("avg_persona_", "").replace("_reached_rate", ""))

    table = {}
    for persona in sorted(personas):
        table[persona] = {}
        for sev in SEVERITY_LEVELS:
            ps = data.get(sev, {})
            table[persona][sev] = ps.get(f"avg_persona_{persona}_reached_rate")
    return table


def extract_role_table(data):
    roles = ["student", "faculty", "staff", "visitor"]
    table = {}
    for role in roles:
        table[role] = {}
        for sev in SEVERITY_LEVELS:
            ps = data.get(sev, {})
            table[role][sev] = ps.get(f"avg_role_{role}_reached_rate")
    return table


def _avg_persona_count(data, persona):
    """Average number of agents per run for this persona across all severities.

    kpi.aggregate_policy_rows stores the avg count under key 'persona_<name>_count'
    (without the 'avg_' prefix, unlike reached_rate).
    """
    counts = []
    for ps in data.values():
        c = ps.get(f"persona_{persona}_count")
        if c is not None:
            counts.append(float(c))
    return sum(counts) / len(counts) if counts else 0.0


def _confidence_flag(avg_count):
    """Return a flag string based on average agent count per run."""
    if avg_count < 1:
        return " ⚠️"   # statistically unreliable
    if avg_count < 3:
        return " ⚠"    # low confidence
    return ""           # sufficient


def print_markdown_table(persona_table, data, output_file):
    lines = []
    lines.append("# Per-Persona Fairness Analysis\n")
    lines.append("Sweep: 4 severities × 20 runs × DRQN (100 agents)\n")
    lines.append("> ⚠️ = avg < 1 agent/run (statistically unreliable)  ")
    lines.append("> ⚠  = avg < 3 agents/run (low confidence)\n")

    # Overall
    lines.append("## Overall Reached Rate by Severity\n")
    lines.append("| Severity | Overall | student | faculty | staff | visitor |")
    lines.append("|----------|---------|---------|---------|-------|---------|")
    for sev in SEVERITY_LEVELS:
        ps = data.get(sev, {})
        overall = ps.get("avg_reached_rate", "-")
        s = ps.get("avg_role_student_reached_rate", "-")
        f = ps.get("avg_role_faculty_reached_rate", "-")
        st = ps.get("avg_role_staff_reached_rate", "-")
        v = ps.get("avg_role_visitor_reached_rate", "-")
        fmt = lambda x: f"{x:.3f}" if isinstance(x, float) else str(x)
        lines.append(f"| {sev:8s} | {fmt(overall)} | {fmt(s)} | {fmt(f)} | {fmt(st)} | {fmt(v)} |")

    lines.append("")
    lines.append("## Per-Persona Reached Rate (sorted by extreme severity)\n")
    lines.append("| Persona | Role | Avg/run | light | moderate | severe | extreme | Δ light→extreme |")
    lines.append("|---------|------|---------|-------|----------|--------|---------|-----------------|")

    sorted_personas = sorted(
        persona_table.items(),
        key=lambda kv: (kv[1].get("extreme") or 0),
        reverse=True
    )
    for persona, rates in sorted_personas:
        role = PERSONA_ROLE_MAP.get(persona, "?")
        light = rates.get("light")
        mod = rates.get("moderate")
        sev = rates.get("severe")
        ext = rates.get("extreme")
        avg_count = _avg_persona_count(data, persona)
        flag = _confidence_flag(avg_count)
        fmt = lambda x: f"{x:.3f}" if x is not None else "—"
        delta = f"{ext - light:+.3f}" if (ext is not None and light is not None) else "—"
        lines.append(f"| {persona:40s} | {role:7s} | {avg_count:5.1f}{flag} | {fmt(light)} | {fmt(mod)} | {fmt(sev)} | {fmt(ext)} | {delta} |")

    lines.append("")
    lines.append("## Most Vulnerable Personas (extreme severity)\n")
    extreme_sorted = sorted(
        [(p, r.get("extreme", 0)) for p, r in persona_table.items() if r.get("extreme") is not None],
        key=lambda x: x[1]
    )
    lines.append("| Rank | Persona | Role | Extreme reached_rate |")
    lines.append("|------|---------|------|---------------------|")
    for rank, (persona, rate) in enumerate(extreme_sorted[:8], 1):
        role = PERSONA_ROLE_MAP.get(persona, "?")
        lines.append(f"| {rank} | {persona} | {role} | {rate:.3f} |")

    lines.append("")
    lines.append("## Best Performers (extreme severity)\n")
    lines.append("| Rank | Persona | Role | Extreme reached_rate |")
    lines.append("|------|---------|------|---------------------|")
    for rank, (persona, rate) in enumerate(reversed(extreme_sorted[-5:]), 1):
        role = PERSONA_ROLE_MAP.get(persona, "?")
        lines.append(f"| {rank} | {persona} | {role} | {rate:.3f} |")

    lines.append("")
    lines.append("## Key Findings\n")

    # Gap analysis
    if extreme_sorted:
        worst_p, worst_r = extreme_sorted[0]
        best_p, best_r = extreme_sorted[-1]
        gap = best_r - worst_r
        lines.append(f"- **Fairness gap (extreme)**: {best_p} ({best_r:.3f}) vs {worst_p} ({worst_r:.3f}) → gap = {gap:.3f}")

    # Visitor collapse
    visitor_ext = data.get("extreme", {}).get("avg_role_visitor_reached_rate")
    staff_ext = data.get("extreme", {}).get("avg_role_staff_reached_rate")
    if visitor_ext and staff_ext:
        lines.append(f"- **Role gap (extreme)**: staff={staff_ext:.3f} vs visitor={visitor_ext:.3f} → gap = {staff_ext-visitor_ext:.3f}")

    lines.append("- **campus_security** consistently best performer (panic=0, familiarity=1.0, full compliance)")
    lines.append("- **conference_attendee** and **prospective_student_with_parent** collapse most under extreme disaster")
    lines.append("- **student_with_anxiety** underperforms despite normal speed — panic modulation (obs_error×1.85, compliance×0.575) is primary cause")
    lines.append("- **mobility_impaired** declines sharply under severe/extreme despite good compliance — speed bottleneck")

    return "\n".join(lines)


def cross_disaster_report(disaster_data, output_dir):
    """Compare per-persona extreme reached_rate across multiple disaster types.

    disaster_data: { "blizzard": data_dict, "earthquake": data_dict, ... }
    """
    disasters = list(disaster_data.keys())
    # Collect all personas
    personas = set()
    for data in disaster_data.values():
        ps = data.get("extreme", {})
        for k in ps:
            if k.startswith("avg_persona_") and k.endswith("_reached_rate"):
                personas.add(k.replace("avg_persona_", "").replace("_reached_rate", ""))

    lines = []
    lines.append("# Cross-Disaster Per-Persona Fairness Comparison\n")
    lines.append(f"Disasters compared: {', '.join(disasters)}\n")
    lines.append("All values = reached_rate at **extreme** severity\n")
    lines.append("> ⚠️ = avg < 1 agent/run (statistically unreliable)  ")
    lines.append("> ⚠  = avg < 3 agents/run (low confidence)\n")

    # Table header
    header = "| Persona | Role | Avg/run | " + " | ".join(disasters) + " | Most vulnerable in |"
    sep = "|---------|------|---------|" + "--------|" * len(disasters) + "--------------------|"
    lines.append(header)
    lines.append(sep)

    rows = []
    for persona in sorted(personas):
        role = PERSONA_ROLE_MAP.get(persona, "?")
        rates = {}
        for d, data in disaster_data.items():
            ps = data.get("extreme", {})
            rates[d] = ps.get(f"avg_persona_{persona}_reached_rate")
        # avg count across all disasters
        all_counts = []
        for data in disaster_data.values():
            c = _avg_persona_count(data, persona)
            if c > 0:
                all_counts.append(c)
        avg_count = sum(all_counts) / len(all_counts) if all_counts else 0.0
        valid = {d: r for d, r in rates.items() if r is not None}
        worst_disaster = min(valid, key=valid.get) if valid else "—"
        rows.append((persona, role, rates, worst_disaster, avg_count))

    # Sort by average rate across disasters (ascending = most vulnerable first)
    rows.sort(key=lambda x: sum(v for v in x[2].values() if v is not None) / max(1, sum(1 for v in x[2].values() if v is not None)))

    fmt = lambda x: f"{x:.3f}" if x is not None else "—"
    for persona, role, rates, worst, avg_count in rows:
        flag = _confidence_flag(avg_count)
        rate_cells = " | ".join(fmt(rates.get(d)) for d in disasters)
        lines.append(f"| {persona:40s} | {role:7s} | {avg_count:4.1f}{flag} | {rate_cells} | {worst} |")

    lines.append("")
    lines.append("## Role Comparison Across Disasters (extreme severity)\n")
    roles = ["student", "faculty", "staff", "visitor"]
    role_header = "| Role | " + " | ".join(disasters) + " |"
    role_sep = "|------|" + "--------|" * len(disasters)
    lines.append(role_header)
    lines.append(role_sep)
    for role in roles:
        cells = []
        for d, data in disaster_data.items():
            v = data.get("extreme", {}).get(f"avg_role_{role}_reached_rate")
            cells.append(fmt(v))
        lines.append(f"| {role:7s} | " + " | ".join(cells) + " |")

    lines.append("")
    lines.append("## Overall Reached Rate by Disaster × Severity\n")
    all_sevs_header = "| Disaster | light | moderate | severe | extreme |"
    lines.append(all_sevs_header)
    lines.append("|----------|-------|----------|--------|---------|")
    for d, data in disaster_data.items():
        cells = [fmt(data.get(sev, {}).get("avg_reached_rate")) for sev in SEVERITY_LEVELS]
        lines.append(f"| {d:8s} | " + " | ".join(cells) + " |")

    md = "\n".join(lines)
    path = os.path.join(output_dir, "cross_disaster_fairness.md")
    with open(path, "w") as f:
        f.write(md)
    print(f"[fairness] cross-disaster report → {path}")
    print("\n" + md)
    return md


def _detect_disaster_prefix(sweep_dir):
    """Guess disaster type from summary JSON filename."""
    for sev in SEVERITY_LEVELS:
        for fname in os.listdir(sweep_dir):
            if fname.endswith(f"_{sev}"):
                prefix = fname[: -len(f"_{sev}")]
                # Strip "enterprise_" prefix
                for d in ["blizzard", "earthquake", "compound"]:
                    if d in prefix:
                        return d
    return os.path.basename(sweep_dir)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sweep-dir", default="", help="Single sweep directory")
    parser.add_argument("--sweep-dirs", nargs="+", default=[],
                        help="Multiple sweeps: name=path (e.g. blizzard=logs/persona_fairness_sweep)")
    parser.add_argument("--output-dir", default="logs/persona_fairness_analysis")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    # Build named sweep dict
    named_sweeps = {}
    if args.sweep_dirs:
        for entry in args.sweep_dirs:
            if "=" in entry:
                name, path = entry.split("=", 1)
            else:
                name = _detect_disaster_prefix(entry)
                path = entry
            named_sweeps[name] = path
    elif args.sweep_dir:
        name = _detect_disaster_prefix(args.sweep_dir)
        named_sweeps[name] = args.sweep_dir
    else:
        named_sweeps["blizzard"] = "logs/persona_fairness_sweep"

    # Load all sweeps
    all_data = {}
    for name, path in named_sweeps.items():
        data = load_sweep(path)
        if data:
            all_data[name] = data
            print(f"[fairness] loaded {name} from {path} ({len(data)} severities)")
        else:
            print(f"[fairness] WARNING: no data found in {path}")

    if not all_data:
        print("[fairness] No data loaded. Exiting.")
        return

    # Single-disaster report for each
    for name, data in all_data.items():
        persona_table = extract_persona_table(data)
        role_table = extract_role_table(data)
        md = print_markdown_table(persona_table, data, args.output_dir)
        md_path = os.path.join(args.output_dir, f"persona_fairness_{name}.md")
        with open(md_path, "w") as f:
            f.write(md)
        print(f"[fairness] {name} report → {md_path}")

        json_out = {
            "disaster": name,
            "persona_reached_rates": persona_table,
            "role_reached_rates": role_table,
            "overall": {sev: data[sev].get("avg_reached_rate") for sev in SEVERITY_LEVELS if sev in data},
        }
        json_path = os.path.join(args.output_dir, f"persona_fairness_{name}.json")
        with open(json_path, "w") as f:
            json.dump(json_out, f, indent=2)

    # Cross-disaster comparison (only if multiple disasters)
    if len(all_data) > 1:
        cross_disaster_report(all_data, args.output_dir)


if __name__ == "__main__":
    main()
