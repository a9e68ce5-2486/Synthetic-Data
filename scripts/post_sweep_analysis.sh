#!/usr/bin/env bash
# post_sweep_analysis.sh
# Waits for llm_persona_sweep to finish, then runs fairness analysis
# and updates the paper table with final numbers.
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
SWEEP_LOG="logs/llm_persona_sweep.log"
BLIZZARD_DIR="logs/llm_persona_sweep_blizzard"
EARTHQUAKE_DIR="logs/llm_persona_sweep_earthquake"
COMPOUND_DIR="logs/llm_persona_sweep_compound"
FAIRNESS_OUT="logs/llm_persona_fairness"
TEX_FILE="paper/capstone_final.tex"

echo "[post-sweep] Waiting for sweep to complete..."
while true; do
    if grep -q "ALL DONE" "${SWEEP_LOG}" 2>/dev/null; then
        echo "[post-sweep] Sweep complete. Starting analysis..."
        break
    fi
    sleep 30
done

# ── 1. Fairness analysis ─────────────────────────────────────────────────────
echo "[post-sweep] Running persona fairness analysis..."
mkdir -p "${FAIRNESS_OUT}"

for DISASTER in blizzard earthquake compound; do
    SWEEP_DIR="logs/llm_persona_sweep_${DISASTER}"
    if [ -d "${SWEEP_DIR}" ]; then
        "${PYTHON_BIN}" analyze_persona_fairness.py \
            --sweep-dir "${SWEEP_DIR}" \
            --output-dir "${FAIRNESS_OUT}" \
            --disaster "${DISASTER}" \
            2>&1 | tee -a "${FAIRNESS_OUT}/fairness_${DISASTER}.log"
        echo "[post-sweep] fairness done: ${DISASTER}"
    fi
done

# ── 2. Extract numbers for paper ─────────────────────────────────────────────
echo "[post-sweep] Extracting results for paper..."
"${PYTHON_BIN}" - <<'PYEOF'
import json, os

results = {}
for disaster in ["blizzard", "earthquake", "compound"]:
    path = f"logs/llm_persona_sweep_{disaster}/disaster_severity_sweep.json"
    if not os.path.exists(path):
        print(f"[warn] missing: {path}")
        continue
    with open(path) as f:
        data = json.load(f)
    results[disaster] = {}
    for row in data["rows"]:
        sev = row["severity"]
        results[disaster][sev] = {
            "reached": row["avg_reached_rate"],
            "exposure": row["avg_exposure_total"],
        }

# Print table rows for paper
print("\n=== Paper Table 3 (persona-aware DRQN, 40 agents) ===")
for disaster, sevs in results.items():
    print(f"\n{disaster.capitalize()}")
    for sev in ["light", "moderate", "severe", "extreme"]:
        if sev in sevs:
            r = sevs[sev]["reached"]
            e = sevs[sev]["exposure"]
            print(f"  {sev:10}  reached={r:.3f}  exposure={e:.1f}")

# Save as JSON for paper update
with open("logs/llm_persona_fairness/paper_table_numbers.json", "w") as f:
    json.dump(results, f, indent=2)
print("\n[done] saved to logs/llm_persona_fairness/paper_table_numbers.json")
PYEOF

# ── 3. Update paper table with actual numbers ─────────────────────────────────
echo "[post-sweep] Updating paper table..."
"${PYTHON_BIN}" - <<'PYEOF'
import json, re

with open("logs/llm_persona_fairness/paper_table_numbers.json") as f:
    results = json.load(f)

with open("paper/capstone_final.tex") as f:
    tex = f.read()

# Replace (in progress) rows with actual numbers
for disaster in ["earthquake", "compound"]:
    if disaster not in results:
        continue
    cap = disaster.capitalize()
    for sev in ["light", "moderate", "severe", "extreme"]:
        if sev not in results[disaster]:
            continue
        r = results[disaster][sev]["reached"]
        e = results[disaster][sev]["exposure"]
        old = f"  & {sev.capitalize():<8} & \\\\multicolumn{{2}}{{c}}{{\\\\textit{{(in progress)}}}} \\\\\\\\"
        new = f"  & {sev.capitalize():<8} & {r:.3f} & {e:.1f} \\\\"
        # Use simpler string replacement
        old_pat = f"& {sev.capitalize():<8} & \\multicolumn{{2}}{{c}}{{\\textit{{(in progress)}}}}"
        new_val = f"& {sev.capitalize():<8} & {r:.3f} & {e:.1f}"
        tex = tex.replace(old_pat, new_val)

with open("paper/capstone_final.tex", "w") as f:
    f.write(tex)
print("[done] paper/capstone_final.tex updated")
PYEOF

echo ""
echo "============================================================"
echo "[post-sweep] ALL DONE"
echo "  Fairness results : ${FAIRNESS_OUT}/"
echo "  Paper numbers    : paper/capstone_final.tex (updated)"
echo "  Run numbers      : logs/llm_persona_fairness/paper_table_numbers.json"
echo "============================================================"
