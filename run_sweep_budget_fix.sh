#!/usr/bin/env bash
# run_sweep_budget_fix.sh
#
# Validate the step-budget fix by running severity sweeps for both new models
# (progressive_v2 and domain_rand_v2) and printing a before/after comparison.
#
# Usage:
#   ./run_sweep_budget_fix.sh [scenario]
#
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
SCENARIO="${1:-scenarios/enterprise_blizzard.json}"
BASELINE_CKPT="logs/drqn_blocked_finetune/drqn_torch_best.pt"
PROGRESSIVE_CKPT="logs/drqn_progressive_severity_v2/drqn_torch_best.pt"
DOMAIN_RAND_CKPT="logs/drqn_domain_rand_v2/drqn_torch_best.pt"

echo "========================================================"
echo "[sweep_budget_fix] scenario: ${SCENARIO}"
echo "[sweep_budget_fix] will sweep:"
echo "  baseline    : ${BASELINE_CKPT}"
echo "  progressive : ${PROGRESSIVE_CKPT}"
echo "  domain_rand : ${DOMAIN_RAND_CKPT}"
echo "========================================================"

run_sweep() {
  local label="$1"
  local ckpt="$2"
  local out="$3"
  if [ ! -f "${ckpt}" ]; then
    echo "[sweep_budget_fix] SKIP ${label}: checkpoint not found (${ckpt})"
    return
  fi
  echo ""
  echo "[sweep_budget_fix] running ${label} ..."
  "${PYTHON_BIN}" run_disaster_severity_sweep.py \
    --scenario "${SCENARIO}" \
    --output-dir "${out}" \
    --drqn-checkpoint "${ckpt}"
  echo "[sweep_budget_fix] ${label} done -> ${out}/disaster_severity_sweep.json"
}

run_sweep "progressive_v2"  "${PROGRESSIVE_CKPT}"  "logs/severity_sweep_progressive_v2"
run_sweep "domain_rand_v2"  "${DOMAIN_RAND_CKPT}"  "logs/severity_sweep_domain_rand_v2"

# Print comparison table
echo ""
echo "========================================================"
echo "[sweep_budget_fix] COMPARISON TABLE"
echo "========================================================"
"${PYTHON_BIN}" - <<'PYEOF'
import json, os

COLS = ["light", "moderate", "severe", "extreme"]

def load(path):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        data = json.load(f)
    return {r["severity"]: r for r in data.get("rows", [])}

sources = [
    ("baseline",       "logs/disaster_severity_sweep/disaster_severity_sweep.json"),
    ("progressive",    "logs/severity_sweep_progressive/disaster_severity_sweep.json"),
    ("domain_rand",    "logs/severity_sweep_domain_rand/disaster_severity_sweep.json"),
    ("progressive_v2", "logs/severity_sweep_progressive_v2/disaster_severity_sweep.json"),
    ("domain_rand_v2", "logs/severity_sweep_domain_rand_v2/disaster_severity_sweep.json"),
]

print(f"\n{'model':18} {'light':>8} {'moderate':>10} {'severe':>8} {'extreme':>9}")
print("-" * 58)
for label, path in sources:
    data = load(path)
    if data is None:
        continue
    vals = [f"{data[s]['avg_reached_rate']:.4f}" if s in data else "  n/a  " for s in COLS]
    print(f"{label:18} {vals[0]:>8} {vals[1]:>10} {vals[2]:>8} {vals[3]:>9}")
print()
PYEOF

echo "[sweep_budget_fix] done"
