#!/usr/bin/env bash
# run_reachability_analysis.sh
#
# Diagnose why reached_rate drops under high disaster severity by classifying
# agents into: UNREACHABLE / BUDGET_LIMITED / POLICY_SOLVABLE.
#
# Usage:
#   ./run_reachability_analysis.sh [scenario] [checkpoint] [output_dir] [num_seeds]
#
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
SCENARIO="${1:-scenarios/enterprise_blizzard.json}"

# Default: prefer progressive model, else domain rand, else blocked finetune
if [ -f "logs/drqn_progressive_severity/drqn_torch_best.pt" ]; then
  DEFAULT_CKPT="logs/drqn_progressive_severity/drqn_torch_best.pt"
elif [ -f "logs/drqn_domain_rand/drqn_torch_best.pt" ]; then
  DEFAULT_CKPT="logs/drqn_domain_rand/drqn_torch_best.pt"
else
  DEFAULT_CKPT="logs/drqn_blocked_finetune/drqn_torch_best.pt"
fi
CHECKPOINT="${2:-${DEFAULT_CKPT}}"
OUT_DIR="${3:-logs/reachability_analysis}"
NUM_SEEDS="${4:-10}"

echo "[reachability] scenario  -> ${SCENARIO}"
echo "[reachability] checkpoint-> ${CHECKPOINT}"
echo "[reachability] output    -> ${OUT_DIR}"
echo "[reachability] seeds     -> ${NUM_SEEDS}"

"${PYTHON_BIN}" analyze_reachability.py \
  --scenario "${SCENARIO}" \
  --checkpoint "${CHECKPOINT}" \
  --num-seeds "${NUM_SEEDS}" \
  --output-dir "${OUT_DIR}"

echo ""
echo "[reachability] done"
echo "[reachability] results: ${OUT_DIR}/reachability_analysis.md"
