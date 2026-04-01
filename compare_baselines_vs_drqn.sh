#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
SCENARIO="${1:-scenarios/enterprise_baseline.json}"
OUT_DIR="${2:-logs/baselines_vs_drqn}"
DRQN_CKPT="${3:-logs/drqn_blocked_finetune/drqn_torch_best.pt}"

echo "[compare] scenario -> ${SCENARIO}"
echo "[compare] drqn checkpoint -> ${DRQN_CKPT}"
"${PYTHON_BIN}" compare_baselines_vs_drqn.py \
  --scenario "${SCENARIO}" \
  --output-dir "${OUT_DIR}" \
  --drqn-checkpoint "${DRQN_CKPT}"

echo "[compare] done"
echo "[compare] summary: ${OUT_DIR}/baselines_vs_drqn_comparison.json"
