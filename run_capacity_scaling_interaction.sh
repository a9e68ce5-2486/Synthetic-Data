#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
OUT_DIR="${1:-logs/capacity_scaling_interaction_v1}"
DRQN_CKPT="${2:-logs/drqn_blocked_finetune/drqn_torch_best.pt}"

echo "[capacity-interaction] output -> ${OUT_DIR}"
echo "[capacity-interaction] drqn checkpoint -> ${DRQN_CKPT}"

"${PYTHON_BIN}" run_capacity_scaling.py \
  --interaction-aware \
  --output-dir "${OUT_DIR}" \
  --drqn-checkpoint "${DRQN_CKPT}"

echo "[capacity-interaction] done"
echo "[capacity-interaction] summary: ${OUT_DIR}/capacity_scaling_summary.json"
