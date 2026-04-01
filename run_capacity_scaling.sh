#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
OUT_DIR="${1:-logs/capacity_scaling}"
DRQN_CKPT="${2:-logs/drqn_blocked_finetune/drqn_torch_best.pt}"

echo "[capacity] output -> ${OUT_DIR}"
echo "[capacity] drqn checkpoint -> ${DRQN_CKPT}"

"${PYTHON_BIN}" run_capacity_scaling.py \
  --output-dir "${OUT_DIR}" \
  --drqn-checkpoint "${DRQN_CKPT}"

echo "[capacity] done"
echo "[capacity] summary: ${OUT_DIR}/capacity_scaling_summary.json"
