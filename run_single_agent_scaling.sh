#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
OUT_DIR="${1:-logs/single_agent_scaling}"
DRQN_CKPT="${2:-logs/drqn_blocked_finetune/drqn_torch_best.pt}"

echo "[scaling] output -> ${OUT_DIR}"
echo "[scaling] drqn checkpoint -> ${DRQN_CKPT}"

"${PYTHON_BIN}" run_single_agent_scaling.py \
  --output-dir "${OUT_DIR}" \
  --drqn-checkpoint "${DRQN_CKPT}"

echo "[scaling] done"
echo "[scaling] summary: ${OUT_DIR}/single_agent_scaling_summary.json"
