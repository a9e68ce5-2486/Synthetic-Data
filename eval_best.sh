#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
CHECKPOINT="${1:-logs/drqn_stable_v2_tune2/drqn_torch_best.pt}"
OUT_DIR="${2:-${CHECKPOINT%/*}_eval}"

echo "[eval] checkpoint -> ${CHECKPOINT}"
"${PYTHON_BIN}" eval_best_model.py \
  --checkpoint "${CHECKPOINT}" \
  --output-dir "${OUT_DIR}"

echo "[eval] done"
echo "[eval] summary: ${OUT_DIR}/best_model_eval_summary.json"
