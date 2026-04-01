#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
TRAIN_OUT="${1:-logs/drqn_stable_v2_tune2}"
EVAL_OUT="${2:-${TRAIN_OUT}_eval}"

echo "[run] training (preset=stable_v2) -> ${TRAIN_OUT}"
"${PYTHON_BIN}" drqn_minimal.py \
  --preset stable_v2 \
  --output-dir "${TRAIN_OUT}"

echo "[run] evaluating best checkpoint -> ${EVAL_OUT}"
"${PYTHON_BIN}" eval_best_model.py \
  --checkpoint "${TRAIN_OUT}/drqn_torch_best.pt" \
  --output-dir "${EVAL_OUT}"

echo "[run] done"
echo "[run] train history: ${TRAIN_OUT}/drqn_torch_history.csv"
echo "[run] best model:    ${TRAIN_OUT}/drqn_torch_best.pt"
echo "[run] eval summary:  ${EVAL_OUT}/best_model_eval_summary.json"
