#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
CHECKPOINT="${1:-logs/drqn_blocked_finetune/drqn_torch_best.pt}"
OUT_DIR="${2:-${CHECKPOINT%/*}_multiseed_eval}"

echo "[eval-multiseed] checkpoint -> ${CHECKPOINT}"
"${PYTHON_BIN}" eval_multiseed.py \
  --checkpoint "${CHECKPOINT}" \
  --output-dir "${OUT_DIR}"

echo "[eval-multiseed] done"
echo "[eval-multiseed] summary: ${OUT_DIR}/multiseed_eval_summary.json"
