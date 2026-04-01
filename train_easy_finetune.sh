#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
INIT_CKPT="${1:-logs/drqn_easy_long/drqn_torch_best.pt}"
OUT_DIR="${2:-logs/drqn_easy_continue}"
EPISODES="${3:-600}"

echo "[train_easy_continue] init -> ${INIT_CKPT}"
echo "[train_easy_continue] output -> ${OUT_DIR}"
"${PYTHON_BIN}" drqn_minimal.py \
  --preset stable_train \
  --episodes "${EPISODES}" \
  --curriculum-mode distance \
  --curriculum-end-dist 1765 \
  --curriculum-full-at-episode 1800 \
  --curriculum-freeze-episode -1 \
  --eps-decay 0.995 \
  --eps-end 0.01 \
  --target-update 800 \
  --w-exposure 0.3 \
  --best-min-episode 100 \
  --block-from-snow-threshold 1.5 \
  --block-from-snow-prob 0.0 \
  --init-checkpoint "${INIT_CKPT}" \
  --output-dir "${OUT_DIR}"

echo "[train_easy_continue] done"
echo "[train_easy_continue] best: ${OUT_DIR}/drqn_torch_best.pt"
