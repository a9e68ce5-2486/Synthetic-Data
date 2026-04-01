#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
INIT_CKPT="${1:-logs/drqn_easy_adaptive/drqn_torch_best.pt}"
OUT_DIR="${2:-logs/drqn_blocked_finetune}"
EPISODES="${3:-600}"

echo "[finetune_blocked] init -> ${INIT_CKPT}"
echo "[finetune_blocked] output -> ${OUT_DIR}"
"${PYTHON_BIN}" drqn_minimal.py \
  --preset stable_train \
  --episodes "${EPISODES}" \
  --curriculum-mode distance \
  --curriculum-end-dist 2622 \
  --curriculum-freeze-episode -1 \
  --eps-decay 0.997 \
  --target-update 800 \
  --w-exposure 0.7 \
  --best-min-episode 100 \
  --block-from-snow-threshold 0.85 \
  --block-from-snow-prob 0.003 \
  --init-checkpoint "${INIT_CKPT}" \
  --output-dir "${OUT_DIR}"

echo "[finetune_blocked] done"
echo "[finetune_blocked] best: ${OUT_DIR}/drqn_torch_best.pt"
