#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
INIT_CKPT="${1:-logs/drqn_easy_pretrain/drqn_torch_best.pt}"
OUT_DIR="${2:-logs/drqn_full_finetune}"

echo "[finetune_full] init -> ${INIT_CKPT}"
echo "[finetune_full] output -> ${OUT_DIR}"
"${PYTHON_BIN}" drqn_minimal.py \
  --preset stable_train \
  --episodes 1000 \
  --curriculum-mode distance \
  --curriculum-end-dist 2622 \
  --curriculum-freeze-episode -1 \
  --eps-decay 0.997 \
  --target-update 800 \
  --w-exposure 0.7 \
  --best-min-episode 100 \
  --init-checkpoint "${INIT_CKPT}" \
  --output-dir "${OUT_DIR}"

echo "[finetune_full] done"
echo "[finetune_full] best: ${OUT_DIR}/drqn_torch_best.pt"
