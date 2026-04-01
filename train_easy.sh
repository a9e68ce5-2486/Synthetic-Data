#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
OUT_DIR="${1:-logs/drqn_easy_pretrain}"
EPISODES="${2:-600}"

echo "[train_easy] output -> ${OUT_DIR}"
"${PYTHON_BIN}" drqn_minimal.py \
  --preset stable_train \
  --episodes "${EPISODES}" \
  --curriculum-mode distance \
  --curriculum-end-dist 1765 \
  --curriculum-freeze-episode -1 \
  --eps-decay 0.992 \
  --eps-end 0.01 \
  --target-update 800 \
  --w-exposure 0.3 \
  --best-min-episode 100 \
  --block-from-snow-threshold 1.5 \
  --block-from-snow-prob 0.0 \
  --output-dir "${OUT_DIR}"

echo "[train_easy] done"
echo "[train_easy] best: ${OUT_DIR}/drqn_torch_best.pt"

#./train_easy.sh
