#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
OUT_DIR="${1:-logs/drqn_easy_adaptive}"
EPISODES="${2:-800}"

echo "[train_easy_adaptive] output -> ${OUT_DIR}"
"${PYTHON_BIN}" drqn_minimal.py \
  --preset stable_train \
  --episodes "${EPISODES}" \
  --curriculum-mode adaptive_distance \
  --curriculum-start-dist 700 \
  --curriculum-end-dist 1765 \
  --curriculum-adaptive-window 20 \
  --curriculum-advance-threshold 0.85 \
  --curriculum-regress-threshold 0.40 \
  --curriculum-adaptive-step 120 \
  --eps-decay 0.992 \
  --eps-end 0.01 \
  --target-update 800 \
  --w-exposure 0.3 \
  --best-min-episode 100 \
  --block-from-snow-threshold 1.5 \
  --block-from-snow-prob 0.0 \
  --output-dir "${OUT_DIR}"

echo "[train_easy_adaptive] done"
echo "[train_easy_adaptive] best: ${OUT_DIR}/drqn_torch_best.pt"
