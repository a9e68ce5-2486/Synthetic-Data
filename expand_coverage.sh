#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
INIT_CKPT="${1:-logs/drqn_easy_pretrain/drqn_torch_best.pt}"
OUT_DIR="${2:-logs/drqn_coverage_expand}"
EPISODES="${3:-600}"

echo "[expand_coverage] init -> ${INIT_CKPT}"
echo "[expand_coverage] output -> ${OUT_DIR}"
"${PYTHON_BIN}" drqn_minimal.py \
  --preset stable_v2 \
  --episodes "${EPISODES}" \
  --curriculum-mode coverage \
  --curriculum-start-coverage 0.2 \
  --curriculum-end-coverage 1.0 \
  --curriculum-freeze-episode -1 \
  --eps-decay 0.997 \
  --target-update 800 \
  --w-exposure 0.5 \
  --best-min-episode 100 \
  --init-checkpoint "${INIT_CKPT}" \
  --output-dir "${OUT_DIR}"

echo "[expand_coverage] done"
echo "[expand_coverage] best: ${OUT_DIR}/drqn_torch_best.pt"
