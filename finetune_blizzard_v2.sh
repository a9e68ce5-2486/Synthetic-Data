#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
INIT_CKPT="${1:-logs/drqn_blocked_finetune/drqn_torch_best.pt}"
OUT_DIR="${2:-logs/drqn_blizzard_finetune_v2}"
EPISODES_STAGE1="${3:-200}"
EPISODES_STAGE2="${4:-600}"

STAGE1_DIR="${OUT_DIR}/moderate_stage"
STAGE2_DIR="${OUT_DIR}/severe_stage"

echo "[finetune_blizzard_v2] init -> ${INIT_CKPT}"
echo "[finetune_blizzard_v2] output -> ${OUT_DIR}"
echo "[finetune_blizzard_v2] stage1 -> moderate (${EPISODES_STAGE1} episodes)"
"${PYTHON_BIN}" drqn_minimal.py \
  --preset stable_train \
  --episodes "${EPISODES_STAGE1}" \
  --curriculum-mode distance \
  --curriculum-start-dist 800 \
  --curriculum-end-dist 2622 \
  --curriculum-freeze-episode -1 \
  --eps-decay 0.996 \
  --eps-end 0.01 \
  --target-update 800 \
  --w-exposure 0.9 \
  --best-min-episode 100 \
  --block-from-snow-threshold 0.82 \
  --block-from-snow-prob 0.002 \
  --init-checkpoint "${INIT_CKPT}" \
  --output-dir "${STAGE1_DIR}"

echo "[finetune_blizzard_v2] stage2 -> severe (${EPISODES_STAGE2} episodes)"
"${PYTHON_BIN}" drqn_minimal.py \
  --preset stable_train \
  --episodes "${EPISODES_STAGE2}" \
  --curriculum-mode distance \
  --curriculum-start-dist 700 \
  --curriculum-end-dist 2622 \
  --curriculum-freeze-episode -1 \
  --eps-decay 0.996 \
  --eps-end 0.01 \
  --target-update 800 \
  --w-exposure 1.2 \
  --best-min-episode 100 \
  --block-from-snow-threshold 0.72 \
  --block-from-snow-prob 0.004 \
  --init-checkpoint "${STAGE1_DIR}/drqn_torch_best.pt" \
  --output-dir "${STAGE2_DIR}"

ln -sfn "severe_stage/drqn_torch_best.pt" "${OUT_DIR}/drqn_torch_best.pt"
ln -sfn "severe_stage/drqn_torch_final.pt" "${OUT_DIR}/drqn_torch_final.pt"

echo "[finetune_blizzard_v2] done"
echo "[finetune_blizzard_v2] moderate best: ${STAGE1_DIR}/drqn_torch_best.pt"
echo "[finetune_blizzard_v2] severe best: ${STAGE2_DIR}/drqn_torch_best.pt"
echo "[finetune_blizzard_v2] linked best: ${OUT_DIR}/drqn_torch_best.pt"
