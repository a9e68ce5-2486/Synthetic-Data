#!/usr/bin/env bash
# train_domain_rand.sh
#
# Fine-tune from a pretrained checkpoint using domain randomization.
# Each episode samples hazard parameters uniformly across the full severity
# range (light → extreme), producing a single robust model instead of a
# per-severity model.
#
# Domain randomization ranges (cover all severity presets):
#   block_init_prob  : Uniform(0.00, 0.12)   [light=0.00 … extreme=0.12]
#   snow_threshold   : Uniform(0.60, 0.92)   [extreme=0.60 … light=0.92]
#   snow_prob        : Uniform(0.001, 0.007)  [light=0.001 … extreme=0.007]
#
# Step-budget fix (2026-03-30, reachability analysis):
#   dist_p50≈600m → needs ~429 steps; dist_p90≈1500m → needs ~1071 steps.
#   Old: scale=0.08, min=100 → effective budget=100 steps (insufficient).
#   New: scale=0.35, min=400, max=1200 → budget=230~545 steps (realistic).
#
# Usage:
#   ./train_domain_rand.sh [init_ckpt] [output_dir] [episodes]
#
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
INIT_CKPT="${1:-logs/drqn_easy_pretrain/drqn_torch_best.pt}"
OUT_DIR="${2:-logs/drqn_domain_rand}"
EPISODES="${3:-800}"

echo "============================================================"
echo "[domain_rand] init checkpoint : ${INIT_CKPT}"
echo "[domain_rand] output directory: ${OUT_DIR}"
echo "[domain_rand] episodes        : ${EPISODES}"
echo "[domain_rand] block_init_prob : Uniform(0.00, 0.12)"
echo "[domain_rand] snow_threshold  : Uniform(0.60, 0.92)"
echo "[domain_rand] snow_prob       : Uniform(0.001, 0.007)"
echo "============================================================"

"${PYTHON_BIN}" drqn_minimal.py \
  --preset stable_train \
  --episodes "${EPISODES}" \
  --curriculum-mode adaptive_distance \
  --curriculum-start-dist 400 \
  --curriculum-end-dist 2622 \
  --curriculum-adaptive-window 20 \
  --curriculum-advance-threshold 0.75 \
  --curriculum-regress-threshold 0.35 \
  --curriculum-adaptive-step 150 \
  --eps-start 0.6 \
  --eps-end 0.05 \
  --eps-decay 0.997 \
  --target-update 600 \
  --w-exposure 1.5 \
  --best-min-episode 80 \
  --block-init-prob 0.0 \
  --block-from-snow-threshold 0.85 \
  --block-from-snow-prob 0.001 \
  --domain-rand true \
  --domain-rand-block-init-max 0.12 \
  --domain-rand-snow-threshold-min 0.60 \
  --domain-rand-snow-threshold-max 0.92 \
  --domain-rand-snow-prob-max 0.007 \
  --step-budget-scale 0.35 \
  --step-budget-min 400 \
  --step-budget-max 1200 \
  --init-checkpoint "${INIT_CKPT}" \
  --output-dir "${OUT_DIR}"

echo ""
echo "[domain_rand] DONE"
echo "  best checkpoint: ${OUT_DIR}/drqn_torch_best.pt"
