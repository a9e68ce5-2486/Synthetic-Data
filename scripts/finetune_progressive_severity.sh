#!/usr/bin/env bash
# finetune_progressive_severity.sh
#
# 4-stage progressive fine-tuning that matches the training distribution to the
# actual evaluation severity presets (light → moderate → severe → extreme).
#
# Key fix vs previous finetune scripts:
#   1. block_init_prob matches EVAC_BLOCK_INIT_PROB for each severity level.
#   2. step_budget params corrected based on reachability analysis (2026-03-30):
#        dist_p50≈600m → needs ~429 steps; dist_p90≈1500m → needs ~1071 steps.
#        Old: scale=0.08 → budget=68 steps (clamped to min=100) — WAY too small.
#        New: scale=0.35, min=400, max=1200 → budget=230~545 steps (realistic).
#
# Severity mapping (from scenario_loader.py):
#   light:    block_init_prob=0.00, threshold=0.92, snow_prob=0.001
#   moderate: block_init_prob=0.02, threshold=0.82, snow_prob=0.002
#   severe:   block_init_prob=0.06, threshold=0.72, snow_prob=0.004
#   extreme:  block_init_prob=0.12, threshold=0.60, snow_prob=0.007
#
# Usage:
#   ./finetune_progressive_severity.sh [init_ckpt] [output_dir] [eps_per_stage]
#
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
INIT_CKPT="${1:-logs/drqn_easy_pretrain/drqn_torch_best.pt}"
OUT_DIR="${2:-logs/drqn_progressive_severity}"
EPS="${3:-300}"

STAGE1_DIR="${OUT_DIR}/stage1_light"
STAGE2_DIR="${OUT_DIR}/stage2_moderate"
STAGE3_DIR="${OUT_DIR}/stage3_severe"
STAGE4_DIR="${OUT_DIR}/stage4_extreme"

echo "============================================================"
echo "[progressive] init checkpoint : ${INIT_CKPT}"
echo "[progressive] output directory: ${OUT_DIR}"
echo "[progressive] episodes/stage  : ${EPS}"
echo "============================================================"

# ---------------------------------------------------------------------------
# Stage 1 – Light (matches light blizzard severity preset)
#   EVAC_BLOCK_INIT_PROB=0.00, threshold=0.92, snow_prob=0.001
# ---------------------------------------------------------------------------
echo ""
echo "[stage 1/4] light severity (block_init=0.00, threshold=0.92, snow_prob=0.001)"
"${PYTHON_BIN}" drqn_minimal.py \
  --preset stable_train \
  --episodes "${EPS}" \
  --curriculum-mode distance \
  --curriculum-start-dist 600 \
  --curriculum-end-dist 2622 \
  --curriculum-freeze-episode -1 \
  --eps-start 0.5 \
  --eps-end 0.05 \
  --eps-decay 0.997 \
  --target-update 600 \
  --w-exposure 1.0 \
  --best-min-episode 50 \
  --block-init-prob 0.00 \
  --block-from-snow-threshold 0.92 \
  --block-from-snow-prob 0.001 \
  --step-budget-scale 0.35 \
  --step-budget-min 400 \
  --step-budget-max 1200 \
  --init-checkpoint "${INIT_CKPT}" \
  --output-dir "${STAGE1_DIR}"

# ---------------------------------------------------------------------------
# Stage 2 – Moderate (matches moderate blizzard severity preset)
#   EVAC_BLOCK_INIT_PROB=0.02, threshold=0.82, snow_prob=0.002
# ---------------------------------------------------------------------------
echo ""
echo "[stage 2/4] moderate severity (block_init=0.02, threshold=0.82, snow_prob=0.002)"
"${PYTHON_BIN}" drqn_minimal.py \
  --preset stable_train \
  --episodes "${EPS}" \
  --curriculum-mode distance \
  --curriculum-start-dist 500 \
  --curriculum-end-dist 2622 \
  --curriculum-freeze-episode -1 \
  --eps-start 0.4 \
  --eps-end 0.05 \
  --eps-decay 0.997 \
  --target-update 600 \
  --w-exposure 1.2 \
  --best-min-episode 50 \
  --block-init-prob 0.02 \
  --block-from-snow-threshold 0.82 \
  --block-from-snow-prob 0.002 \
  --step-budget-scale 0.35 \
  --step-budget-min 400 \
  --step-budget-max 1200 \
  --init-checkpoint "${STAGE1_DIR}/drqn_torch_best.pt" \
  --output-dir "${STAGE2_DIR}"

# ---------------------------------------------------------------------------
# Stage 3 – Severe (matches severe blizzard severity preset)
#   EVAC_BLOCK_INIT_PROB=0.06, threshold=0.72, snow_prob=0.004
# ---------------------------------------------------------------------------
echo ""
echo "[stage 3/4] severe severity (block_init=0.06, threshold=0.72, snow_prob=0.004)"
"${PYTHON_BIN}" drqn_minimal.py \
  --preset stable_train \
  --episodes "${EPS}" \
  --curriculum-mode distance \
  --curriculum-start-dist 400 \
  --curriculum-end-dist 2622 \
  --curriculum-freeze-episode -1 \
  --eps-start 0.35 \
  --eps-end 0.05 \
  --eps-decay 0.996 \
  --target-update 700 \
  --w-exposure 1.5 \
  --best-min-episode 60 \
  --block-init-prob 0.06 \
  --block-from-snow-threshold 0.72 \
  --block-from-snow-prob 0.004 \
  --step-budget-scale 0.35 \
  --step-budget-min 400 \
  --step-budget-max 1200 \
  --init-checkpoint "${STAGE2_DIR}/drqn_torch_best.pt" \
  --output-dir "${STAGE3_DIR}"

# ---------------------------------------------------------------------------
# Stage 4 – Extreme (matches extreme blizzard severity preset)
#   EVAC_BLOCK_INIT_PROB=0.12, threshold=0.60, snow_prob=0.007
# ---------------------------------------------------------------------------
echo ""
echo "[stage 4/4] extreme severity (block_init=0.12, threshold=0.60, snow_prob=0.007)"
"${PYTHON_BIN}" drqn_minimal.py \
  --preset stable_train \
  --episodes "${EPS}" \
  --curriculum-mode distance \
  --curriculum-start-dist 300 \
  --curriculum-end-dist 2622 \
  --curriculum-freeze-episode -1 \
  --eps-start 0.30 \
  --eps-end 0.05 \
  --eps-decay 0.995 \
  --target-update 700 \
  --w-exposure 1.8 \
  --best-min-episode 60 \
  --block-init-prob 0.12 \
  --block-from-snow-threshold 0.60 \
  --block-from-snow-prob 0.007 \
  --step-budget-scale 0.35 \
  --step-budget-min 400 \
  --step-budget-max 1200 \
  --init-checkpoint "${STAGE3_DIR}/drqn_torch_best.pt" \
  --output-dir "${STAGE4_DIR}"

# Expose the final best checkpoint at top-level for easy reference
ln -sfn "stage4_extreme/drqn_torch_best.pt" "${OUT_DIR}/drqn_torch_best.pt"

echo ""
echo "============================================================"
echo "[progressive] DONE"
echo "  stage1 best : ${STAGE1_DIR}/drqn_torch_best.pt"
echo "  stage2 best : ${STAGE2_DIR}/drqn_torch_best.pt"
echo "  stage3 best : ${STAGE3_DIR}/drqn_torch_best.pt"
echo "  stage4 best : ${STAGE4_DIR}/drqn_torch_best.pt"
echo "  final best  : ${OUT_DIR}/drqn_torch_best.pt  (-> stage4)"
echo "============================================================"
