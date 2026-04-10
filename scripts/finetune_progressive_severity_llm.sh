#!/usr/bin/env bash
# finetune_progressive_severity_llm.sh
#
# 4-stage progressive fine-tuning using LLM-generated blizzard severity parameters
# (from scenarios/llm_severity_presets.json).
#
# Key differences vs finetune_progressive_severity.sh:
#   1. Obs vector is now 42-dim (12 base+persona + 5×6 neighbors) — network is
#      larger than the original 37-dim model. Pass --init-checkpoint only if the
#      checkpoint was trained with the same obs_dim=42; otherwise train from scratch.
#   2. Persona sampling is enabled automatically each episode (see drqn_minimal.py).
#   3. LLM-calibrated params differ notably from hardcoded values:
#        - threshold is LOWER (roads block sooner for moderate/extreme)
#        - snow_prob is HIGHER (faster conversion from snow to closed)
#        - block_init is similar or lower (LLM judged initial damage realistically)
#
# LLM severity mapping (blizzard, from llm_severity_presets.json):
#   light:    block_init=0.00, threshold=0.80, snow_prob=0.010
#   moderate: block_init=0.01, threshold=0.40, snow_prob=0.020
#   severe:   block_init=0.01, threshold=0.60, snow_prob=0.030
#   extreme:  block_init=0.05, threshold=0.40, snow_prob=0.030
#
# Usage:
#   ./finetune_progressive_severity_llm.sh [init_ckpt] [output_dir] [eps_per_stage]
#
# To train from scratch (required when obs_dim changed 37→42):
#   ./finetune_progressive_severity_llm.sh "" logs/drqn_llm_persona 400
#
# To fine-tune from an existing 42-dim checkpoint:
#   ./finetune_progressive_severity_llm.sh logs/drqn_llm_persona/drqn_torch_best.pt logs/drqn_llm_persona_v2 300
#
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
INIT_CKPT="${1:-}"
OUT_DIR="${2:-logs/drqn_llm_persona}"
EPS="${3:-400}"

STAGE1_DIR="${OUT_DIR}/stage1_light"
STAGE2_DIR="${OUT_DIR}/stage2_moderate"
STAGE3_DIR="${OUT_DIR}/stage3_severe"
STAGE4_DIR="${OUT_DIR}/stage4_extreme"

echo "============================================================"
echo "[llm-progressive] LLM blizzard severity params + persona obs"
echo "[llm-progressive] init checkpoint : ${INIT_CKPT:-<none, train from scratch>}"
echo "[llm-progressive] output directory: ${OUT_DIR}"
echo "[llm-progressive] episodes/stage  : ${EPS}"
echo "[llm-progressive] obs_dim         : 42  (7 base + 5 persona + 5×6 neighbors)"
echo "============================================================"

# Build optional init-checkpoint flag
INIT_FLAG=""
if [ -n "${INIT_CKPT}" ] && [ -f "${INIT_CKPT}" ]; then
    INIT_FLAG="--init-checkpoint ${INIT_CKPT}"
fi

# ---------------------------------------------------------------------------
# Stage 1 – Light (LLM: block_init=0.00, threshold=0.80, snow_prob=0.010)
# ---------------------------------------------------------------------------
echo ""
echo "[stage 1/4] light severity (block_init=0.00, threshold=0.80, snow_prob=0.010)"
# shellcheck disable=SC2086
"${PYTHON_BIN}" drqn_minimal.py \
  --preset stable_train \
  --episodes "${EPS}" \
  --curriculum-mode distance \
  --curriculum-start-dist 600 \
  --curriculum-end-dist 2622 \
  --curriculum-freeze-episode -1 \
  --eps-start 0.6 \
  --eps-end 0.05 \
  --eps-decay 0.997 \
  --target-update 600 \
  --w-exposure 1.0 \
  --best-min-episode 60 \
  --block-init-prob 0.00 \
  --block-from-snow-threshold 0.80 \
  --block-from-snow-prob 0.010 \
  --step-budget-scale 0.35 \
  --step-budget-min 400 \
  --step-budget-max 1200 \
  ${INIT_FLAG} \
  --output-dir "${STAGE1_DIR}"

# ---------------------------------------------------------------------------
# Stage 2 – Moderate (LLM: block_init=0.01, threshold=0.40, snow_prob=0.020)
# Note: threshold=0.40 is much lower than the hardcoded 0.82 — roads block
# sooner once snow accumulates. eps is lowered slightly from stage 1.
# ---------------------------------------------------------------------------
echo ""
echo "[stage 2/4] moderate severity (block_init=0.01, threshold=0.40, snow_prob=0.020)"
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
  --best-min-episode 60 \
  --block-init-prob 0.01 \
  --block-from-snow-threshold 0.40 \
  --block-from-snow-prob 0.020 \
  --step-budget-scale 0.35 \
  --step-budget-min 400 \
  --step-budget-max 1200 \
  --init-checkpoint "${STAGE1_DIR}/drqn_torch_best.pt" \
  --output-dir "${STAGE2_DIR}"

# ---------------------------------------------------------------------------
# Stage 3 – Severe (LLM: block_init=0.01, threshold=0.60, snow_prob=0.030)
# threshold is slightly higher than moderate (0.60 vs 0.40) — LLM judged that
# heavy blizzards need more accumulation before roads close vs moderate.
# ---------------------------------------------------------------------------
echo ""
echo "[stage 3/4] severe severity (block_init=0.01, threshold=0.60, snow_prob=0.030)"
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
  --best-min-episode 70 \
  --block-init-prob 0.01 \
  --block-from-snow-threshold 0.60 \
  --block-from-snow-prob 0.030 \
  --step-budget-scale 0.35 \
  --step-budget-min 400 \
  --step-budget-max 1200 \
  --init-checkpoint "${STAGE2_DIR}/drqn_torch_best.pt" \
  --output-dir "${STAGE3_DIR}"

# ---------------------------------------------------------------------------
# Stage 4 – Extreme (LLM: block_init=0.05, threshold=0.40, snow_prob=0.030)
# High snow_prob (clamped at 0.030 = safety clamp max) + lowest threshold
# makes this the most road-degraded scenario.
# ---------------------------------------------------------------------------
echo ""
echo "[stage 4/4] extreme severity (block_init=0.05, threshold=0.40, snow_prob=0.030)"
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
  --best-min-episode 80 \
  --block-init-prob 0.05 \
  --block-from-snow-threshold 0.40 \
  --block-from-snow-prob 0.030 \
  --step-budget-scale 0.35 \
  --step-budget-min 400 \
  --step-budget-max 1200 \
  --init-checkpoint "${STAGE3_DIR}/drqn_torch_best.pt" \
  --output-dir "${STAGE4_DIR}"

# Expose the final best checkpoint at top-level for easy reference
ln -sfn "stage4_extreme/drqn_torch_best.pt" "${OUT_DIR}/drqn_torch_best.pt"

echo ""
echo "============================================================"
echo "[llm-progressive] DONE"
echo "  stage1 best : ${STAGE1_DIR}/drqn_torch_best.pt"
echo "  stage2 best : ${STAGE2_DIR}/drqn_torch_best.pt"
echo "  stage3 best : ${STAGE3_DIR}/drqn_torch_best.pt"
echo "  stage4 best : ${STAGE4_DIR}/drqn_torch_best.pt"
echo "  final best  : ${OUT_DIR}/drqn_torch_best.pt  (-> stage4)"
echo "============================================================"
echo ""
echo "Next: update batch_runner.py checkpoint path and run a 3-disaster sweep to"
echo "evaluate the new persona-aware model against all LLM severity presets."
