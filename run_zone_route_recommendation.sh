#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
SCENARIO="${1:-scenarios/enterprise_baseline.json}"
CHECKPOINT="${2:-logs/drqn_blocked_finetune/drqn_torch_best.pt}"
SEED="${3:-20260323}"
NUM_ZONES="${4:-6}"

"${PYTHON_BIN}" zone_route_recommendation.py \
  --scenario "${SCENARIO}" \
  --checkpoint "${CHECKPOINT}" \
  --seed "${SEED}" \
  --num-zones "${NUM_ZONES}"
