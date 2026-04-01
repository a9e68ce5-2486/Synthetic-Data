#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
CKPT="${1:-logs/drqn_easy_pretrain/drqn_torch_best.pt}"

"${PYTHON_BIN}" evacuation_main.py \
  --policy drqn \
  --drqn-checkpoint "${CKPT}"

#./run_drqn_animation.sh

#reward
#= - time_cost
#- exposure_cost
#+ progress_reward
#+ checkpoint_reward
#+ shelter_arrival_reward
#+ early_arrival_bonus
#+ alive_bonus
#- invalid_or_blocked_penalty