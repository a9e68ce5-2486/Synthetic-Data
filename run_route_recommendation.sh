#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
SCENARIO="${1:-scenarios/enterprise_baseline.json}"
START_NODE="${2:-}"
CHECKPOINT="${3:-logs/drqn_blocked_finetune/drqn_torch_best.pt}"

if [[ -z "${START_NODE}" ]]; then
  echo "usage: ./run_route_recommendation.sh <scenario> <start_node> [checkpoint]" >&2
  exit 1
fi

"${PYTHON_BIN}" route_recommendation.py \
  --scenario "${SCENARIO}" \
  --start-node "${START_NODE}" \
  --checkpoint "${CHECKPOINT}"
