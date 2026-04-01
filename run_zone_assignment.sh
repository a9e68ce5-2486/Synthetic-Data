#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
SCENARIO="${1:-scenarios/enterprise_baseline.json}"
SEED="${2:-20260323}"
NUM_ZONES="${3:-6}"

"${PYTHON_BIN}" zone_assignment.py \
  --scenario "${SCENARIO}" \
  --seed "${SEED}" \
  --num-zones "${NUM_ZONES}"
