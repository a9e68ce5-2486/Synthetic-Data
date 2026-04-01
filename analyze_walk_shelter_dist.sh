#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
SCENARIO="${1:-scenarios/enterprise_blizzard_quick.json}"
OUT_JSON="${2:-logs/walk_shelter_dist_summary.json}"

mkdir -p "$(dirname "${OUT_JSON}")"

"${PYTHON_BIN}" analyze_walk_shelter_dist.py \
  --scenario "${SCENARIO}" \
  --output "${OUT_JSON}"
