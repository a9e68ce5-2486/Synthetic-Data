#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
OUT_DIR="${1:-logs/zone_management_batch}"
CHECKPOINT="${2:-logs/drqn_blocked_finetune/drqn_torch_best.pt}"
SEED="${3:-20260323}"
NUM_ZONES="${4:-6}"

echo "[zone-batch] output -> ${OUT_DIR}"
echo "[zone-batch] checkpoint -> ${CHECKPOINT}"

"${PYTHON_BIN}" run_zone_management_batch.py \
  --python-bin "${PYTHON_BIN}" \
  --output-dir "${OUT_DIR}" \
  --checkpoint "${CHECKPOINT}" \
  --seed "${SEED}" \
  --num-zones "${NUM_ZONES}"

echo "[zone-batch] done"
echo "[zone-batch] summary: ${OUT_DIR}/zone_management_batch_summary.json"
