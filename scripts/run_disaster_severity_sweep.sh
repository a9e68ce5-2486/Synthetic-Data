#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-venv/bin/python}"
SCENARIO="${1:-scenarios/enterprise_blizzard.json}"
OUT_DIR="${2:-logs/disaster_severity_sweep}"
# Default checkpoint: persona-aware model trained with LLM severity params (obs_dim=42)
if [ -f "logs/drqn_llm_persona/drqn_torch_best.pt" ]; then
  DEFAULT_CKPT="logs/drqn_llm_persona/drqn_torch_best.pt"
elif [ -f "logs/drqn_progressive_severity_v3_extreme/drqn_torch_best.pt" ]; then
  DEFAULT_CKPT="logs/drqn_progressive_severity_v3_extreme/drqn_torch_best.pt"
else
  DEFAULT_CKPT="logs/drqn_progressive_severity/drqn_torch_best.pt"
fi
CHECKPOINT="${3:-${DEFAULT_CKPT}}"

echo "[severity] scenario -> ${SCENARIO}"
echo "[severity] output -> ${OUT_DIR}"
echo "[severity] checkpoint -> ${CHECKPOINT}"

"${PYTHON_BIN}" run_disaster_severity_sweep.py \
  --scenario "${SCENARIO}" \
  --output-dir "${OUT_DIR}" \
  --drqn-checkpoint "${CHECKPOINT}"

echo "[severity] done"
echo "[severity] summary: ${OUT_DIR}/disaster_severity_sweep.json"
