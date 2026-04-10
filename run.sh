#!/bin/bash
# run.sh — shortcuts for all demo commands
# Usage: ./run.sh <command>

set -a; source .env 2>/dev/null; set +a
PYTHON=./venv/bin/python3
CHECKPOINT=logs/drqn_llm_persona/drqn_torch_best.pt

case "$1" in

  demo)
    # Real-time animated simulation (blizzard by default)
    SCENARIO=${2:-blizzard}
    $PYTHON evacuation_main.py \
      --scenario scenarios/enterprise_${SCENARIO}.json \
      --policy drqn \
      --drqn-checkpoint $CHECKPOINT \
      --visual-speed-scale 20.0
    ;;

  ui)
    # Gradio web interface → http://localhost:7860
    $PYTHON advisor_ui.py
    ;;

  advise)
    # Personal Advisor CLI (interactive description prompt)
    $PYTHON personal_advisor.py \
      --scenario scenarios/enterprise_blizzard.json \
      --checkpoint $CHECKPOINT \
      --disaster-type blizzard --severity moderate \
      --start-node 4119764241 \
      --interactive
    ;;

  map)
    # Static folium route map (opens in browser)
    JSON=$(ls logs/personal_advisor/*.json 2>/dev/null | head -1)
    if [ -z "$JSON" ]; then
      echo "No saved advice JSON found. Run './run.sh advise' first."
      exit 1
    fi
    $PYTHON visualize_map.py \
      --mode route \
      --route-json "$JSON" \
      --checkpoint $CHECKPOINT \
      --scenario scenarios/enterprise_blizzard.json \
      --output logs/maps/route_map.html
    open logs/maps/route_map.html
    ;;

  api)
    # FastAPI REST server → http://localhost:8000/docs
    ADVISOR_SCENARIO=scenarios/enterprise_blizzard.json \
    ADVISOR_SEVERITY=moderate \
    ADVISOR_CHECKPOINT=$CHECKPOINT \
    $PYTHON -m uvicorn advisor_api:app --host 0.0.0.0 --port 8000
    ;;

  *)
    echo ""
    echo "Usage: ./run.sh <command> [option]"
    echo ""
    echo "  demo [blizzard|earthquake|compound]  Animated simulation (default: blizzard)"
    echo "  ui                                   Gradio web UI → http://localhost:7860"
    echo "  advise                               Personal Advisor CLI (interactive)"
    echo "  map                                  Static route map (opens in browser)"
    echo "  api                                  REST API server → http://localhost:8000/docs"
    echo ""
    ;;
esac
