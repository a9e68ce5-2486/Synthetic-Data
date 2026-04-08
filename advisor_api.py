"""
advisor_api.py — FastAPI service for the Personal Evacuation Advisor

Wraps the three-layer pipeline (LLM behavior profiling → DRQN navigation →
LLM recommendation) as a REST API.

Endpoints:
  GET  /health                  — liveness check
  GET  /scenarios               — list available scenario files
  GET  /nodes/random            — sample a random valid walk-graph node
  POST /advise                  — run full pipeline, return profile + route + recommendation

Startup:
  The walk graph (EvacEnv) is loaded ONCE at startup using the scenario and
  severity specified via environment variables or --scenario / --severity flags.
  Restarting the server is required if you want to switch scenarios.

Usage:
    # Default scenario (enterprise_blizzard, moderate)
    uvicorn advisor_api:app --host 0.0.0.0 --port 8000

    # Custom scenario
    ADVISOR_SCENARIO=scenarios/enterprise_blizzard.json \\
    ADVISOR_SEVERITY=severe \\
    ADVISOR_CHECKPOINT=logs/drqn_progressive_severity_v3_extreme/drqn_torch_best.pt \\
    uvicorn advisor_api:app --host 0.0.0.0 --port 8000 --reload

    # Quick smoke-test (no LLM):
    curl -X POST http://localhost:8000/advise \\
      -H "Content-Type: application/json" \\
      -d '{"start_node": 1638160433, "severity": "moderate"}'
"""

import glob as _glob
import os
import random

import numpy as np
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field

# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------

app = FastAPI(
    title="Campus Evacuation Personal Advisor",
    description=(
        "Three-layer LLM-DRQN evacuation pipeline. "
        "Layer 1: LLM behavior profiling from natural language. "
        "Layer 3: DRQN navigation on OSM walk graph. "
        "Output: personalized plain-language evacuation recommendation."
    ),
    version="1.0.0",
)

# ---------------------------------------------------------------------------
# Global state — loaded once at startup
# ---------------------------------------------------------------------------

_env = None           # EvacEnv instance
_scenario = None      # loaded scenario dict
_checkpoint = None    # path to DRQN .pt file
_ctx = None           # temporary_config context manager (kept active)


@app.on_event("startup")
def _startup():
    global _env, _scenario, _checkpoint, _ctx

    scenario_path = os.environ.get(
        "ADVISOR_SCENARIO", "scenarios/enterprise_blizzard.json"
    )
    severity = os.environ.get("ADVISOR_SEVERITY", "moderate")
    _checkpoint = os.environ.get(
        "ADVISOR_CHECKPOINT",
        "logs/drqn_progressive_severity_v3_extreme/drqn_torch_best.pt",
    )

    from scenario_loader import load_scenario, temporary_config, _apply_disaster_rules
    from evac_env import EvacEnv

    _scenario = load_scenario(scenario_path)
    _scenario["disaster_severity"] = severity
    _scenario = _apply_disaster_rules(_scenario)

    overrides = _scenario.get("config_overrides", {})
    _ctx = temporary_config(overrides)
    _ctx.__enter__()

    _env = EvacEnv()

    print(f"[api] scenario : {_scenario.get('name')}  severity : {severity}")
    print(f"[api] walk nodes: {_env.G_walk.number_of_nodes():,}  "
          f"shelters: {len(_env.shelters)}  "
          f"checkpoint: {_checkpoint}")


@app.on_event("shutdown")
def _shutdown():
    if _ctx is not None:
        try:
            _ctx.__exit__(None, None, None)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Request / Response models
# ---------------------------------------------------------------------------

class AdviseRequest(BaseModel):
    description: str = Field(
        default="",
        description=(
            "Natural language description of the person evacuating. "
            "Leave empty to use default behavior profile."
        ),
        examples=["I am a first-year international student. This is my second week on campus."],
    )
    start_node: int = Field(
        description="OSM node ID of the person's current location on the walk graph.",
        examples=[1638160433],
    )
    severity: str = Field(
        default="moderate",
        description="Disaster severity override for this request.",
        pattern="^(light|moderate|severe|extreme)$",
    )
    api_key: str = Field(
        default="",
        description=(
            "Groq API key for LLM steps. Falls back to GROQ_API_KEY env var. "
            "Omit to use algorithmic defaults."
        ),
    )
    seed: int = Field(default=20260323, description="Random seed for reproducibility.")


class BehaviorProfile(BaseModel):
    walk_speed_multiplier: float
    compliance_rate: float
    panic_level: float
    observation_error_multiplier: float
    decision_delay_steps: int
    shelter_familiarity: float


class RouteSummary(BaseModel):
    recommended_shelter: int | None
    reached: bool
    steps: int
    estimated_minutes: float
    exposure: float
    replan_count: int
    path_length_nodes: int


class AdviseResponse(BaseModel):
    description: str
    profile: BehaviorProfile
    route: RouteSummary
    recommendation: str
    disaster_type: str
    severity: str
    profile_source: str   # "llm" | "default"


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------

@app.get("/health", summary="Liveness check")
def health():
    """Returns 200 if the server is up and the environment is loaded."""
    if _env is None:
        raise HTTPException(status_code=503, detail="Environment not loaded yet")
    return {
        "status": "ok",
        "scenario": _scenario.get("name") if _scenario else None,
        "walk_nodes": _env.G_walk.number_of_nodes(),
        "shelters": len(_env.shelters),
        "checkpoint_exists": os.path.exists(_checkpoint) if _checkpoint else False,
    }


@app.get("/scenarios", summary="List available scenario files")
def list_scenarios():
    """Returns all .json files found under the scenarios/ directory."""
    files = _glob.glob("scenarios/*.json")
    return {"scenarios": sorted(files)}


@app.get("/nodes/random", summary="Sample a random valid walk-graph node")
def random_node(n: int = 1):
    """Returns n random OSM node IDs that are valid start locations."""
    if _env is None:
        raise HTTPException(status_code=503, detail="Environment not loaded")
    nodes = random.sample(list(_env.G_walk.nodes()), min(n, _env.G_walk.number_of_nodes()))
    return {"nodes": [int(nd) for nd in nodes]}


@app.post("/advise", response_model=AdviseResponse, summary="Run full evacuation pipeline")
def advise(req: AdviseRequest):
    """
    Run the three-layer pipeline for a single person:

    1. **Layer 1** — LLM parses `description` into a quantitative behavior profile.
       Falls back to default profile if no API key is provided.
    2. **Layer 3** — DRQN navigates from `start_node` to the best shelter,
       using the behavior profile (familiarity, panic, speed, compliance).
    3. **Output** — LLM generates a personalized plain-language recommendation.
       Falls back to rule-based text if no API key.

    Returns the behavior profile, route summary, and recommendation.
    """
    if _env is None:
        raise HTTPException(status_code=503, detail="Environment not loaded")

    # Validate start_node
    start_node = req.start_node
    if start_node not in _env.G_walk:
        raise HTTPException(
            status_code=422,
            detail=f"Node {start_node} not found in walk graph. "
                   "Use GET /nodes/random to get a valid node ID.",
        )

    # Resolve API key
    api_key = req.api_key or os.environ.get("GROQ_API_KEY", "")

    # Determine effective severity (request overrides server default)
    severity = req.severity
    disaster_type = _scenario.get("disaster_type", "blizzard") if _scenario else "blizzard"

    from personal_advisor import (
        PersonalAdvisor,
        _validate_profile,
        _profile_from_description,
        _generate_recommendation,
        _fallback_recommendation,
        _run_drqn_route,
    )

    # --- Layer 1: behavior profile ---
    profile_source = "default"
    if api_key and req.description.strip():
        try:
            from groq import Groq
            client = Groq(api_key=api_key)
            profile = _validate_profile(_profile_from_description(client, req.description))
            profile_source = "llm"
        except Exception as e:
            profile = _validate_profile({})
    else:
        profile = _validate_profile({})

    # --- Layer 3: DRQN route ---
    if not _checkpoint or not os.path.exists(_checkpoint):
        raise HTTPException(
            status_code=503,
            detail=f"DRQN checkpoint not found: {_checkpoint}. "
                   "Set ADVISOR_CHECKPOINT env var to the correct path.",
        )
    try:
        route = _run_drqn_route(
            env=_env,
            checkpoint_path=_checkpoint,
            start_node=start_node,
            profile=profile,
            seed=req.seed,
            device="auto",
            max_neighbors=None,
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"DRQN route failed: {e}")

    # --- Output: recommendation ---
    if api_key and req.description.strip():
        try:
            recommendation = _generate_recommendation(
                client, req.description, profile, route, disaster_type, severity
            )
        except Exception:
            recommendation = _fallback_recommendation(profile, route, disaster_type, severity)
    else:
        recommendation = _fallback_recommendation(profile, route, disaster_type, severity)

    # Serialize shelter node (may be numpy int)
    shelter_raw = route.get("recommended_shelter")
    shelter_int = int(shelter_raw) if shelter_raw is not None else None

    return AdviseResponse(
        description=req.description,
        profile=BehaviorProfile(**profile),
        route=RouteSummary(
            recommended_shelter=shelter_int,
            reached=bool(route["reached"]),
            steps=int(route["steps"]),
            estimated_minutes=round(route["steps"] * 5 / 60, 1),
            exposure=round(float(route["exposure"]), 4),
            replan_count=int(route["replan_count"]),
            path_length_nodes=int(route["path_length_nodes"]),
        ),
        recommendation=recommendation,
        disaster_type=disaster_type,
        severity=severity,
        profile_source=profile_source,
    )
