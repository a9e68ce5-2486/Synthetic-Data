# CampusResilience Model Spec (Phase 2 Frozen)

## Scope

This document freezes the current model definition implemented in code for reproducible evaluation.

## Environment

- Spatial graph:
  - Walk graph: `env.G_walk`
  - Drive graph: `env.G_drive`
- Backends:
  - OSM campus graph when `EVAC_USE_OSM=True`
  - Fallback grid when `EVAC_USE_OSM=False`
- Shelters:
  - Selected from walk nodes (`EVAC_SHELTER_COUNT`)
  - Car goals are mapped to drive-reachable nodes that correspond to shelters

## Agents

- Pedestrian agents (`PedAgent`, mode=`walk`)
- Car agents (`CarAgent`, mode=`drive`)
- Shuttle agents (`ShuttleAgent`, mode=`drive`, fixed loop route)

Population composition:
- Faculty ratio: `EVAC_FACULTY_RATIO`
- Staff ratio: `EVAC_STAFF_RATIO`

## POMDP Mapping

### State (S)

For each step:
- Agent states: node, edge progress, reached/alive, exposure, role
- Network hazard state:
  - Edge blocked flags
  - Edge snow depth
  - Edge slope
- Shelter set and current occupancy outcome (derived from reached agents)

### Action (A)

Per mobile agent:
- Move toward assigned shelter goal under current policy
- Replan path if blocked (belief-based)
- Shuttle follows route and dwell logic

### Transition (T)

Deterministic + stochastic:
- Agent moves along weighted edges by speed
- Hazard evolves each step:
  - Snow accumulates (`EVAC_SNOW_ACCUM_PER_STEP ± noise`)
  - Edges may become blocked after snow threshold with probability

### Observation (Omega) and Observation Function (O)

Partial local observation within `EVAC_OBS_RADIUS_M`:
- Observed blocked/snow/slope for nearby edges
- Observation error via `EVAC_OBS_ERROR_WALK`, `EVAC_OBS_ERROR_DRIVE`

### Reward Proxy (R)

Current implementation uses evaluation metrics instead of a direct RL reward function:
- Reachability (`reached_rate`, `t90`, `t95`)
- Survival (`alive_rate`)
- Risk burden (`avg_exposure_total`)
- Fairness (`reach_rate_gap` faculty vs staff)

## Policy Set (Frozen Baselines)

- `round_robin`
- `nearest`
- `priority_faculty`

Policy names and behavior are defined in `policy.py`.

## Hazard Rules (Frozen)

- Initial blocked probability: `EVAC_BLOCK_INIT_PROB` (current default `0.0`)
- Snow starts from zero when dynamic snow mode enabled:
  - `EVAC_SNOW_DYNAMIC=True`
  - `EVAC_SNOW_START_ZERO=True`
- Snow-induced blocking:
  - Threshold: `EVAC_BLOCK_FROM_SNOW_THRESHOLD`
  - Probability: `EVAC_BLOCK_FROM_SNOW_PROB`

## Metrics (Run-Level)

From `kpi.py`:
- `alive_rate`
- `reached_rate`
- `avg_exposure_total`
- `t90_step`
- `t95_step`
- `faculty_reached_rate`
- `staff_reached_rate`
- `reach_rate_gap`
- `top_bottlenecks`

## Out of Scope (Current Phase)

- DRQN training/inference
- End-user personalized LLM route advisor
- Dynamic control optimization beyond baseline policy set
