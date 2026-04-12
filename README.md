# Campus Evacuation Planning with LLM Agents and DRQN

A campus evacuation simulation system that combines LLM-based human behavior modeling, LLM-driven zone coordination, and Deep Recurrent Q-Network (DRQN) navigation on a real OSM graph of the University of Utah campus.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 1: LLM Human Behavior Profiler                        │
│  - Input: natural language persona descriptions              │
│  - Output: quantitative behavior parameters per agent        │
│  - Model: Llama 3.3 70B (Groq)                               │
│  - File: llm_behavior_profiler.py                            │
└──────────────────────────┬──────────────────────────────────┘
                           │ agent_profiles.json
┌──────────────────────────▼──────────────────────────────────┐
│  Layer 2: LLM Zone Coordinator Agent                          │
│  - Tool-using LLM (ReAct loop)                               │
│  - Tools: shelter status, zone population, road conditions   │
│  - Output: zone-to-shelter assignment                        │
│  - File: llm_zone_coordinator.py                             │
└──────────────────────────┬──────────────────────────────────┘
                           │ zone assignments
┌──────────────────────────▼──────────────────────────────────┐
│  Layer 3: DRQN Navigation                                     │
│  - Graph-based POMDP on OSM walk/drive network               │
│  - Partial observability (80m radius)                        │
│  - Dueling DQN + GRU, dynamic step budget                    │
│  - File: drqn_minimal.py, batch_runner.py                    │
└─────────────────────────────────────────────────────────────┘
```

| Layer | Method | Responsible for |
|-------|--------|----------------|
| 1 | LLM Behavior Profiling | Who are the evacuees (age, mobility, panic tendency) |
| 2 | LLM Zone Coordinator | Where to go (zone-level shelter assignment with reasoning) |
| 3 | DRQN on OSM graph | How to get there (step-by-step navigation under uncertainty) |

---

## Key Features

### Evacuation Environment
- Real University of Utah campus OSM graph (walk + drive networks)
- Dynamic hazard simulation: blizzard (snow accumulation + road blocking), earthquake (initial road damage), compound
- Disaster severity grading: `light` / `moderate` / `severe` / `extreme`
- Partial observability: each agent observes within 80m radius only
- Shelter capacity management and reassignment

### DRQN Policy
- State: 42-dimensional observation (7 spatial + 5 persona + 30 neighbor features)
- Action: top-k neighbor selection on OSM graph (variable branching factor)
- Training: 4-stage curriculum (light → moderate → severe → extreme), persona randomly sampled each episode from all 20 profiles
- Checkpoint: `logs/drqn_llm_persona/drqn_torch_best.pt`

### LLM Human Behavior Profiling (Layer 1)
20 persona types across 4 role categories (student 60% / staff 20% / faculty 15% / visitor 5%):

| Role | Persona | Speed | Compliance | Panic | Obs. Error | Shelter Familiarity |
|------|---------|-------|-----------|-------|-----------|-------------------|
| student | young_student | 1.20× | 0.60 | 0.40 | 1.50× | 0.40 |
| student | freshman_student | 1.20× | 0.40 | 0.80 | 2.50× | 0.20 |
| student | graduate_student | 1.20× | 0.90 | 0.10 | 0.80× | 0.80 |
| student | international_student | 1.10× | 0.40 | 0.60 | 2.20× | 0.20 |
| student | student_athlete | **1.40×** | 0.60 | 0.10 | 0.80× | 0.80 |
| student | student_with_anxiety | 0.80× | 0.60 | **0.85** | 2.20× | 0.40 |
| student | part_time_student | 0.80× | 0.60 | 0.10 | 1.50× | 0.30 |
| faculty | senior_faculty | 0.60× | 0.95 | 0.10 | 0.80× | 0.90 |
| faculty | junior_faculty | 1.20× | 0.90 | 0.10 | 0.80× | 0.60 |
| faculty | adjunct_instructor | 1.00× | 0.60 | 0.10 | 1.20× | 0.40 |
| staff | staff_admin | 1.10× | 0.95 | 0.05 | 0.80× | 0.95 |
| staff | facilities_staff | 1.20× | 0.95 | 0.05 | 0.80× | 0.95 |
| staff | campus_security | 1.20× | **1.00** | **0.00** | **0.50×** | **1.00** |
| staff | healthcare_staff | 0.80× | 0.95 | 0.10 | 0.80× | 0.95 |
| staff | research_scientist | 1.00× | 0.90 | 0.10 | 1.00× | 0.40 |
| staff | it_staff | 1.00× | 0.90 | 0.10 | 0.80× | 0.80 |
| visitor | visitor | 0.80× | 0.60 | 0.80 | 2.50× | 0.10 |
| visitor | mobility_impaired | **0.30×** | 0.90 | 0.60 | 1.50× | 0.40 |
| visitor | conference_attendee | 1.00× | 0.60 | 0.10 | 1.50× | 0.20 |
| visitor | prospective_student_with_parent | 0.60× | 0.80 | 0.70 | 2.20× | 0.10 |

---

## Results

### DRQN vs Baselines (blizzard, 40 agents, 20 runs)

| Policy | Reached Rate | Notes |
|--------|-------------|-------|
| round_robin | 0.058–0.010 | Collapses under disaster |
| nearest | 0.173–0.031 | Distance-aware but unadaptive |
| DRQN (prev., 37-dim) | 0.845–0.519 | Pre-persona checkpoint |
| **DRQN (persona-aware, 42-dim)** | **0.790–0.545** | LLM-calibrated params, diverse personas |

DRQN achieves **71×** reached_rate vs round_robin at extreme blizzard.

### Blizzard Severity Sweep (persona-aware DRQN, LLM-calibrated params, 200 agents, 20 runs)

| Severity | Reached Rate | Avg Exposure |
|----------|-------------|--------------|
| Light    | 0.787 | 33.8 |
| Moderate | 0.715 | 62.3 |
| Severe   | 0.676 | 84.8 |
| Extreme  | 0.526 | 68.9 |

### Three-Disaster Sweep (200 agents, 20 runs, extreme severity, per-role breakdown)

| Disaster | Reached Rate | Faculty | Staff |
|----------|-------------|---------|-------|
| Blizzard   | 0.526 | 0.589 | 0.566 |
| Earthquake | 0.466 | 0.556 | **0.764** |
| Compound   | 0.424 | 0.537 | **0.698** |

Earthquake produces large staff/visitor asymmetry: staff navigate effectively even with 80% initial blockage; visitors fail catastrophically due to low familiarity and high observation error. Results are highly consistent with 40-agent runs (Δ ≤ 0.02 across all conditions).

### Per-Persona Fairness (200 agents, cross-disaster)

Most vulnerable at blizzard extreme: adjunct_instructor (0.279), international_student (0.372), conference_attendee (0.389).  
Fairness gap: **0.397** at blizzard extreme, **0.938** at earthquake extreme (campus_security: 1.000 vs conference_attendee: 0.062), **0.894** at compound extreme (campus_security: 0.889 vs mobility_impaired: 0.000).

### Three-Layer Pipeline Integration (blizzard moderate, 5 seeds)

| Config | Description | Reached Rate | Avg Exposure |
|--------|-------------|-------------|--------------|
| B | Algo zone + persona DRQN (Layer 1+3) | 0.713 | 71.6 |
| C | LLM zone + persona DRQN (Layer 1+2+3) | 0.691 | 262.0 |

Per-role breakdown reveals the key difference:

| Role | Config B (algo zone) | Config C (LLM zone) |
|------|---------------------|---------------------|
| Staff   | 0.769 | **0.983** |
| Faculty | 0.733 | **0.950** |
| Student | 0.000 | **0.950** |

Layer 2 (LLM zone coordinator) trades 2.2% overall throughput for a dramatic fairness improvement: the algorithmic assignment leaves students completely unserved (0.000), while the LLM balances all roles above 95%. The higher exposure reflects the LLM routing agents to further but less congested shelters. Layer 2's value is **role-level equity**, not raw throughput.

---

## Project Structure

```
.
├── run.sh                       # Shortcut commands (demo / ui / advise / map / api)
│
├── Core simulation
│   ├── evac_env.py              # OSM graph, hazards, shelters
│   ├── config.py                # Global parameters
│   ├── scenario_loader.py       # Scenario JSON + LLM severity presets
│   ├── batch_runner.py          # Multi-agent batch simulation + DRQN inference
│   ├── drqn_minimal.py          # DRQN training: GRU + Dueling DQN, curriculum
│   ├── policy.py                # Baseline policies: round_robin, nearest
│   ├── kpi.py                   # KPI: reached_rate, t90/t95, fairness
│   ├── shelter.py               # Shelter capacity management
│   └── zone_assignment.py       # Zone clustering + shelter assignment
│
├── agents/
│   ├── base_agent.py            # Base agent: movement, belief, persona fields
│   ├── ped_agent.py             # Pedestrian agent
│   ├── car_agent.py             # Car agent
│   └── shuttle_agent.py         # Campus shuttle agent
│
├── Three-layer pipeline
│   ├── llm_behavior_profiler.py  # Layer 1: persona description → behavior params
│   ├── agent_profiles.json       # Generated profiles for 20 personas
│   ├── llm_zone_coordinator.py   # Layer 2: ReAct LLM zone coordinator
│   ├── llm_scenario_generator.py # LLM disaster parameter calibration
│   ├── personal_advisor.py       # NL input → profile → DRQN route → NL output
│   ├── advisor_api.py            # FastAPI REST service (POST /advise)
│   └── advisor_ui.py             # Gradio web interface (browser UI)
│
├── route_recommendation.py      # Single-agent route rollout
├── visualize_map.py             # folium maps (route + simulation modes)
├── management_report.py         # Management summary generation
│
├── scenarios/
│   ├── enterprise_blizzard.json
│   ├── enterprise_earthquake.json
│   ├── enterprise_compound.json
│   ├── enterprise_baseline.json
│   ├── llm_severity_presets.json        # LLM-calibrated disaster parameters
│   ├── llm_severity_presets_reasoning.json
│   ├── profiles/                        # Disaster type presets
│   └── archive/                         # Scaling/capacity experiment scenarios
│
├── scripts/
│   ├── demo_pipeline.py              # CLI three-layer pipeline demo
│   ├── eval_zone_coordinator.py      # Layer 2: LLM vs algorithmic comparison
│   ├── eval_pipeline_integration.py  # End-to-end three-layer experiment
│   ├── analyze_persona_fairness.py   # Per-persona fairness analysis
│   ├── run_disaster_severity_sweep.py
│   └── *.sh                          # Training / fine-tuning shell scripts
│
├── logs/                        # Experiment outputs (checkpoints, CSVs, JSONs)
│   └── drqn_llm_persona/        # Final trained model
│       └── drqn_torch_best.pt
│
├── docs/                        # Reference documents
│   └── LLM_AGENT_EXTENSION_ZH.md   # Literature review and paper narrative
│
└── paper/
    └── capstone_final.tex       # Full LaTeX paper
```

---

## Setup

```bash
# 1. Create virtualenv and install dependencies
python3 -m venv venv
venv/bin/pip install torch networkx osmnx shapely pandas numpy groq fastapi uvicorn folium pyproj gradio

# 2. Set Groq API key
echo "GROQ_API_KEY=gsk_your_key_here" > .env
```

```bash
# Run commands (auto-loads .env)
./run.sh demo [blizzard|earthquake|compound]  # Animated simulation (matplotlib)
./run.sh ui                                   # Gradio web UI → http://localhost:7860
./run.sh advise                               # Personal Advisor CLI (interactive)
./run.sh map                                  # Static route map (opens in browser)
./run.sh api                                  # REST API → http://localhost:8000/docs
```

---

## Related Papers

| Paper | Venue | Relevance |
|-------|-------|-----------|
| Dang et al. — LLM Agents for Fire Evacuation | Safety Science 2025 | Direct predecessor: LLM + cellular automata evacuation |
| Chen et al. — FLARE Wildfire Evacuation | ACL 2025 | LLM + RL hybrid evacuation decision, +20.47% over baselines |
| Li et al. — LLM Simulations for Emergency Policy | arXiv 2025 | Campus-scale (13k agents) LLM emergency simulation |
| Park et al. — Generative Agents | UIST 2023 | Memory + reflection persona architecture |
| Ahn et al. — SayCan | CoRL 2022 | LLM planner + RL executor architecture |
| Yao et al. — ReAct | ICLR 2023 | Tool-using LLM agent loop |

Full literature review: [docs/LLM_AGENT_EXTENSION_ZH.md](docs/LLM_AGENT_EXTENSION_ZH.md)
