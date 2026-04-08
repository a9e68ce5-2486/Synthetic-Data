# LLM Agent Extension: Literature Review and Implementation Plan

> Document created: 2026-03-31

---

## 1. Why LLM Agents Are Needed

Three limitations in the current system:

1. **All agents have identical behavior parameters**: Speed, observation error, and decision logic are uniform across all agents, which fails to capture real population heterogeneity (elderly, mobility-impaired, students, and faculty respond very differently).
2. **Zone assignment is purely algorithmic**: No semantic reasoning capability — it cannot explain *why* a particular zone is assigned to a particular shelter.

**Solution**: A three-layer architecture

```
Layer 1 (LLM):  Human Behavior Profiling   → generates diverse agent profiles
Layer 2 (LLM):  Zone Coordinator Agent     → tool-using LLM for zone-level decisions
Layer 3 (DRQN): Navigation                 → per-agent step-by-step graph routing
```

---

## 2. Related Work

### Category A: LLM Agents for Evacuation Simulation

#### 1. LLM Agents for Fire Evacuation Simulation

- **Authors**: Pei Dang, Jun Zhu, Weilian Li, Yakun Xie, Heng Zhang
- **Journal**: *Safety Science*, Volume 191, Elsevier, 2025
- **DOI**: 10.1016/j.ssci.2025.001602

**Core Contribution**:

Integrates GPT-4, ERNIE-Bot 4.0, Llama-2-70B, and ChatGLM2-6B with Cellular Automata to simulate fire evacuation in a LiDAR-reconstructed shopping mall. Each agent has personalized memory, cognition, and LLM-driven decisions.

Key findings:
- Larger LLMs produce more consistent and efficient evacuation strategies
- Agent background (persona) and communication behavior significantly affect group evacuation outcomes
- Behavioral differences across LLMs are quantifiable

**Relevance to this project**: This is the most direct predecessor for "LLM-based human behavior modeling." It directly justifies the Layer 1 design.

---

#### 2. FLARE: Wildfire Evacuation Decision Prediction with Behavioral Theory-Informed LLMs

- **Authors**: Ruxiao Chen, Chenguang Wang, Yuran Sun, Xilei Zhao, Susu Xu
- **Venue**: ACL 2025 (Long Paper)
- **arXiv**: 2502.17701

**Core Contribution**:

Combines LLM + behavioral theory (Protection Motivation Theory) + Chain-of-Thought + memory-based RL to predict individual wildfire evacuation decisions.

Key findings:
- Average improvement of **+20.47%** over traditional theory-driven behavioral models
- Strong cross-event generalization (transfers across different wildfire events)
- Behavioral theory + LLM combination outperforms LLM alone

**Relevance**: Closest architectural parallel. LLM + RL hybrid for evacuation decision prediction mirrors the division of labor between LLM behavior profiling (Layer 1) and DRQN navigation (Layer 3) in this project.

---

#### 3. What Makes LLM Agent Simulations Useful for Policy Practice? An Iterative Design Study in Emergency Preparedness

- **Authors**: Yuxuan Li, Sauvik Das, Hirokazu Shirado
- **Year**: 2025
- **arXiv**: 2509.21868

**Core Contribution**:

A year-long collaboration with a university emergency response team, scaling LLM agent simulation to **13,000 agents** for campus emergency evacuation.

Three design principles:
1. Build user trust through verifiable scenarios
2. Use preliminary simulation results to surface implicit domain knowledge
3. Treat simulation refinement and policy implementation as mutually reinforcing processes

**Relevance**: Closest application scenario (university campus + emergency evacuation + policy tool). The 13k-agent scale validates LLM simulation feasibility; the three design principles can directly frame the system design narrative in the final paper.

---

### Category B: Foundational LLM Social Simulation Architectures

#### 4. Generative Agents: Interactive Simulacra of Human Behavior

- **Authors**: Joon Sung Park, Joseph O'Brien, Carrie Jun Cai, Meredith Ringel Morris, Percy Liang, Michael S. Bernstein
- **Venue**: ACM UIST 2023 (Best Paper Award)

**Core Contribution**:

Proposes the generative agent architecture:
- **Memory stream**: natural language episodic memory
- **Reflection**: periodic synthesis of higher-level insights
- **Retrieval**: dynamic recall of relevant memories to support planning

25 agents in a Sims-like environment autonomously spread party invitations, form social relationships, and coordinate activities.

**Relevance**: Theoretical foundation for Layer 1 LLM behavior profiling. The memory-reflect-retrieve-plan loop can model how evacuees update decisions in response to dynamic information (shelter full, road blocked).

---

#### 5. Generative Agent Simulations of 1,000 People

- **Authors**: Joon Sung Park et al. (Stanford HAI)
- **Year**: 2024
- **arXiv**: 2411.10109

**Core Contribution**:

Interviews 1,052 real individuals to build digital twins. LLM agents replicate real survey answers with **85% accuracy** — matching human recall accuracy two weeks later. Accuracy gaps across racial and ideological groups are smaller than simple demographic-conditioning methods.

**Relevance**: Validates the feasibility of using LLM personas to represent real population diversity, directly supporting the use of LLM-generated profiles for faculty / student / mobility-impaired personas.

---

#### 6. AgentSociety: Large-Scale Simulation of LLM-Driven Generative Agents

- **Authors**: Jinghua Piao et al. (Tsinghua University)
- **Year**: 2025
- **arXiv**: 2502.08691

**Core Contribution**:

City-scale simulation with **10,000+ LLM agents** featuring granular emotions, needs, motivations, and cognitive states. Includes a **hurricane impact experiment** that simulates external disaster effects on social behavior, with results aligned to real-world data.

**Relevance**: The hurricane impact experiment directly mirrors disaster evacuation scenarios, validating large-scale LLM agents in disaster settings.

---

#### 7. Large Language Models Empowered Agent-Based Modeling and Simulation: A Survey

- **Authors**: Chen Gao, Xiaochong Lan, Nian Li et al. (Tsinghua)
- **Journal**: *Humanities and Social Sciences Communications* (Nature Portfolio, 2024)
- **DOI**: 10.1038/s41599-024-03611-3

**Core Contribution**:

Systematic survey integrating traditional ABM with LLM agents, covering:
- Persona design methodologies
- Memory module architectures
- Planning module architectures
- Agent social coordination strategies

**Relevance**: Bridges traditional evacuation ABM and LLM agent design, providing a systematic methodological foundation for Layer 1 behavior profiling.

---

### Category C: LLM + RL Hybrid Architectures

#### 8. SayCan: Do As I Can, Not As I Say

- **Authors**: Michael Ahn et al. (Google Robotics, 42 authors)
- **Venue**: CoRL 2022
- **arXiv**: 2204.01691

**Core Contribution**:

Seminal "LLM as planner, RL as executor" paper. LLM provides high-level semantic plans; a pre-trained RL value function (affordance) evaluates which actions are feasible in the current state. Achieves 70% success on long-horizon household tasks.

**Relevance**: Theoretical foundation for the Layer 2 (LLM zone coordinator) + Layer 3 (DRQN navigation) layered architecture. LLM decides *where to go* (shelter), DRQN decides *how to get there*.

---

#### 9. Plan-Seq-Learn: Language Model Guided RL for Long Horizon Tasks

- **Authors**: Murtaza Dalal, Tarun Chiruvolu, Devendra Chaplot, Ruslan Salakhutdinov
- **Venue**: ICLR 2024

**Core Contribution**:

LLM generates high-level subgoal sequences → motion planner executes transitions between subgoals → RL learns low-level control online. No predefined skill library required. Achieves 85%+ success across 25+ tasks.

**Relevance**: More flexible than SayCan. LLM generates zone-level waypoints / shelter sequences; DRQN handles per-step graph decisions without requiring predefined skills.

---

#### 10. ReAct: Synergizing Reasoning and Acting in Language Models

- **Authors**: Shunyu Yao, Jeffrey Zhao, Dian Yu et al.
- **Venue**: ICLR 2023
- **arXiv**: 2210.03629

**Core Contribution**:

Standard agent loop where LLM simultaneously produces reasoning traces and tool calls. Outperforms pure RL and pure imitation learning on HotPotQA, ALFWorld, WebShop, and other benchmarks using only 1-2 in-context examples.

**Relevance**: Implementation template for the Layer 2 LLM zone coordinator. Reasoning → query tools (shelter status, road conditions) → output assignment decisions is exactly a ReAct loop.

---

#### 11. Inner Monologue: Embodied Reasoning through Planning with Language Models

- **Authors**: Wenlong Huang et al. (Google)
- **Venue**: CoRL 2022
- **arXiv**: 2207.05608

**Core Contribution**:

LLM receives natural language environment feedback (success detectors, scene descriptions, human corrections) to form a "closed-loop inner monologue" and dynamically replan. No fine-tuning required.

**Relevance**: Zone coordinator receiving real-time simulation state updates (congestion, shelter capacity, road closures) and dynamically revising zone assignments is exactly this paper's evacuation scenario analogue.

---

### Category D: Synthetic Population and Persona Modeling

#### 12. Large Language Models as Urban Residents: An LLM Agent Framework for Personal Mobility Generation

- **Authors**: Jiawei Wang, Renhe Jiang, Chuang Yang et al.
- **Venue**: NeurIPS 2024
- **arXiv**: 2402.14744

**Core Contribution**:

First framework using LLM to generate synthetic populations with demographic attributes (age, income, occupation) and simulate personalized daily mobility patterns. Uses self-consistency calibration + RAG to align with real travel survey data.

**Relevance**: Concrete methodology for Layer 1 synthetic population generation. The pipeline of demographic attributes (age, role, mobility) → LLM → behavior parameters (speed, compliance, panic level) directly maps to this project.

---

### Category E: LLM Agent Evaluation and Disaster Application Surveys

#### 13. Harnessing Large Language Models for Disaster Management: A Survey

- **Authors**: Zhenyu Lei et al.
- **Year**: 2025
- **arXiv**: 2501.06932

**Core Contribution**:

First systematic survey of LLM applications across the complete natural disaster management lifecycle (pre-disaster, response, recovery), analyzing 70+ studies (2020-2024).

**Relevance**: Establishes the related work context, providing a full picture of where LLM applications already exist in disaster management — useful for positioning this system in the literature gap.

---

#### 14. AgentBench: Evaluating LLMs as Agents

- **Authors**: Xiao Liu et al. (Tsinghua / THUDM)
- **Venue**: ICLR 2024
- **arXiv**: 2308.03688

**Core Contribution**:

LLM agent benchmark across 8 environments (OS, database, knowledge graph, games, web), evaluating 29 LLMs. Key bottlenecks identified: long-horizon reasoning and instruction-following.

**Relevance**: Provides evaluation methodology for assessing the LLM zone coordinator's performance.

---

#### 15. From Individual to Society: A Survey on Social Simulation Driven by LLM-based Agents

- **Authors**: Xinyi Mou et al. (Fudan)
- **Year**: 2024
- **arXiv**: 2412.03563

**Core Contribution**:

Most comprehensive LLM social simulation survey with a three-tier taxonomy: Individual Simulation, Scenario Simulation, and Society Simulation. Evacuation falls under "Scenario Simulation" (multi-agent cooperation in a defined situation).

---

## 3. Implementation Plan

### Layer 1: LLM Human Behavior Profiler

**Goal**: Use LLM to translate persona descriptions into quantitative behavior parameters, introducing realistic population heterogeneity into the simulation.

**Persona Design (5 types)**:

| Persona | Description | Expected Behavior |
|---------|-------------|------------------|
| senior_faculty | Professor 60+, limited mobility, calm under pressure | Speed↓, compliance↑, panic↓ |
| young_student | Student ~20, highly mobile, moderate panic tendency | Speed↑, compliance↓, panic↑ |
| staff_admin | Administrative staff, familiar with campus, evacuation-trained | Speed≈, compliance↑↑, knows shelter locations |
| mobility_impaired | Wheelchair user or visually impaired, reliant on assistance | Speed↓↓, requires accessible routes |
| visitor | External visitor, unfamiliar with campus | Obs. error↑, panic↑↑, no shelter knowledge |

**LLM Output Format**:

```json
{
  "persona": "senior_faculty",
  "walk_speed_multiplier": 0.65,
  "compliance_rate": 0.90,
  "panic_level": 0.10,
  "observation_error_multiplier": 1.30,
  "decision_delay_steps": 2,
  "shelter_familiarity": 0.85
}
```

**Implementation file**: `llm_behavior_profiler.py`

---

### Layer 2: LLM Zone Coordinator Agent

**Goal**: Use a tool-using LLM (ReAct loop) for zone-level shelter assignment, directly comparable to the existing DRQN-based zone recommendation.

**Available tools**:

```python
get_shelter_status()     # remaining capacity and current occupancy per shelter
get_zone_population()    # headcount and persona distribution per zone
get_road_conditions()    # road blockage status and severity level
get_distance_matrix()    # distances from zone centroids to each shelter
assign_zone_to_shelter() # output assignment decision
```

**Implementation file**: `llm_zone_coordinator.py`

**Comparison experiment**: LLM coordinator vs DRQN-based zone recommendation — metrics: reached_rate, shelter capacity utilization, number of weak backup zones.

---

### Layer 3: DRQN Navigation (existing)

Retains the existing DRQN system. Receives Layer 1 persona parameters as agent initialization inputs and Layer 2 zone assignments as target shelter settings.

---

## 4. Narrative Contribution to the Paper

**Suggested title framing**:

> A Three-Layer LLM-DRQN Architecture for Campus Evacuation Planning: LLM-Driven Human Behavior Modeling, Zone Coordination, and Graph-Based Navigation

**Contribution breakdown by layer**:

| Layer | Method | Problem Addressed |
|-------|--------|-------------------|
| Layer 1 | LLM Behavior Profiling | Population heterogeneity (age, ability, panic tendency) |
| Layer 2 | LLM Zone Coordinator (ReAct) | Semantics-driven zone-level shelter assignment |
| Layer 3 | DRQN on OSM graph | Per-agent step-by-step navigation under partial observability |

**Positioning relative to prior work**:

- vs. Dang et al. (2025): same use of LLM for evacuation behavior modeling, but adds graph-based DRQN navigation and zone-level coordination
- vs. FLARE (Chen et al., 2025): same LLM + RL hybrid, but campus-specific with multi-zone coordination
- vs. Li et al. (2025): same campus emergency target, but adds LLM-profiled diverse agents + DRQN routing

---

## 5. Implementation Progress (2026-04-02)

| Step | Task | Status |
|------|------|--------|
| 1 | Design 5 personas, generate behavior parameters via Groq (Llama 3.3 70B) | ✅ Done |
| 2 | `llm_behavior_profiler.py` + `agent_profiles.json` | ✅ Done |
| 3 | `agents/base_agent.py` — add persona fields | ✅ Done |
| 4 | `agents/ped_agent.py` — multiply speed by `speed_multiplier` | ✅ Done |
| 5 | `batch_runner.py` — auto-load profiles, assign persona by role | ✅ Done |
| 6 | 5-persona severity sweep vs uniform agents | ✅ Done |
| 7 | Expand to 20 personas across 4 role categories (realistic campus population) | ✅ Done |
| 8 | Fix: wire obs_error / familiarity / compliance / delay into simulation | ✅ Done |
| 9 | 20-persona v2 severity sweep + all-policies full comparison | ✅ Done |
| 10 | Wire `panic_level` into simulation (amplifies obs_error, reduces compliance) | ✅ Done |
| 11 | `llm_zone_coordinator.py` — ReAct loop + 4 tools + algorithmic fallback | ✅ Done |
| 12 | `personal_advisor.py` — NL description → LLM profile → DRQN → NL recommendation | ✅ Done |
| 13 | Raise `EVAC_PED_COUNT` to 100 for per-persona statistical analysis | ✅ Done |
| 14 | 20-persona v3 sweep (100 agents, panic wired) | ✅ Done |
| 15 | Per-persona fairness analysis + `analyze_persona_fairness.py` | ✅ Done |
| 16 | Map visualization `visualize_map.py` (folium, route + simulation modes) | ✅ Done |
| 17 | Earthquake / Compound persona sweep | ✅ Done |
| 18 | End-to-end demo script (`demo_pipeline.py`) | ✅ Done |
| 19 | Layer 2 evaluation: LLM coordinator vs algorithmic (`eval_zone_coordinator.py`) | ✅ Done |
| 20 | DRQN obs vector with persona fields (retrain) | ⬜ Planned |
| 21 | Personal Advisor API (FastAPI endpoint, `advisor_api.py`) | ✅ Done |
| 22 | End-to-end three-layer pipeline integration | ⬜ Planned |
| 23 | Scale to 200 agents (student×120, faculty×30, staff×40, visitor×10) | ✅ Done |
| 24 | LLM scenario generator (`llm_scenario_generator.py`) — NL description → disaster params | ✅ Done |
| 25 | 200-agent × original params three-disaster sweep | ✅ Done |
| 26 | 200-agent × LLM-generated params three-disaster sweep | ⬜ Planned |

### LLM Scenario Generator (Step 24, 2026-04-06)

**Script**: `llm_scenario_generator.py`

**Design**: The LLM receives only physical descriptions of each parameter — no numeric ranges are given in the prompt. The LLM uses its own knowledge of real-world disasters to decide what values are physically reasonable. A safety clamp is applied post-generation as a hard guardrail only.

**Run**:
```bash
python3 llm_scenario_generator.py --api-key $GROQ_API_KEY --verbose --compare
```

**Outputs**:
- `scenarios/llm_severity_presets.json` — parameters (auto-loaded by `scenario_loader.py`)
- `scenarios/llm_severity_presets_reasoning.json` — LLM reasoning per scenario (citable in paper)

**Key differences vs hand-coded values:**

| Parameter | Original (blizzard extreme) | LLM | Interpretation |
|-----------|----------------------------|-----|----------------|
| `EVAC_OBS_ERROR_WALK` | 0.25 | **0.50** | LLM judges whiteout visibility as more severe |
| `EVAC_BLOCK_INIT_PROB` (earthquake extreme) | 0.55 | **0.80** | LLM judges M8.5 initial structural collapse as worse |
| `EVAC_BLOCK_PROB` (blizzard light) | 0.05 | 0.03 | LLM judges light blizzard road closure rate as lower |
| `EVAC_SNOW_MIN` (extreme) | 0.50 | 0.20 | LLM judges initial snow cover not necessarily maximum |

**Integration**: `scenario_loader.py` detects `scenarios/llm_severity_presets.json` at import time; if found, LLM values take priority over hard-coded tables. Falls back to hard-coded if file absent.

---

### Layer 2 Evaluation Results (2026-04-02)

**Script**: `eval_zone_coordinator.py`  
**Setup**: enterprise_blizzard × moderate × seeds 42–46 × num_zones=6  

**Metrics** (6 dimensions):

| Metric | Description | Better |
|--------|-------------|--------|
| avg_primary_distance_m | Mean graph distance from zone members to assigned shelter | lower |
| load_balance_std | Std of demand per shelter | lower |
| shelter_diversity | Number of distinct shelters used | higher |
| backup_coverage | Fraction of zones with a valid backup shelter | higher |
| invalid_assignments | Fraction of hallucinated shelter IDs | lower |
| reasoning_quality | Fraction of zones with meaningful LLM reasoning | higher |

**Offline mode results (no Groq API key — algorithmic fallback for both):**

| Metric | Algorithmic | LLM Coordinator | Winner |
|--------|-------------|-----------------|--------|
| Avg distance to shelter (m) | 1105.9 ±167.2 | 1105.9 ±167.2 | tie |
| Load balance std | 5.3 ±1.4 | 5.3 ±1.4 | tie |
| Distinct shelters used | 3.6 ±0.5 | 3.6 ±0.5 | tie |
| Backup shelter coverage | 1.0 ±0.0 | 1.0 ±0.0 | tie |
| Invalid assignments | 0.0 ±0.0 | 0.0 ±0.0 | tie |
| Reasoning quality | 1.0 ±0.0 | 0.0 ±0.0 | Algo |

> **Note**: In offline mode, LLM coordinator falls back to algorithmic 100% of the time (no API key), so the first five metrics are identical. Reasoning quality shows 1.0 for algorithmic because the fallback fills in `"algorithmic"` as reasoning text.  
> **To run with LLM**: `python eval_zone_coordinator.py --scenario scenarios/enterprise_blizzard.json --api-key $GROQ_API_KEY --seeds 42 43 44 45 46`  
> Expected LLM behavior: higher shelter_diversity (LLM tends to distribute load), lower load_balance_std (capacity-aware reasoning), reasoning_quality ≥ 0.8.

**Output**: `logs/zone_eval/zone_eval_report.md`, `logs/zone_eval/zone_eval_results.json`

---

### Per-Persona Fairness Analysis Results (2026-04-03)

**Setup**: enterprise_blizzard × 4 severities × 20 runs × DRQN (100 agents, 20 personas, panic wired)

#### Role-level reached_rate

| Severity | Overall | student | faculty | staff | visitor |
|----------|---------|---------|---------|-------|---------|
| light    | 0.770 | 0.756 | 0.788 | 0.839 | 0.647 |
| moderate | 0.708 | 0.676 | 0.719 | 0.846 | 0.551 |
| severe   | 0.615 | 0.584 | 0.673 | 0.701 | 0.429 |
| extreme  | 0.593 | 0.557 | 0.683 | 0.715 | 0.280 |

#### Key findings

- **Fairness gap (extreme)**: campus_security (0.783) vs conference_attendee (0.062) → **gap = 0.721**
- **Role gap (extreme)**: staff (0.715) vs visitor (0.280) → gap = 0.435
- `conference_attendee`: collapses to 6.2% under extreme — familiarity=0.20, effective obs_error=1.65×
- `mobility_impaired`: 0.833 under light → 0.214 under extreme — speed bottleneck amplified by severe conditions
- `junior_faculty`: only persona stable across all severities (+0.003 light→extreme)
- **These results directly motivate the Personal Advisor system**

### Personal Advisor Design Decision (2026-04-03)

**Route output: static snapshot (Direction A)**

`extract_route()` runs one full simulation. The output `path_nodes` already incorporates all road blockages and DRQN replanning:

```
Agent encounters blocked road during simulation → DRQN replans → continues → final path_nodes
```

- Output route = final path after all blockage detours; `replan_count` records mid-route replans
- LLM output includes alternative route guidance whenever `replan_count > 0` or `severity = severe/extreme`
- Guidance detail scales with `shelter_familiarity` (high → brief; low/none → full step-by-step)
- **Blocked-road alternative is always included for all personas**, regardless of familiarity level

**Future Work — Dynamic updates (Direction B):**

User transmits current GPS position continuously; system re-calls `advise()` and updates route in real time. Requires a mobile frontend that maps GPS coordinates to OSM node IDs.

### Full Comparison Results (2026-04-02)

**Key finding: v1 (fields not wired) showed only 1-2% gap — statistical noise, not real heterogeneity. After wiring all 4 fields in v2, the gap grows to 5-12%, representing genuine persona effects.**

| Severity | Uniform | 5-persona v1 | 20-persona v2 | v2 vs Uniform |
|----------|---------|-------------|--------------|--------------|
| light    | 0.848   | 0.835       | **0.796**    | −0.052 |
| moderate | 0.792   | 0.774       | **0.708**    | −0.084 |
| severe   | 0.736   | 0.734       | **0.623**    | −0.113 |
| extreme  | 0.716   | 0.712       | **0.597**    | −0.119 |

**Baseline policies on blizzard severity sweep:**

| Severity | round_robin | nearest | DRQN (uniform) |
|----------|------------|---------|----------------|
| light    | 0.058 | 0.173 | 0.835 |
| moderate | 0.022 | 0.100 | 0.774 |
| severe   | 0.012 | 0.061 | 0.734 |
| extreme  | 0.010 | 0.031 | 0.712 |

Baselines collapse under blizzard because they use distance-based movement (max 840m in 600 steps) while campus walk distances reach p90=1765m. DRQN's edge-hop model is not subject to this constraint.

### Expanded to 20 Personas (Llama 3.3 70B via Groq)

**4 role categories reflecting University of Utah population:**

| Role | Weight | Personas |
|------|--------|---------|
| student | 60% | young_student, freshman_student, graduate_student, international_student, student_athlete, student_with_anxiety, part_time_student |
| faculty | 15% | senior_faculty, junior_faculty, adjunct_instructor |
| staff | 20% | staff_admin, facilities_staff, campus_security, healthcare_staff, research_scientist, it_staff |
| visitor | 5% | visitor, mobility_impaired, conference_attendee, prospective_student_with_parent |

**Generated parameter highlights:**

```
Persona                              speed  comply  panic  obs_err  delay  famil
campus_security                       1.20    1.00   0.00     0.50      0   1.00  ← best performer
student_athlete                       1.40    0.60   0.10     0.80      0   0.80
student_with_anxiety                  0.80    0.60   0.85     2.20      3   0.40  ← highest panic
freshman_student                      1.20    0.40   0.80     2.50      3   0.20  ← lowest compliance
mobility_impaired                     0.30    0.90   0.60     1.50      2   0.40  ← slowest
visitor/prospective_*              0.6–0.8  0.60–0.80  0.70–0.80  2.20–2.50  3  0.10  ← unfamiliar
```

### Persona Assignment Strategy (`batch_runner.py`)

```
student → young_student (30%), freshman_student (20%), graduate_student (20%),
          international_student (10%), student_with_anxiety (8%), student_athlete (7%),
          part_time_student (5%)
faculty → senior_faculty (45%), junior_faculty (35%), adjunct_instructor (20%)
staff   → staff_admin (30%), facilities_staff (20%), healthcare_staff (15%),
          research_scientist (15%), campus_security (10%), it_staff (10%)
visitor → visitor (35%), conference_attendee (30%), prospective_student_with_parent (20%),
          mobility_impaired (15%)
```

### Modified Files

| File | Changes |
|------|---------|
| `llm_behavior_profiler.py` | PERSONAS expanded from 5 to 20 across 4 role categories |
| `agent_profiles.json` | Regenerated: all 20 persona behavior parameters via Llama 3.3 70B |
| `agents/base_agent.py` | Added persona, speed_multiplier, and 5 other fields (backward-compatible defaults) |
| `agents/ped_agent.py` | `step()` now uses `EVAC_SPEED_WALK * self.speed_multiplier` |
| `config.py` | Added `EVAC_ROLE_WEIGHTS` dict, replacing binary faculty/staff split |
| `batch_runner.py` | `_PERSONA_WEIGHTS` updated for 4 roles; `_build_agents()` uses role-weighted sampling |
