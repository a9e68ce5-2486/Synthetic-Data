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

## 5. Implementation Progress (2026-04-01)

| Step | Task | Status |
|------|------|--------|
| 1 | Design 5 personas, generate behavior parameters via Groq (Llama 3.3 70B) | ✅ Done |
| 2 | `llm_behavior_profiler.py` + `agent_profiles.json` | ✅ Done |
| 3 | `agents/base_agent.py` — add persona fields | ✅ Done |
| 4 | `agents/ped_agent.py` — multiply speed by `speed_multiplier` | ✅ Done |
| 5 | `batch_runner.py` — auto-load profiles, assign persona by role | ✅ Done |
| 6 | 5-persona severity sweep vs uniform agents | ✅ Done |
| 7 | Expand to 20 personas across 4 role categories (realistic campus population) | ✅ Done |
| 8 | 20-persona severity sweep (vs 5-persona and uniform) | 🔄 In Progress |
| 9 | `llm_zone_coordinator.py` (ReAct loop + 4 tools) | ⬜ Planned |
| 10 | LLM coordinator vs DRQN zone recommendation comparison | ⬜ Planned |
| 11 | End-to-end three-layer pipeline integration | ⬜ Planned |

### 5-Persona vs Uniform Comparison Results

| Severity | Uniform | LLM-Profiled (5) | Δ reached |
|----------|---------|-----------------|-----------|
| light    | 0.8482  | 0.8346          | −0.0136   |
| moderate | 0.7918  | 0.7736          | −0.0182   |
| severe   | 0.7364  | 0.7336          | −0.0028   |
| extreme  | 0.7164  | 0.7118          | −0.0046   |

LLM-profiled agents show ~1-2% lower reached_rate than uniform — an **expected and meaningful result**. It quantifies the evacuation disadvantage of vulnerable populations (mobility_impaired, visitor) that the uniform model ignores.

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
