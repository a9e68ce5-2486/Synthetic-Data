# 3-Layer Pipeline Integration Results

**Scenario**: scenarios/enterprise_blizzard.json  
**Severity**: moderate  
**Seeds**: [42, 43, 44, 45, 46]  
**Layer 2 LLM used**: True  

## Context

From the full 20-run llm_persona_sweep (blizzard moderate): **reached_rate = 0.724**  
This represents the Layer 1+3 baseline (algo zone, persona-aware DRQN).  

## Results

| Config | Description | Reached Rate | Exposure | Staff RR | Faculty RR |
|--------|-------------|-------------|---------|---------|----------|
| B | Persona-aware DRQN + algo zone (Layer 1+3) | 0.713 | 71.6 | 0.769 | 0.733 |
| C | Persona-aware DRQN + LLM zone (Layer 1+2+3) | 0.691 | 262.0 | 0.983 | 0.950 |

**Layer 2 contribution (C vs B)**: Δreached = -0.022  
