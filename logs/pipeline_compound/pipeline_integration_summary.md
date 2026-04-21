# 3-Layer Pipeline Integration Results

**Scenario**: scenarios/enterprise_compound.json  
**Severity**: moderate  
**Seeds**: [42, 43, 44, 45, 46]  
**Layer 2 LLM used**: True  

## Context

From the full 20-run llm_persona_sweep (blizzard moderate): **reached_rate = 0.724**  
This represents the Layer 1+3 baseline (algo zone, persona-aware DRQN).  

## Results

| Config | Description | Reached Rate | Exposure | Staff RR | Faculty RR |
|--------|-------------|-------------|---------|---------|----------|
| B | Persona-aware DRQN + algo zone (Layer 1+3) | 0.345 | 60.9 | 0.472 | 0.291 |
| C | Persona-aware DRQN + LLM zone (Layer 1+2+3) | 0.378 | 602.0 | 0.648 | 0.350 |

**Layer 2 contribution (C vs B)**: Δreached = +0.033  
