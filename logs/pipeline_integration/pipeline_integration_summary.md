# 3-Layer Pipeline Integration Results

**Scenario**: scenarios/enterprise_blizzard.json  
**Severity**: moderate  
**Seeds**: [42, 43, 44, 45, 46]  
**Layer 2 LLM used**: False  

## Context

From the full 20-run llm_persona_sweep (blizzard moderate): **reached_rate = 0.724**  
This represents the Layer 1+3 baseline (algo zone, persona-aware DRQN).  

## Results

| Config | Description | Reached Rate | Exposure | Staff RR | Faculty RR |
|--------|-------------|-------------|---------|---------|----------|
| B | Persona-aware DRQN + algo zone (Layer 1+3) | 0.713 | 82.2 | 0.756 | 0.747 |
| C | Persona-aware DRQN + algo zone (Layer 1+2+3) | 0.702 | 204.9 | 0.967 | 0.950 |

**Layer 2 contribution (C vs B)**: Δreached = -0.011  

> ⚠ Offline run: no API key — Layer 2 used algorithmic fallback (C ≈ B).  
> To quantify true Layer 2 contribution, rerun with `--api-key $GROQ_API_KEY`.  
