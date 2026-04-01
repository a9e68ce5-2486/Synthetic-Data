# Scenario Data Layer

This folder separates **scenario definition** from code-level config editing.

## Structure

- `enterprise_*.json`: runnable scenario files for experiments.
- `profiles/*.json`: reusable disaster profiles (hazard/observability assumptions).

## How merging works

1. Load scenario file.
2. If `profile` is set, load `profiles/<profile>.json`.
3. Apply profile `config_overrides`.
4. If `disaster_severity` is set, apply the severity preset on top of the profile hazard defaults.
5. Apply scenario `config_overrides` on top last (scenario wins on conflicts).

## Minimal scenario fields

- `name`
- `num_runs`
- `base_seed`
- `policies`
- `config_overrides`

## Disaster intensity grading

Scenarios may optionally define:

- `disaster_type`
- `disaster_severity`

Supported severity levels:

- `light`
- `moderate`
- `severe`
- `extreme`

Currently intensity grading is supported for:

- `blizzard`
- `earthquake`
- `compound`

When `disaster_severity` is provided, `scenario_loader.py` applies:

- profile hazard defaults
- then severity overrides
- then scenario-specific `config_overrides`

Example:

```json
{
  "name": "enterprise_blizzard_moderate",
  "disaster_type": "blizzard",
  "disaster_severity": "moderate",
  "profile": "blizzard",
  "num_runs": 20,
  "base_seed": 20260227,
  "policies": ["round_robin", "nearest"],
  "config_overrides": {
    "EVAC_PED_COUNT": 40,
    "EVAC_CAR_COUNT": 15
  }
}
```

Interpretation:

- `light`: weak disruption, limited closures
- `moderate`: noticeable snow / damage, partial closures
- `severe`: strong disruption, substantial closures
- `extreme`: near worst-case closure pressure

## Example

```bash
python3 evacuation_main.py --mode batch --scenario scenarios/enterprise_blizzard.json --output-dir logs
```
