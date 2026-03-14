# Scenario Data Layer

This folder separates **scenario definition** from code-level config editing.

## Structure

- `enterprise_*.json`: runnable scenario files for experiments.
- `profiles/*.json`: reusable disaster profiles (hazard/observability assumptions).

## How merging works

1. Load scenario file.
2. If `profile` is set, load `profiles/<profile>.json`.
3. Apply profile `config_overrides`.
4. Apply scenario `config_overrides` on top (scenario wins on conflicts).

## Minimal scenario fields

- `name`
- `num_runs`
- `base_seed`
- `policies`
- `config_overrides`

## Example

```bash
python3 evacuation_main.py --mode batch --scenario scenarios/enterprise_blizzard.json --output-dir logs
```
