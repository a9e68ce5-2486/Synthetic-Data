# Disaster Severity Sweep

- Base scenario: `scenarios/enterprise_blizzard.json`
- DRQN checkpoint: `logs/drqn_llm_persona/drqn_torch_best.pt`
- Policies: `['drqn']`

- `light` / `drqn`: reached=0.7873, alive=1.0, exposure=33.79, t90=None, t95=None, gap=-0.0317, reassignments=0.0
- `moderate` / `drqn`: reached=0.7146, alive=1.0, exposure=62.2855, t90=None, t95=None, gap=-0.0091, reassignments=0.0
- `severe` / `drqn`: reached=0.6764, alive=1.0, exposure=84.7528, t90=None, t95=None, gap=0.0636, reassignments=0.0
- `extreme` / `drqn`: reached=0.5264, alive=1.0, exposure=68.9179, t90=None, t95=None, gap=0.023, reassignments=0.0
