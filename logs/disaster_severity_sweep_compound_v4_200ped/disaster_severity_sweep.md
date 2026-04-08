# Disaster Severity Sweep

- Base scenario: `scenarios/enterprise_compound.json`
- DRQN checkpoint: `logs/drqn_progressive_severity_v3_extreme/drqn_torch_best.pt`
- Policies: `['drqn']`

- `light` / `drqn`: reached=0.6255, alive=1.0, exposure=79.4369, t90=None, t95=None, gap=-0.0054, reassignments=0.0
- `moderate` / `drqn`: reached=0.3891, alive=1.0, exposure=62.9864, t90=None, t95=None, gap=-0.1854, reassignments=0.0
- `severe` / `drqn`: reached=0.3546, alive=1.0, exposure=61.3878, t90=None, t95=None, gap=-0.0949, reassignments=0.0
- `extreme` / `drqn`: reached=0.43, alive=1.0, exposure=37.1057, t90=None, t95=None, gap=-0.1426, reassignments=0.0
