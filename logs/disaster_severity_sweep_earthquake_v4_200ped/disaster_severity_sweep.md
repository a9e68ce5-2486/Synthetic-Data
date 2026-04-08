# Disaster Severity Sweep

- Base scenario: `scenarios/enterprise_earthquake.json`
- DRQN checkpoint: `logs/drqn_progressive_severity_v3_extreme/drqn_torch_best.pt`
- Policies: `['drqn']`

- `light` / `drqn`: reached=0.6291, alive=1.0, exposure=132.833, t90=None, t95=None, gap=0.0411, reassignments=0.0
- `moderate` / `drqn`: reached=0.4891, alive=1.0, exposure=169.7981, t90=None, t95=None, gap=-0.1124, reassignments=0.0
- `severe` / `drqn`: reached=0.3382, alive=1.0, exposure=185.5086, t90=None, t95=None, gap=-0.195, reassignments=0.0
- `extreme` / `drqn`: reached=0.4054, alive=1.0, exposure=138.2769, t90=None, t95=None, gap=-0.1066, reassignments=0.0
