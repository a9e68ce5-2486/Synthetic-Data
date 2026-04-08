# Disaster Severity Sweep

- Base scenario: `scenarios/enterprise_blizzard.json`
- DRQN checkpoint: `logs/drqn_progressive_severity_v3_extreme/drqn_torch_best.pt`
- Policies: `['drqn']`

- `light` / `drqn`: reached=0.7755, alive=1.0, exposure=36.4536, t90=None, t95=None, gap=-0.018, reassignments=0.0
- `moderate` / `drqn`: reached=0.7164, alive=1.0, exposure=91.4995, t90=None, t95=None, gap=-0.0707, reassignments=0.0
- `severe` / `drqn`: reached=0.6236, alive=1.0, exposure=138.1085, t90=None, t95=None, gap=-0.0372, reassignments=0.0
- `extreme` / `drqn`: reached=0.5909, alive=1.0, exposure=92.2343, t90=None, t95=None, gap=-0.0466, reassignments=0.0
