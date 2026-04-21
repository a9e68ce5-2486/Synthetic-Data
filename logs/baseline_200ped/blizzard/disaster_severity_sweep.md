# Disaster Severity Sweep

- Base scenario: `scenarios/enterprise_blizzard.json`
- DRQN checkpoint: `logs/drqn_llm_persona/drqn_torch_best.pt`
- Policies: `['round_robin', 'nearest', 'drqn']`

- `light` / `round_robin`: reached=0.0373, alive=1.0, exposure=295.1181, t90=None, t95=None, gap=0.0106, reassignments=0.0
- `light` / `nearest`: reached=0.0745, alive=1.0, exposure=291.6362, t90=None, t95=None, gap=-0.0338, reassignments=0.0
- `light` / `drqn`: reached=0.7891, alive=1.0, exposure=37.3429, t90=None, t95=None, gap=-0.0724, reassignments=0.0
- `moderate` / `round_robin`: reached=0.0009, alive=1.0, exposure=43.0017, t90=None, t95=None, gap=0.0, reassignments=0.0
- `moderate` / `nearest`: reached=0.0046, alive=1.0, exposure=37.3086, t90=None, t95=None, gap=-0.0054, reassignments=0.0
- `moderate` / `drqn`: reached=0.7137, alive=1.0, exposure=73.6556, t90=None, t95=None, gap=-0.0216, reassignments=0.0
- `severe` / `round_robin`: reached=0.0073, alive=1.0, exposure=64.8094, t90=None, t95=None, gap=0.0, reassignments=0.0
- `severe` / `nearest`: reached=0.0227, alive=1.0, exposure=79.7534, t90=None, t95=None, gap=-0.0054, reassignments=0.0
- `severe` / `drqn`: reached=0.6718, alive=1.0, exposure=91.653, t90=None, t95=None, gap=0.0183, reassignments=0.0
- `extreme` / `round_robin`: reached=0.0145, alive=1.0, exposure=77.0643, t90=None, t95=None, gap=-0.0078, reassignments=0.0
- `extreme` / `nearest`: reached=0.0191, alive=1.0, exposure=51.9424, t90=None, t95=None, gap=-0.0198, reassignments=0.0
- `extreme` / `drqn`: reached=0.5191, alive=1.0, exposure=63.7693, t90=None, t95=None, gap=0.0229, reassignments=0.0
