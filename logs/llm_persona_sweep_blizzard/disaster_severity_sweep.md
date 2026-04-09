# Disaster Severity Sweep

- Base scenario: `scenarios/enterprise_blizzard.json`
- DRQN checkpoint: `logs/drqn_llm_persona/drqn_torch_best.pt`
- Policies: `['drqn']`

- `light` / `drqn`: reached=0.79, alive=1.0, exposure=31.2609, t90=None, t95=None, gap=-0.0637, reassignments=0.0
- `moderate` / `drqn`: reached=0.7236, alive=1.0, exposure=70.6697, t90=None, t95=None, gap=-0.0379, reassignments=0.0
- `severe` / `drqn`: reached=0.6782, alive=1.0, exposure=84.3664, t90=None, t95=None, gap=0.0442, reassignments=0.0
- `extreme` / `drqn`: reached=0.5445, alive=1.0, exposure=72.5192, t90=None, t95=None, gap=0.0218, reassignments=0.0
