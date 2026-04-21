# Disaster Severity Sweep

- Base scenario: `scenarios/enterprise_earthquake.json`
- DRQN checkpoint: `logs/drqn_llm_persona/drqn_torch_best.pt`
- Policies: `['drqn']`

- `light` / `drqn`: reached=0.3682, alive=1.0, exposure=116.1186, t90=None, t95=None, gap=-0.1393, reassignments=0.0
- `moderate` / `drqn`: reached=0.3882, alive=1.0, exposure=75.9108, t90=None, t95=None, gap=-0.1326, reassignments=0.0
- `severe` / `drqn`: reached=0.4155, alive=1.0, exposure=60.8517, t90=None, t95=None, gap=-0.1057, reassignments=0.0
- `extreme` / `drqn`: reached=0.4664, alive=1.0, exposure=21.4573, t90=None, t95=None, gap=-0.208, reassignments=0.0
