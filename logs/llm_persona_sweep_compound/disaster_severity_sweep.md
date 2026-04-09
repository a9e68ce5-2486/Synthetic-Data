# Disaster Severity Sweep

- Base scenario: `scenarios/enterprise_compound.json`
- DRQN checkpoint: `logs/drqn_llm_persona/drqn_torch_best.pt`
- Policies: `['drqn']`

- `light` / `drqn`: reached=0.6245, alive=1.0, exposure=76.3546, t90=None, t95=None, gap=-0.0668, reassignments=0.0
- `moderate` / `drqn`: reached=0.3745, alive=1.0, exposure=65.4545, t90=None, t95=None, gap=-0.106, reassignments=0.0
- `severe` / `drqn`: reached=0.3627, alive=1.0, exposure=63.6833, t90=None, t95=None, gap=-0.0436, reassignments=0.0
- `extreme` / `drqn`: reached=0.4173, alive=1.0, exposure=37.6712, t90=None, t95=None, gap=-0.1832, reassignments=0.0
