# Disaster Severity Sweep

- Base scenario: `scenarios/enterprise_compound.json`
- DRQN checkpoint: `logs/drqn_llm_persona/drqn_torch_best.pt`
- Policies: `['drqn']`

- `light` / `drqn`: reached=0.6255, alive=1.0, exposure=72.4214, t90=None, t95=None, gap=-0.0248, reassignments=0.0
- `moderate` / `drqn`: reached=0.3682, alive=1.0, exposure=62.6089, t90=None, t95=None, gap=-0.1859, reassignments=0.0
- `severe` / `drqn`: reached=0.34, alive=1.0, exposure=58.9911, t90=None, t95=None, gap=-0.0556, reassignments=0.0
- `extreme` / `drqn`: reached=0.4236, alive=1.0, exposure=33.7343, t90=None, t95=None, gap=-0.1618, reassignments=0.0
