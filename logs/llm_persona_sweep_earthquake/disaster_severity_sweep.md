# Disaster Severity Sweep

- Base scenario: `scenarios/enterprise_earthquake.json`
- DRQN checkpoint: `logs/drqn_llm_persona/drqn_torch_best.pt`
- Policies: `['drqn']`

- `light` / `drqn`: reached=0.3836, alive=1.0, exposure=112.949, t90=None, t95=None, gap=-0.1877, reassignments=0.0
- `moderate` / `drqn`: reached=0.3955, alive=1.0, exposure=72.6921, t90=None, t95=None, gap=-0.1827, reassignments=0.0
- `severe` / `drqn`: reached=0.4036, alive=1.0, exposure=62.1459, t90=None, t95=None, gap=-0.1924, reassignments=0.0
- `extreme` / `drqn`: reached=0.4491, alive=1.0, exposure=27.3355, t90=None, t95=None, gap=-0.2524, reassignments=0.0
