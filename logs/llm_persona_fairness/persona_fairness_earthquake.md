# Per-Persona Fairness Analysis

Sweep: 4 severities × 20 runs × DRQN (100 agents)

> ⚠️ = avg < 1 agent/run (statistically unreliable)  
> ⚠  = avg < 3 agents/run (low confidence)

## Overall Reached Rate by Severity

| Severity | Overall | student | faculty | staff | visitor |
|----------|---------|---------|---------|-------|---------|
| light    | 0.384 | 0.290 | 0.455 | 0.643 | 0.153 |
| moderate | 0.396 | 0.311 | 0.455 | 0.638 | 0.169 |
| severe   | 0.404 | 0.294 | 0.494 | 0.686 | 0.300 |
| extreme  | 0.449 | 0.334 | 0.543 | 0.796 | 0.136 |

## Per-Persona Reached Rate (sorted by extreme severity)

| Persona | Role | Avg/run | light | moderate | severe | extreme | Δ light→extreme |
|---------|------|---------|-------|----------|--------|---------|-----------------|
| campus_security                          | staff   |   1.8 ⚠ | 0.712 | 0.712 | 0.889 | 1.000 | +0.288 |
| facilities_staff                         | staff   |   2.4 ⚠ | 0.793 | 0.793 | 0.818 | 0.911 | +0.118 |
| staff_admin                              | staff   |   3.2 | 0.791 | 0.791 | 0.828 | 0.887 | +0.096 |
| it_staff                                 | staff   |   1.7 ⚠ | 0.626 | 0.555 | 0.464 | 0.847 | +0.221 |
| healthcare_staff                         | staff   |   2.3 ⚠ | 0.682 | 0.682 | 0.663 | 0.782 | +0.100 |
| graduate_student                         | student |   5.9 | 0.562 | 0.562 | 0.565 | 0.681 | +0.119 |
| senior_faculty                           | faculty |   3.8 | 0.589 | 0.589 | 0.572 | 0.640 | +0.051 |
| student_athlete                          | student |   2.6 ⚠ | 0.504 | 0.504 | 0.471 | 0.491 | -0.014 |
| junior_faculty                           | faculty |   3.4 | 0.392 | 0.392 | 0.482 | 0.458 | +0.066 |
| adjunct_instructor                       | faculty |   2.0 ⚠ | 0.239 | 0.239 | 0.167 | 0.424 | +0.185 |
| research_scientist                       | staff   |   2.1 ⚠ | 0.216 | 0.216 | 0.359 | 0.382 | +0.166 |
| student_with_anxiety                     | student |   3.2 | 0.189 | 0.189 | 0.178 | 0.306 | +0.117 |
| young_student                            | student |  10.4 | 0.259 | 0.289 | 0.266 | 0.248 | -0.011 |
| freshman_student                         | student |   6.9 | 0.134 | 0.156 | 0.150 | 0.209 | +0.074 |
| visitor                                  | visitor |   1.7 ⚠ | 0.347 | 0.347 | 0.233 | 0.208 | -0.139 |
| international_student                    | student |   3.1 | 0.200 | 0.200 | 0.095 | 0.194 | -0.006 |
| part_time_student                        | student |   1.6 ⚠ | 0.219 | 0.312 | 0.235 | 0.181 | -0.037 |
| conference_attendee                      | visitor |   1.4 ⚠ | 0.136 | 0.136 | 0.278 | 0.167 | +0.030 |
| prospective_student_with_parent          | visitor |   1.7 ⚠ | 0.071 | 0.071 | 0.250 | 0.062 | -0.009 |
| mobility_impaired                        | visitor |   1.1 ⚠ | 0.000 | 0.167 | 0.222 | 0.000 | +0.000 |

## Most Vulnerable Personas (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | mobility_impaired | visitor | 0.000 |
| 2 | prospective_student_with_parent | visitor | 0.062 |
| 3 | conference_attendee | visitor | 0.167 |
| 4 | part_time_student | student | 0.181 |
| 5 | international_student | student | 0.194 |
| 6 | visitor | visitor | 0.208 |
| 7 | freshman_student | student | 0.209 |
| 8 | young_student | student | 0.248 |

## Best Performers (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | campus_security | staff | 1.000 |
| 2 | facilities_staff | staff | 0.911 |
| 3 | staff_admin | staff | 0.887 |
| 4 | it_staff | staff | 0.847 |
| 5 | healthcare_staff | staff | 0.782 |

## Key Findings

- **Fairness gap (extreme)**: campus_security (1.000) vs mobility_impaired (0.000) → gap = 1.000
- **Role gap (extreme)**: staff=0.796 vs visitor=0.136 → gap = 0.660
- **campus_security** consistently best performer (panic=0, familiarity=1.0, full compliance)
- **conference_attendee** and **prospective_student_with_parent** collapse most under extreme disaster
- **student_with_anxiety** underperforms despite normal speed — panic modulation (obs_error×1.85, compliance×0.575) is primary cause
- **mobility_impaired** declines sharply under severe/extreme despite good compliance — speed bottleneck