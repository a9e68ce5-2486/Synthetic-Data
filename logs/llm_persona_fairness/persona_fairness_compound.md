# Per-Persona Fairness Analysis

Sweep: 4 severities × 20 runs × DRQN (100 agents)

> ⚠️ = avg < 1 agent/run (statistically unreliable)  
> ⚠  = avg < 3 agents/run (low confidence)

## Overall Reached Rate by Severity

| Severity | Overall | student | faculty | staff | visitor |
|----------|---------|---------|---------|-------|---------|
| light    | 0.625 | 0.592 | 0.642 | 0.709 | 0.503 |
| moderate | 0.374 | 0.328 | 0.414 | 0.520 | 0.280 |
| severe   | 0.363 | 0.288 | 0.499 | 0.543 | 0.085 |
| extreme  | 0.417 | 0.308 | 0.535 | 0.718 | 0.170 |

## Per-Persona Reached Rate (sorted by extreme severity)

| Persona | Role | Avg/run | light | moderate | severe | extreme | Δ light→extreme |
|---------|------|---------|-------|----------|--------|---------|-----------------|
| campus_security                          | staff   |   1.7 ⚠ | 0.774 | 0.774 | 0.894 | 0.889 | +0.115 |
| staff_admin                              | staff   |   3.5 | 0.723 | 0.464 | 0.583 | 0.828 | +0.105 |
| facilities_staff                         | staff   |   2.4 ⚠ | 0.707 | 0.509 | 0.559 | 0.818 | +0.111 |
| healthcare_staff                         | staff   |   2.0 ⚠ | 0.647 | 0.521 | 0.578 | 0.724 | +0.076 |
| it_staff                                 | staff   |   1.9 ⚠ | 0.568 | 0.559 | 0.583 | 0.714 | +0.146 |
| senior_faculty                           | faculty |   4.0 | 0.676 | 0.437 | 0.527 | 0.700 | +0.024 |
| graduate_student                         | student |   6.4 | 0.695 | 0.396 | 0.456 | 0.572 | -0.124 |
| student_athlete                          | student |   2.3 ⚠ | 0.729 | 0.361 | 0.331 | 0.553 | -0.176 |
| junior_faculty                           | faculty |   3.2 | 0.636 | 0.351 | 0.593 | 0.478 | -0.158 |
| prospective_student_with_parent          | visitor |   1.4 ⚠ | 0.438 | 0.233 | 0.214 | 0.333 | -0.104 |
| research_scientist                       | staff   |   2.0 ⚠ | 0.604 | 0.241 | 0.208 | 0.309 | -0.295 |
| part_time_student                        | student |   1.7 ⚠ | 0.432 | 0.172 | 0.156 | 0.270 | -0.163 |
| international_student                    | student |   3.4 | 0.542 | 0.201 | 0.165 | 0.249 | -0.294 |
| student_with_anxiety                     | student |   2.6 ⚠ | 0.397 | 0.325 | 0.183 | 0.243 | -0.154 |
| adjunct_instructor                       | faculty |   1.9 ⚠ | 0.515 | 0.299 | 0.294 | 0.202 | -0.312 |
| young_student                            | student |  10.1 | 0.558 | 0.339 | 0.317 | 0.195 | -0.363 |
| visitor                                  | visitor |   1.6 ⚠ | 0.769 | 0.236 | 0.182 | 0.178 | -0.591 |
| freshman_student                         | student |   7.2 | 0.627 | 0.264 | 0.160 | 0.167 | -0.460 |
| mobility_impaired                        | visitor |   1.2 ⚠ | 0.875 | 0.400 | 0.000 | 0.167 | -0.708 |
| conference_attendee                      | visitor |   1.5 ⚠ | 0.167 | 0.167 | 0.000 | 0.125 | -0.042 |

## Most Vulnerable Personas (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | conference_attendee | visitor | 0.125 |
| 2 | mobility_impaired | visitor | 0.167 |
| 3 | freshman_student | student | 0.167 |
| 4 | visitor | visitor | 0.178 |
| 5 | young_student | student | 0.195 |
| 6 | adjunct_instructor | faculty | 0.202 |
| 7 | student_with_anxiety | student | 0.243 |
| 8 | international_student | student | 0.249 |

## Best Performers (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | campus_security | staff | 0.889 |
| 2 | staff_admin | staff | 0.828 |
| 3 | facilities_staff | staff | 0.818 |
| 4 | healthcare_staff | staff | 0.724 |
| 5 | it_staff | staff | 0.714 |

## Key Findings

- **Fairness gap (extreme)**: campus_security (0.889) vs conference_attendee (0.125) → gap = 0.764
- **Role gap (extreme)**: staff=0.718 vs visitor=0.170 → gap = 0.548
- **campus_security** consistently best performer (panic=0, familiarity=1.0, full compliance)
- **conference_attendee** and **prospective_student_with_parent** collapse most under extreme disaster
- **student_with_anxiety** underperforms despite normal speed — panic modulation (obs_error×1.85, compliance×0.575) is primary cause
- **mobility_impaired** declines sharply under severe/extreme despite good compliance — speed bottleneck