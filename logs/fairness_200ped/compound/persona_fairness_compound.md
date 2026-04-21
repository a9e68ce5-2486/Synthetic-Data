# Per-Persona Fairness Analysis

Sweep: 4 severities × 20 runs × DRQN (100 agents)

> ⚠️ = avg < 1 agent/run (statistically unreliable)  
> ⚠  = avg < 3 agents/run (low confidence)

## Overall Reached Rate by Severity

| Severity | Overall | student | faculty | staff | visitor |
|----------|---------|---------|---------|-------|---------|
| light    | 0.625 | 0.591 | 0.671 | 0.695 | 0.481 |
| moderate | 0.368 | 0.334 | 0.341 | 0.526 | 0.316 |
| severe   | 0.340 | 0.262 | 0.469 | 0.524 | 0.183 |
| extreme  | 0.424 | 0.330 | 0.536 | 0.698 | 0.140 |

## Per-Persona Reached Rate (sorted by extreme severity)

| Persona | Role | Avg/run | light | moderate | severe | extreme | Δ light→extreme |
|---------|------|---------|-------|----------|--------|---------|-----------------|
| campus_security                          | staff   |   1.7 ⚠ | 0.774 | 0.774 | 0.894 | 0.889 | +0.115 |
| staff_admin                              | staff   |   3.5 | 0.718 | 0.489 | 0.583 | 0.828 | +0.110 |
| facilities_staff                         | staff   |   2.4 ⚠ | 0.751 | 0.523 | 0.541 | 0.818 | +0.067 |
| it_staff                                 | staff   |   1.9 ⚠ | 0.576 | 0.548 | 0.500 | 0.750 | +0.174 |
| healthcare_staff                         | staff   |   2.0 ⚠ | 0.706 | 0.521 | 0.578 | 0.643 | -0.063 |
| senior_faculty                           | faculty |   4.0 | 0.686 | 0.413 | 0.549 | 0.634 | -0.053 |
| graduate_student                         | student |   6.4 | 0.700 | 0.389 | 0.469 | 0.625 | -0.075 |
| junior_faculty                           | faculty |   3.2 | 0.711 | 0.246 | 0.528 | 0.521 | -0.190 |
| student_athlete                          | student |   2.3 ⚠ | 0.802 | 0.338 | 0.332 | 0.456 | -0.347 |
| part_time_student                        | student |   1.7 ⚠ | 0.401 | 0.250 | 0.125 | 0.324 | -0.078 |
| young_student                            | student |  10.1 | 0.565 | 0.339 | 0.241 | 0.304 | -0.261 |
| international_student                    | student |   3.4 | 0.539 | 0.304 | 0.101 | 0.247 | -0.292 |
| research_scientist                       | staff   |   2.0 ⚠ | 0.448 | 0.269 | 0.229 | 0.228 | -0.220 |
| freshman_student                         | student |   7.2 | 0.633 | 0.308 | 0.183 | 0.179 | -0.454 |
| conference_attendee                      | visitor |   1.5 ⚠ | 0.278 | 0.278 | 0.333 | 0.167 | -0.111 |
| visitor                                  | visitor |   1.6 ⚠ | 0.731 | 0.389 | 0.227 | 0.167 | -0.564 |
| student_with_anxiety                     | student |   2.6 ⚠ | 0.372 | 0.292 | 0.276 | 0.153 | -0.219 |
| mobility_impaired                        | visitor |   1.2 ⚠ | 0.750 | 0.300 | 0.000 | 0.148 | -0.602 |
| adjunct_instructor                       | faculty |   1.9 ⚠ | 0.632 | 0.216 | 0.197 | 0.131 | -0.501 |
| prospective_student_with_parent          | visitor |   1.4 ⚠ | 0.250 | 0.050 | 0.000 | 0.083 | -0.167 |

## Most Vulnerable Personas (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | prospective_student_with_parent | visitor | 0.083 |
| 2 | adjunct_instructor | faculty | 0.131 |
| 3 | mobility_impaired | visitor | 0.148 |
| 4 | student_with_anxiety | student | 0.153 |
| 5 | conference_attendee | visitor | 0.167 |
| 6 | visitor | visitor | 0.167 |
| 7 | freshman_student | student | 0.179 |
| 8 | research_scientist | staff | 0.228 |

## Best Performers (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | campus_security | staff | 0.889 |
| 2 | staff_admin | staff | 0.828 |
| 3 | facilities_staff | staff | 0.818 |
| 4 | it_staff | staff | 0.750 |
| 5 | healthcare_staff | staff | 0.643 |

## Key Findings

- **Fairness gap (extreme)**: campus_security (0.889) vs prospective_student_with_parent (0.083) → gap = 0.806
- **Role gap (extreme)**: staff=0.698 vs visitor=0.140 → gap = 0.558
- **campus_security** consistently best performer (panic=0, familiarity=1.0, full compliance)
- **conference_attendee** and **prospective_student_with_parent** collapse most under extreme disaster
- **student_with_anxiety** underperforms despite normal speed — panic modulation (obs_error×1.85, compliance×0.575) is primary cause
- **mobility_impaired** declines sharply under severe/extreme despite good compliance — speed bottleneck