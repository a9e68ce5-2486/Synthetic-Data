# Per-Persona Fairness Analysis

Sweep: 4 severities × 20 runs × DRQN (100 agents)

> ⚠️ = avg < 1 agent/run (statistically unreliable)  
> ⚠  = avg < 3 agents/run (low confidence)

## Overall Reached Rate by Severity

| Severity | Overall | student | faculty | staff | visitor |
|----------|---------|---------|---------|-------|---------|
| light    | 0.625 | 0.603 | 0.671 | 0.676 | 0.428 |
| moderate | 0.389 | 0.350 | 0.366 | 0.551 | 0.343 |
| severe   | 0.355 | 0.285 | 0.437 | 0.532 | 0.235 |
| extreme  | 0.430 | 0.338 | 0.531 | 0.674 | 0.249 |

## Per-Persona Reached Rate (sorted by extreme severity)

| Persona | Role | Avg/run | light | moderate | severe | extreme | Δ light→extreme |
|---------|------|---------|-------|----------|--------|---------|-----------------|
| campus_security                          | staff   |   1.7 ⚠ | 0.798 | 0.774 | 0.849 | 0.889 | +0.091 |
| staff_admin                              | staff   |   3.5 | 0.688 | 0.549 | 0.616 | 0.844 | +0.155 |
| facilities_staff                         | staff   |   2.4 ⚠ | 0.751 | 0.551 | 0.524 | 0.818 | +0.067 |
| senior_faculty                           | faculty |   4.0 | 0.697 | 0.365 | 0.590 | 0.695 | -0.003 |
| healthcare_staff                         | staff   |   2.0 ⚠ | 0.676 | 0.540 | 0.547 | 0.673 | -0.004 |
| graduate_student                         | student |   6.4 | 0.685 | 0.412 | 0.489 | 0.632 | -0.053 |
| student_athlete                          | student |   2.3 ⚠ | 0.844 | 0.485 | 0.444 | 0.532 | -0.311 |
| it_staff                                 | staff   |   1.9 ⚠ | 0.545 | 0.548 | 0.512 | 0.500 | -0.045 |
| prospective_student_with_parent          | visitor |   1.4 ⚠ | 0.312 | 0.367 | 0.143 | 0.458 | +0.146 |
| junior_faculty                           | faculty |   3.2 | 0.685 | 0.346 | 0.496 | 0.439 | -0.246 |
| part_time_student                        | student |   1.7 ⚠ | 0.490 | 0.250 | 0.115 | 0.343 | -0.146 |
| mobility_impaired                        | visitor |   1.2 ⚠ | 0.625 | 0.000 | 0.429 | 0.296 | -0.329 |
| young_student                            | student |  10.1 | 0.590 | 0.374 | 0.255 | 0.286 | -0.304 |
| visitor                                  | visitor |   1.6 ⚠ | 0.705 | 0.375 | 0.076 | 0.233 | -0.472 |
| international_student                    | student |   3.4 | 0.520 | 0.252 | 0.254 | 0.226 | -0.294 |
| freshman_student                         | student |   7.2 | 0.558 | 0.301 | 0.172 | 0.218 | -0.340 |
| student_with_anxiety                     | student |   2.6 ⚠ | 0.449 | 0.325 | 0.255 | 0.187 | -0.262 |
| research_scientist                       | staff   |   2.0 ⚠ | 0.401 | 0.278 | 0.271 | 0.169 | -0.232 |
| conference_attendee                      | visitor |   1.5 ⚠ | 0.389 | 0.389 | 0.167 | 0.167 | -0.222 |
| adjunct_instructor                       | faculty |   1.9 ⚠ | 0.603 | 0.279 | 0.171 | 0.042 | -0.561 |

## Most Vulnerable Personas (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | adjunct_instructor | faculty | 0.042 |
| 2 | conference_attendee | visitor | 0.167 |
| 3 | research_scientist | staff | 0.169 |
| 4 | student_with_anxiety | student | 0.187 |
| 5 | freshman_student | student | 0.218 |
| 6 | international_student | student | 0.226 |
| 7 | visitor | visitor | 0.233 |
| 8 | young_student | student | 0.286 |

## Best Performers (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | campus_security | staff | 0.889 |
| 2 | staff_admin | staff | 0.844 |
| 3 | facilities_staff | staff | 0.818 |
| 4 | senior_faculty | faculty | 0.695 |
| 5 | healthcare_staff | staff | 0.673 |

## Key Findings

- **Fairness gap (extreme)**: campus_security (0.889) vs adjunct_instructor (0.042) → gap = 0.847
- **Role gap (extreme)**: staff=0.674 vs visitor=0.249 → gap = 0.425
- **campus_security** consistently best performer (panic=0, familiarity=1.0, full compliance)
- **conference_attendee** and **prospective_student_with_parent** collapse most under extreme disaster
- **student_with_anxiety** underperforms despite normal speed — panic modulation (obs_error×1.85, compliance×0.575) is primary cause
- **mobility_impaired** declines sharply under severe/extreme despite good compliance — speed bottleneck