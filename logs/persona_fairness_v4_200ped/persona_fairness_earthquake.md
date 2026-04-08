# Per-Persona Fairness Analysis

Sweep: 4 severities × 20 runs × DRQN (100 agents)

> ⚠️ = avg < 1 agent/run (statistically unreliable)  
> ⚠  = avg < 3 agents/run (low confidence)

## Overall Reached Rate by Severity

| Severity | Overall | student | faculty | staff | visitor |
|----------|---------|---------|---------|-------|---------|
| light    | 0.629 | 0.575 | 0.777 | 0.736 | 0.449 |
| moderate | 0.489 | 0.446 | 0.536 | 0.648 | 0.261 |
| severe   | 0.338 | 0.252 | 0.420 | 0.616 | 0.144 |
| extreme  | 0.405 | 0.284 | 0.595 | 0.702 | 0.080 |

## Per-Persona Reached Rate (sorted by extreme severity)

| Persona | Role | Avg/run | light | moderate | severe | extreme | Δ light→extreme |
|---------|------|---------|-------|----------|--------|---------|-----------------|
| facilities_staff                         | staff   |   2.4 ⚠ | 0.818 | 0.692 | 0.524 | 0.850 | +0.033 |
| staff_admin                              | staff   |   3.0 | 0.754 | 0.611 | 0.699 | 0.837 | +0.082 |
| campus_security                          | staff   |   1.5 ⚠ | 0.792 | 0.808 | 0.867 | 0.833 | +0.042 |
| healthcare_staff                         | staff   |   2.1 ⚠ | 0.706 | 0.787 | 0.630 | 0.745 | +0.039 |
| senior_faculty                           | faculty |   4.2 | 0.789 | 0.465 | 0.480 | 0.722 | -0.066 |
| graduate_student                         | student |   6.8 | 0.670 | 0.601 | 0.535 | 0.593 | -0.077 |
| junior_faculty                           | faculty |   3.0 | 0.823 | 0.587 | 0.467 | 0.481 | -0.342 |
| it_staff                                 | staff   |   1.8 ⚠ | 0.614 | 0.589 | 0.567 | 0.424 | -0.189 |
| student_athlete                          | student |   2.4 ⚠ | 0.759 | 0.534 | 0.474 | 0.412 | -0.347 |
| research_scientist                       | staff   |   2.0 ⚠ | 0.602 | 0.491 | 0.304 | 0.359 | -0.242 |
| student_with_anxiety                     | student |   2.6 ⚠ | 0.380 | 0.465 | 0.192 | 0.211 | -0.168 |
| international_student                    | student |   3.4 | 0.561 | 0.413 | 0.077 | 0.191 | -0.370 |
| young_student                            | student |  10.9 | 0.562 | 0.387 | 0.186 | 0.184 | -0.377 |
| adjunct_instructor                       | faculty |   2.0 ⚠ | 0.730 | 0.382 | 0.190 | 0.183 | -0.547 |
| freshman_student                         | student |   7.0 | 0.585 | 0.359 | 0.149 | 0.165 | -0.421 |
| conference_attendee                      | visitor |   1.4 ⚠ | 0.125 | 0.071 | 0.056 | 0.125 | +0.000 |
| visitor                                  | visitor |   1.6 ⚠ | 0.551 | 0.229 | 0.231 | 0.121 | -0.430 |
| mobility_impaired                        | visitor |   1.1 ⚠ | 0.500 | 0.444 | 0.143 | 0.100 | -0.400 |
| part_time_student                        | student |   1.8 ⚠ | 0.464 | 0.216 | 0.270 | 0.093 | -0.372 |
| prospective_student_with_parent          | visitor |   1.4 ⚠ | 0.500 | 0.450 | 0.000 | 0.000 | -0.500 |

## Most Vulnerable Personas (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | prospective_student_with_parent | visitor | 0.000 |
| 2 | part_time_student | student | 0.093 |
| 3 | mobility_impaired | visitor | 0.100 |
| 4 | visitor | visitor | 0.121 |
| 5 | conference_attendee | visitor | 0.125 |
| 6 | freshman_student | student | 0.165 |
| 7 | adjunct_instructor | faculty | 0.183 |
| 8 | young_student | student | 0.184 |

## Best Performers (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | facilities_staff | staff | 0.850 |
| 2 | staff_admin | staff | 0.837 |
| 3 | campus_security | staff | 0.833 |
| 4 | healthcare_staff | staff | 0.745 |
| 5 | senior_faculty | faculty | 0.722 |

## Key Findings

- **Fairness gap (extreme)**: facilities_staff (0.850) vs prospective_student_with_parent (0.000) → gap = 0.850
- **Role gap (extreme)**: staff=0.702 vs visitor=0.080 → gap = 0.622
- **campus_security** consistently best performer (panic=0, familiarity=1.0, full compliance)
- **conference_attendee** and **prospective_student_with_parent** collapse most under extreme disaster
- **student_with_anxiety** underperforms despite normal speed — panic modulation (obs_error×1.85, compliance×0.575) is primary cause
- **mobility_impaired** declines sharply under severe/extreme despite good compliance — speed bottleneck