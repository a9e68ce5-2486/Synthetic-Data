# Per-Persona Fairness Analysis

Sweep: 4 severities × 20 runs × DRQN (100 agents)

> ⚠️ = avg < 1 agent/run (statistically unreliable)  
> ⚠  = avg < 3 agents/run (low confidence)

## Overall Reached Rate by Severity

| Severity | Overall | student | faculty | staff | visitor |
|----------|---------|---------|---------|-------|---------|
| light    | 0.775 | 0.751 | 0.829 | 0.847 | 0.672 |
| moderate | 0.716 | 0.684 | 0.760 | 0.831 | 0.544 |
| severe   | 0.624 | 0.598 | 0.668 | 0.705 | 0.477 |
| extreme  | 0.591 | 0.553 | 0.666 | 0.713 | 0.410 |

## Per-Persona Reached Rate (sorted by extreme severity)

| Persona | Role | Avg/run | light | moderate | severe | extreme | Δ light→extreme |
|---------|------|---------|-------|----------|--------|---------|-----------------|
| campus_security                          | staff   |   1.8 ⚠ | 0.958 | 0.917 | 0.819 | 0.833 | -0.125 |
| healthcare_staff                         | staff   |   1.8 ⚠ | 0.774 | 0.877 | 0.676 | 0.767 | -0.008 |
| senior_faculty                           | faculty |   4.3 | 0.861 | 0.784 | 0.708 | 0.726 | -0.135 |
| junior_faculty                           | faculty |   3.1 | 0.832 | 0.759 | 0.755 | 0.719 | -0.113 |
| research_scientist                       | staff   |   1.8 ⚠ | 0.733 | 0.662 | 0.510 | 0.713 | -0.020 |
| staff_admin                              | staff   |   3.5 | 0.894 | 0.825 | 0.682 | 0.669 | -0.225 |
| facilities_staff                         | staff   |   2.5 ⚠ | 0.887 | 0.898 | 0.793 | 0.662 | -0.225 |
| graduate_student                         | student |   6.8 | 0.801 | 0.742 | 0.668 | 0.643 | -0.157 |
| international_student                    | student |   3.5 | 0.653 | 0.700 | 0.625 | 0.611 | -0.043 |
| student_athlete                          | student |   2.5 ⚠ | 0.902 | 0.704 | 0.625 | 0.608 | -0.294 |
| it_staff                                 | staff   |   2.2 ⚠ | 0.844 | 0.827 | 0.708 | 0.575 | -0.269 |
| part_time_student                        | student |   1.8 ⚠ | 0.706 | 0.583 | 0.458 | 0.521 | -0.185 |
| freshman_student                         | student |   7.2 | 0.734 | 0.646 | 0.656 | 0.520 | -0.214 |
| young_student                            | student |   9.8 | 0.782 | 0.686 | 0.539 | 0.512 | -0.270 |
| visitor                                  | visitor |   1.6 ⚠ | 0.714 | 0.589 | 0.548 | 0.449 | -0.266 |
| mobility_impaired                        | visitor |   1.3 ⚠ | 0.833 | 0.667 | 0.700 | 0.429 | -0.405 |
| adjunct_instructor                       | faculty |   1.9 ⚠ | 0.729 | 0.756 | 0.543 | 0.420 | -0.309 |
| student_with_anxiety                     | student |   2.2 ⚠ | 0.554 | 0.584 | 0.319 | 0.384 | -0.170 |
| prospective_student_with_parent          | visitor |   1.1 ⚠ | 0.500 | 0.500 | 0.250 | 0.250 | -0.250 |
| conference_attendee                      | visitor |   1.4 ⚠ | 0.578 | 0.385 | 0.438 | 0.125 | -0.453 |

## Most Vulnerable Personas (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | conference_attendee | visitor | 0.125 |
| 2 | prospective_student_with_parent | visitor | 0.250 |
| 3 | student_with_anxiety | student | 0.384 |
| 4 | adjunct_instructor | faculty | 0.420 |
| 5 | mobility_impaired | visitor | 0.429 |
| 6 | visitor | visitor | 0.449 |
| 7 | young_student | student | 0.512 |
| 8 | freshman_student | student | 0.520 |

## Best Performers (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | campus_security | staff | 0.833 |
| 2 | healthcare_staff | staff | 0.767 |
| 3 | senior_faculty | faculty | 0.726 |
| 4 | junior_faculty | faculty | 0.719 |
| 5 | research_scientist | staff | 0.713 |

## Key Findings

- **Fairness gap (extreme)**: campus_security (0.833) vs conference_attendee (0.125) → gap = 0.708
- **Role gap (extreme)**: staff=0.713 vs visitor=0.410 → gap = 0.303
- **campus_security** consistently best performer (panic=0, familiarity=1.0, full compliance)
- **conference_attendee** and **prospective_student_with_parent** collapse most under extreme disaster
- **student_with_anxiety** underperforms despite normal speed — panic modulation (obs_error×1.85, compliance×0.575) is primary cause
- **mobility_impaired** declines sharply under severe/extreme despite good compliance — speed bottleneck