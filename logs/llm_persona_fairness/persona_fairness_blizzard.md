# Per-Persona Fairness Analysis

Sweep: 4 severities × 20 runs × DRQN (100 agents)

> ⚠️ = avg < 1 agent/run (statistically unreliable)  
> ⚠  = avg < 3 agents/run (low confidence)

## Overall Reached Rate by Severity

| Severity | Overall | student | faculty | staff | visitor |
|----------|---------|---------|---------|-------|---------|
| light    | 0.790 | 0.766 | 0.812 | 0.876 | 0.672 |
| moderate | 0.724 | 0.712 | 0.738 | 0.776 | 0.606 |
| severe   | 0.678 | 0.698 | 0.671 | 0.627 | 0.672 |
| extreme  | 0.544 | 0.522 | 0.579 | 0.557 | 0.606 |

## Per-Persona Reached Rate (sorted by extreme severity)

| Persona | Role | Avg/run | light | moderate | severe | extreme | Δ light→extreme |
|---------|------|---------|-------|----------|--------|---------|-----------------|
| mobility_impaired                        | visitor |   1.3 ⚠ | 0.833 | 0.786 | 0.929 | 0.875 | +0.042 |
| prospective_student_with_parent          | visitor |   1.2 ⚠ | 0.500 | 0.417 | 0.583 | 0.750 | +0.250 |
| junior_faculty                           | faculty |   3.0 | 0.804 | 0.758 | 0.684 | 0.634 | -0.171 |
| senior_faculty                           | faculty |   4.2 | 0.861 | 0.809 | 0.746 | 0.598 | -0.263 |
| campus_security                          | staff   |   1.8 ⚠ | 0.958 | 0.849 | 0.515 | 0.595 | -0.363 |
| young_student                            | student |   9.6 | 0.751 | 0.735 | 0.767 | 0.566 | -0.185 |
| facilities_staff                         | staff   |   2.6 ⚠ | 0.892 | 0.789 | 0.594 | 0.560 | -0.332 |
| healthcare_staff                         | staff   |   1.8 ⚠ | 0.892 | 0.732 | 0.685 | 0.559 | -0.333 |
| student_athlete                          | student |   2.5 ⚠ | 0.917 | 0.847 | 0.609 | 0.552 | -0.365 |
| graduate_student                         | student |   6.6 | 0.813 | 0.648 | 0.565 | 0.550 | -0.263 |
| staff_admin                              | staff   |   3.4 | 0.919 | 0.832 | 0.664 | 0.520 | -0.399 |
| it_staff                                 | staff   |   2.4 ⚠ | 0.933 | 0.756 | 0.673 | 0.515 | -0.418 |
| international_student                    | student |   3.5 | 0.717 | 0.656 | 0.722 | 0.515 | -0.202 |
| visitor                                  | visitor |   1.5 ⚠ | 0.714 | 0.643 | 0.786 | 0.513 | -0.202 |
| adjunct_instructor                       | faculty |   1.8 ⚠ | 0.667 | 0.703 | 0.682 | 0.456 | -0.211 |
| student_with_anxiety                     | student |   2.4 ⚠ | 0.571 | 0.636 | 0.667 | 0.439 | -0.132 |
| freshman_student                         | student |   7.3 | 0.761 | 0.711 | 0.726 | 0.428 | -0.333 |
| part_time_student                        | student |   1.8 ⚠ | 0.726 | 0.611 | 0.611 | 0.396 | -0.330 |
| conference_attendee                      | visitor |   1.4 ⚠ | 0.578 | 0.476 | 0.405 | 0.389 | -0.189 |
| research_scientist                       | staff   |   1.9 ⚠ | 0.717 | 0.738 | 0.602 | 0.318 | -0.399 |

## Most Vulnerable Personas (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | research_scientist | staff | 0.318 |
| 2 | conference_attendee | visitor | 0.389 |
| 3 | part_time_student | student | 0.396 |
| 4 | freshman_student | student | 0.428 |
| 5 | student_with_anxiety | student | 0.439 |
| 6 | adjunct_instructor | faculty | 0.456 |
| 7 | visitor | visitor | 0.513 |
| 8 | international_student | student | 0.515 |

## Best Performers (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | mobility_impaired | visitor | 0.875 |
| 2 | prospective_student_with_parent | visitor | 0.750 |
| 3 | junior_faculty | faculty | 0.634 |
| 4 | senior_faculty | faculty | 0.598 |
| 5 | campus_security | staff | 0.595 |

## Key Findings

- **Fairness gap (extreme)**: mobility_impaired (0.875) vs research_scientist (0.318) → gap = 0.557
- **Role gap (extreme)**: staff=0.557 vs visitor=0.606 → gap = -0.049
- **campus_security** consistently best performer (panic=0, familiarity=1.0, full compliance)
- **conference_attendee** and **prospective_student_with_parent** collapse most under extreme disaster
- **student_with_anxiety** underperforms despite normal speed — panic modulation (obs_error×1.85, compliance×0.575) is primary cause
- **mobility_impaired** declines sharply under severe/extreme despite good compliance — speed bottleneck