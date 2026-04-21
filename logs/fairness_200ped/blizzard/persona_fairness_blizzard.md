# Per-Persona Fairness Analysis

Sweep: 4 severities × 20 runs × DRQN (100 agents)

> ⚠️ = avg < 1 agent/run (statistically unreliable)  
> ⚠  = avg < 3 agents/run (low confidence)

## Overall Reached Rate by Severity

| Severity | Overall | student | faculty | staff | visitor |
|----------|---------|---------|---------|-------|---------|
| light    | 0.787 | 0.754 | 0.851 | 0.883 | 0.681 |
| moderate | 0.715 | 0.699 | 0.752 | 0.761 | 0.570 |
| severe   | 0.676 | 0.684 | 0.707 | 0.644 | 0.620 |
| extreme  | 0.526 | 0.497 | 0.589 | 0.566 | 0.461 |

## Per-Persona Reached Rate (sorted by extreme severity)

| Persona | Role | Avg/run | light | moderate | severe | extreme | Δ light→extreme |
|---------|------|---------|-------|----------|--------|---------|-----------------|
| junior_faculty                           | faculty |   3.0 | 0.894 | 0.784 | 0.750 | 0.677 | -0.217 |
| senior_faculty                           | faculty |   4.2 | 0.886 | 0.817 | 0.751 | 0.631 | -0.255 |
| young_student                            | student |   9.6 | 0.745 | 0.733 | 0.759 | 0.594 | -0.151 |
| healthcare_staff                         | staff   |   1.8 ⚠ | 0.863 | 0.732 | 0.713 | 0.588 | -0.275 |
| campus_security                          | staff   |   1.8 ⚠ | 1.000 | 0.849 | 0.515 | 0.577 | -0.423 |
| facilities_staff                         | staff   |   2.6 ⚠ | 0.887 | 0.771 | 0.632 | 0.560 | -0.327 |
| student_with_anxiety                     | student |   2.4 ⚠ | 0.554 | 0.557 | 0.671 | 0.540 | -0.014 |
| staff_admin                              | staff   |   3.4 | 0.936 | 0.815 | 0.673 | 0.537 | -0.399 |
| it_staff                                 | staff   |   2.4 ⚠ | 0.844 | 0.756 | 0.601 | 0.508 | -0.337 |
| mobility_impaired                        | visitor |   1.3 ⚠ | 0.833 | 0.786 | 0.786 | 0.500 | -0.333 |
| prospective_student_with_parent          | visitor |   1.2 ⚠ | 0.500 | 0.417 | 0.667 | 0.500 | +0.000 |
| research_scientist                       | staff   |   1.9 ⚠ | 0.717 | 0.675 | 0.621 | 0.479 | -0.237 |
| part_time_student                        | student |   1.8 ⚠ | 0.627 | 0.657 | 0.630 | 0.469 | -0.159 |
| freshman_student                         | student |   7.3 | 0.754 | 0.721 | 0.730 | 0.460 | -0.294 |
| graduate_student                         | student |   6.6 | 0.795 | 0.658 | 0.552 | 0.450 | -0.345 |
| student_athlete                          | student |   2.5 ⚠ | 0.917 | 0.820 | 0.691 | 0.417 | -0.500 |
| visitor                                  | visitor |   1.5 ⚠ | 0.714 | 0.571 | 0.607 | 0.410 | -0.304 |
| conference_attendee                      | visitor |   1.4 ⚠ | 0.544 | 0.452 | 0.452 | 0.389 | -0.155 |
| international_student                    | student |   3.5 | 0.717 | 0.639 | 0.649 | 0.372 | -0.345 |
| adjunct_instructor                       | faculty |   1.8 ⚠ | 0.698 | 0.703 | 0.703 | 0.279 | -0.418 |

## Most Vulnerable Personas (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | adjunct_instructor | faculty | 0.279 |
| 2 | international_student | student | 0.372 |
| 3 | conference_attendee | visitor | 0.389 |
| 4 | visitor | visitor | 0.410 |
| 5 | student_athlete | student | 0.417 |
| 6 | graduate_student | student | 0.450 |
| 7 | freshman_student | student | 0.460 |
| 8 | part_time_student | student | 0.469 |

## Best Performers (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | junior_faculty | faculty | 0.677 |
| 2 | senior_faculty | faculty | 0.631 |
| 3 | young_student | student | 0.594 |
| 4 | healthcare_staff | staff | 0.588 |
| 5 | campus_security | staff | 0.577 |

## Key Findings

- **Fairness gap (extreme)**: junior_faculty (0.677) vs adjunct_instructor (0.279) → gap = 0.397
- **Role gap (extreme)**: staff=0.566 vs visitor=0.461 → gap = 0.104
- **campus_security** consistently best performer (panic=0, familiarity=1.0, full compliance)
- **conference_attendee** and **prospective_student_with_parent** collapse most under extreme disaster
- **student_with_anxiety** underperforms despite normal speed — panic modulation (obs_error×1.85, compliance×0.575) is primary cause
- **mobility_impaired** declines sharply under severe/extreme despite good compliance — speed bottleneck