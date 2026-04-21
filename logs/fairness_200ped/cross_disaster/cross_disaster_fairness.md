# Cross-Disaster Per-Persona Fairness Comparison

Disasters compared: blizzard, earthquake, compound

All values = reached_rate at **extreme** severity

> ⚠️ = avg < 1 agent/run (statistically unreliable)  
> ⚠  = avg < 3 agents/run (low confidence)

| Persona | Role | Avg/run | blizzard | earthquake | compound | Most vulnerable in |
|---------|------|---------|--------|--------|--------|--------------------|
| conference_attendee                      | visitor |  1.4 ⚠ | 0.389 | 0.062 | 0.167 | earthquake |
| adjunct_instructor                       | faculty |  1.9 ⚠ | 0.279 | 0.216 | 0.131 | compound |
| prospective_student_with_parent          | visitor |  1.4 ⚠ | 0.500 | 0.156 | 0.083 | compound |
| international_student                    | student |  3.3 | 0.372 | 0.130 | 0.247 | earthquake |
| mobility_impaired                        | visitor |  1.2 ⚠ | 0.500 | 0.111 | 0.148 | earthquake |
| student_with_anxiety                     | student |  2.7 ⚠ | 0.540 | 0.153 | 0.153 | earthquake |
| freshman_student                         | student |  7.1 | 0.460 | 0.245 | 0.179 | compound |
| visitor                                  | visitor |  1.6 ⚠ | 0.410 | 0.319 | 0.167 | compound |
| research_scientist                       | staff   |  2.0 ⚠ | 0.479 | 0.316 | 0.228 | compound |
| part_time_student                        | student |  1.7 ⚠ | 0.469 | 0.250 | 0.324 | earthquake |
| young_student                            | student | 10.1 | 0.594 | 0.296 | 0.304 | earthquake |
| student_athlete                          | student |  2.5 ⚠ | 0.417 | 0.583 | 0.456 | blizzard |
| junior_faculty                           | faculty |  3.2 | 0.677 | 0.547 | 0.521 | compound |
| graduate_student                         | student |  6.3 | 0.450 | 0.739 | 0.625 | blizzard |
| senior_faculty                           | faculty |  4.0 | 0.631 | 0.686 | 0.634 | blizzard |
| healthcare_staff                         | staff   |  2.0 ⚠ | 0.588 | 0.768 | 0.643 | blizzard |
| it_staff                                 | staff   |  2.0 ⚠ | 0.508 | 0.931 | 0.750 | blizzard |
| staff_admin                              | staff   |  3.3 | 0.537 | 0.853 | 0.828 | blizzard |
| facilities_staff                         | staff   |  2.5 ⚠ | 0.560 | 0.911 | 0.818 | blizzard |
| campus_security                          | staff   |  1.8 ⚠ | 0.577 | 1.000 | 0.889 | blizzard |

## Role Comparison Across Disasters (extreme severity)

| Role | blizzard | earthquake | compound |
|------|--------|--------|--------|
| student | 0.497 | 0.357 | 0.330 |
| faculty | 0.589 | 0.556 | 0.536 |
| staff   | 0.566 | 0.764 | 0.698 |
| visitor | 0.461 | 0.138 | 0.140 |

## Overall Reached Rate by Disaster × Severity

| Disaster | light | moderate | severe | extreme |
|----------|-------|----------|--------|---------|
| blizzard | 0.787 | 0.715 | 0.676 | 0.526 |
| earthquake | 0.368 | 0.388 | 0.415 | 0.466 |
| compound | 0.625 | 0.368 | 0.340 | 0.424 |