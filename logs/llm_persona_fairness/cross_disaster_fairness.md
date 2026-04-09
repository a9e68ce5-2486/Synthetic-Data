# Cross-Disaster Per-Persona Fairness Comparison

Disasters compared: blizzard, earthquake, compound

All values = reached_rate at **extreme** severity

> ⚠️ = avg < 1 agent/run (statistically unreliable)  
> ⚠  = avg < 3 agents/run (low confidence)

| Persona | Role | Avg/run | blizzard | earthquake | compound | Most vulnerable in |
|---------|------|---------|--------|--------|--------|--------------------|
| conference_attendee                      | visitor |  1.4 ⚠ | 0.389 | 0.167 | 0.125 | compound |
| freshman_student                         | student |  7.1 | 0.428 | 0.209 | 0.167 | compound |
| part_time_student                        | student |  1.7 ⚠ | 0.396 | 0.181 | 0.270 | earthquake |
| visitor                                  | visitor |  1.6 ⚠ | 0.513 | 0.208 | 0.178 | compound |
| international_student                    | student |  3.3 | 0.515 | 0.194 | 0.249 | earthquake |
| student_with_anxiety                     | student |  2.7 ⚠ | 0.439 | 0.306 | 0.243 | compound |
| young_student                            | student | 10.1 | 0.566 | 0.248 | 0.195 | compound |
| research_scientist                       | staff   |  2.0 ⚠ | 0.318 | 0.382 | 0.309 | compound |
| mobility_impaired                        | visitor |  1.2 ⚠ | 0.875 | 0.000 | 0.167 | earthquake |
| adjunct_instructor                       | faculty |  1.9 ⚠ | 0.456 | 0.424 | 0.202 | compound |
| prospective_student_with_parent          | visitor |  1.4 ⚠ | 0.750 | 0.062 | 0.333 | earthquake |
| junior_faculty                           | faculty |  3.2 | 0.634 | 0.458 | 0.478 | earthquake |
| student_athlete                          | student |  2.5 ⚠ | 0.552 | 0.491 | 0.553 | earthquake |
| graduate_student                         | student |  6.3 | 0.550 | 0.681 | 0.572 | blizzard |
| senior_faculty                           | faculty |  4.0 | 0.598 | 0.640 | 0.700 | blizzard |
| healthcare_staff                         | staff   |  2.0 ⚠ | 0.559 | 0.782 | 0.724 | blizzard |
| it_staff                                 | staff   |  2.0 ⚠ | 0.515 | 0.847 | 0.714 | blizzard |
| staff_admin                              | staff   |  3.3 | 0.520 | 0.887 | 0.828 | blizzard |
| facilities_staff                         | staff   |  2.5 ⚠ | 0.560 | 0.911 | 0.818 | blizzard |
| campus_security                          | staff   |  1.8 ⚠ | 0.595 | 1.000 | 0.889 | blizzard |

## Role Comparison Across Disasters (extreme severity)

| Role | blizzard | earthquake | compound |
|------|--------|--------|--------|
| student | 0.522 | 0.334 | 0.308 |
| faculty | 0.579 | 0.543 | 0.535 |
| staff   | 0.557 | 0.796 | 0.718 |
| visitor | 0.606 | 0.136 | 0.170 |

## Overall Reached Rate by Disaster × Severity

| Disaster | light | moderate | severe | extreme |
|----------|-------|----------|--------|---------|
| blizzard | 0.790 | 0.724 | 0.678 | 0.544 |
| earthquake | 0.384 | 0.396 | 0.404 | 0.449 |
| compound | 0.625 | 0.374 | 0.363 | 0.417 |