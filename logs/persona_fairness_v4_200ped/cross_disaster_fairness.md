# Cross-Disaster Per-Persona Fairness Comparison

Disasters compared: blizzard, earthquake, compound

All values = reached_rate at **extreme** severity

> ⚠️ = avg < 1 agent/run (statistically unreliable)  
> ⚠  = avg < 3 agents/run (low confidence)

| Persona | Role | Avg/run | blizzard | earthquake | compound | Most vulnerable in |
|---------|------|---------|--------|--------|--------|--------------------|
| conference_attendee                      | visitor |  1.4 ⚠ | 0.125 | 0.125 | 0.167 | blizzard |
| adjunct_instructor                       | faculty |  1.9 ⚠ | 0.420 | 0.183 | 0.042 | compound |
| prospective_student_with_parent          | visitor |  1.3 ⚠ | 0.250 | 0.000 | 0.458 | earthquake |
| student_with_anxiety                     | student |  2.4 ⚠ | 0.384 | 0.211 | 0.187 | compound |
| visitor                                  | visitor |  1.6 ⚠ | 0.449 | 0.121 | 0.233 | earthquake |
| mobility_impaired                        | visitor |  1.2 ⚠ | 0.429 | 0.100 | 0.296 | earthquake |
| freshman_student                         | student |  7.1 | 0.520 | 0.165 | 0.218 | earthquake |
| part_time_student                        | student |  1.8 ⚠ | 0.521 | 0.093 | 0.343 | earthquake |
| young_student                            | student | 10.3 | 0.512 | 0.184 | 0.286 | earthquake |
| international_student                    | student |  3.4 | 0.611 | 0.191 | 0.226 | earthquake |
| research_scientist                       | staff   |  1.9 ⚠ | 0.713 | 0.359 | 0.169 | compound |
| it_staff                                 | staff   |  2.0 ⚠ | 0.575 | 0.424 | 0.500 | earthquake |
| student_athlete                          | student |  2.4 ⚠ | 0.608 | 0.412 | 0.532 | earthquake |
| junior_faculty                           | faculty |  3.1 | 0.719 | 0.481 | 0.439 | compound |
| graduate_student                         | student |  6.7 | 0.643 | 0.593 | 0.632 | earthquake |
| senior_faculty                           | faculty |  4.2 | 0.726 | 0.722 | 0.695 | compound |
| healthcare_staff                         | staff   |  1.9 ⚠ | 0.767 | 0.745 | 0.673 | compound |
| facilities_staff                         | staff   |  2.4 ⚠ | 0.662 | 0.850 | 0.818 | blizzard |
| staff_admin                              | staff   |  3.3 | 0.669 | 0.837 | 0.844 | blizzard |
| campus_security                          | staff   |  1.6 ⚠ | 0.833 | 0.833 | 0.889 | blizzard |

## Role Comparison Across Disasters (extreme severity)

| Role | blizzard | earthquake | compound |
|------|--------|--------|--------|
| student | 0.553 | 0.284 | 0.338 |
| faculty | 0.666 | 0.595 | 0.531 |
| staff   | 0.713 | 0.702 | 0.674 |
| visitor | 0.410 | 0.080 | 0.249 |

## Overall Reached Rate by Disaster × Severity

| Disaster | light | moderate | severe | extreme |
|----------|-------|----------|--------|---------|
| blizzard | 0.775 | 0.716 | 0.624 | 0.591 |
| earthquake | 0.629 | 0.489 | 0.338 | 0.405 |
| compound | 0.625 | 0.389 | 0.355 | 0.430 |