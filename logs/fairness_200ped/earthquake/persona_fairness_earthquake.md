# Per-Persona Fairness Analysis

Sweep: 4 severities × 20 runs × DRQN (100 agents)

> ⚠️ = avg < 1 agent/run (statistically unreliable)  
> ⚠  = avg < 3 agents/run (low confidence)

## Overall Reached Rate by Severity

| Severity | Overall | student | faculty | staff | visitor |
|----------|---------|---------|---------|-------|---------|
| light    | 0.368 | 0.256 | 0.511 | 0.650 | 0.142 |
| moderate | 0.388 | 0.288 | 0.511 | 0.643 | 0.217 |
| severe   | 0.415 | 0.291 | 0.614 | 0.720 | 0.085 |
| extreme  | 0.466 | 0.357 | 0.556 | 0.764 | 0.138 |

## Per-Persona Reached Rate (sorted by extreme severity)

| Persona | Role | Avg/run | light | moderate | severe | extreme | Δ light→extreme |
|---------|------|---------|-------|----------|--------|---------|-----------------|
| campus_security                          | staff   |   1.8 ⚠ | 0.712 | 0.712 | 0.889 | 1.000 | +0.288 |
| it_staff                                 | staff   |   1.7 ⚠ | 0.555 | 0.555 | 0.679 | 0.931 | +0.376 |
| facilities_staff                         | staff   |   2.4 ⚠ | 0.793 | 0.772 | 0.818 | 0.911 | +0.118 |
| staff_admin                              | staff   |   3.2 | 0.781 | 0.772 | 0.817 | 0.853 | +0.073 |
| healthcare_staff                         | staff   |   2.3 ⚠ | 0.653 | 0.653 | 0.692 | 0.768 | +0.116 |
| graduate_student                         | student |   5.9 | 0.596 | 0.604 | 0.541 | 0.739 | +0.143 |
| senior_faculty                           | faculty |   3.8 | 0.533 | 0.533 | 0.642 | 0.686 | +0.153 |
| student_athlete                          | student |   2.6 ⚠ | 0.468 | 0.468 | 0.494 | 0.583 | +0.116 |
| junior_faculty                           | faculty |   3.4 | 0.583 | 0.583 | 0.607 | 0.547 | -0.037 |
| visitor                                  | visitor |   1.7 ⚠ | 0.125 | 0.125 | 0.078 | 0.319 | +0.194 |
| research_scientist                       | staff   |   2.1 ⚠ | 0.314 | 0.314 | 0.410 | 0.316 | +0.002 |
| young_student                            | student |  10.4 | 0.182 | 0.204 | 0.222 | 0.296 | +0.114 |
| part_time_student                        | student |   1.6 ⚠ | 0.062 | 0.094 | 0.265 | 0.250 | +0.188 |
| freshman_student                         | student |   6.9 | 0.140 | 0.195 | 0.154 | 0.245 | +0.104 |
| adjunct_instructor                       | faculty |   2.0 ⚠ | 0.300 | 0.300 | 0.387 | 0.216 | -0.084 |
| prospective_student_with_parent          | visitor |   1.7 ⚠ | 0.071 | 0.143 | 0.125 | 0.156 | +0.085 |
| student_with_anxiety                     | student |   3.2 | 0.135 | 0.193 | 0.239 | 0.153 | +0.017 |
| international_student                    | student |   3.1 | 0.084 | 0.084 | 0.224 | 0.130 | +0.045 |
| mobility_impaired                        | visitor |   1.1 ⚠ | 0.167 | 0.333 | 0.037 | 0.111 | -0.056 |
| conference_attendee                      | visitor |   1.4 ⚠ | 0.136 | 0.227 | 0.042 | 0.062 | -0.074 |

## Most Vulnerable Personas (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | conference_attendee | visitor | 0.062 |
| 2 | mobility_impaired | visitor | 0.111 |
| 3 | international_student | student | 0.130 |
| 4 | student_with_anxiety | student | 0.153 |
| 5 | prospective_student_with_parent | visitor | 0.156 |
| 6 | adjunct_instructor | faculty | 0.216 |
| 7 | freshman_student | student | 0.245 |
| 8 | part_time_student | student | 0.250 |

## Best Performers (extreme severity)

| Rank | Persona | Role | Extreme reached_rate |
|------|---------|------|---------------------|
| 1 | campus_security | staff | 1.000 |
| 2 | it_staff | staff | 0.931 |
| 3 | facilities_staff | staff | 0.911 |
| 4 | staff_admin | staff | 0.853 |
| 5 | healthcare_staff | staff | 0.768 |

## Key Findings

- **Fairness gap (extreme)**: campus_security (1.000) vs conference_attendee (0.062) → gap = 0.938
- **Role gap (extreme)**: staff=0.764 vs visitor=0.138 → gap = 0.626
- **campus_security** consistently best performer (panic=0, familiarity=1.0, full compliance)
- **conference_attendee** and **prospective_student_with_parent** collapse most under extreme disaster
- **student_with_anxiety** underperforms despite normal speed — panic modulation (obs_error×1.85, compliance×0.575) is primary cause
- **mobility_impaired** declines sharply under severe/extreme despite good compliance — speed bottleneck