156 weeks, seeds [1, 2] pooled per week; median (p10 to p90) over weeks.

(a) Year.

| Base | Policy | Ratio, delivered | Ratio, lower bound | Completion | Flyable hours |
|---|---|---|---|---|---|
| multirotor | WAIT | 0.59 (0.27 to 7.15) | 0.61 (0.27 to 8.02) | 97.7% (60.7% to 98.5%) | 96.4% (85.7% to 100.0%) |
| multirotor | TRUCK_FALLBACK | 0.42 (0.27 to 6.19) | 0.43 (0.27 to 6.56) | 98.1% (67.9% to 98.6%) | 96.4% (85.7% to 100.0%) |
| vtol | WAIT | 5.16 (1.48 to 16.89) | 6.27 (1.57 to 37.43) | 63.1% (14.4% to 94.0%) | 96.4% (85.7% to 100.0%) |
| vtol | TRUCK_FALLBACK | 0.98 (0.63 to 1.36) | 0.98 (0.62 to 1.36) | 98.5% (98.1% to 98.8%) | 96.4% (85.7% to 100.0%) |

Outcome breakdown, share of every request in the year.

| Base | Policy | Requests | delivered_drone | delivered_truck | rejected_cap | blocked_weather | blocked_energy | in_flight | waiting | diverted_or_failed |
|---|---|---|---|---|---|---|---|---|---|---|
| multirotor | WAIT | 425568 | 87.2% | 0.0% | 10.7% | 0.2% | 0.2% | 0.0% | 0.1% | 1.5% |
| multirotor | TRUCK_FALLBACK | 425568 | 85.7% | 4.9% | 7.4% | 0.0% | 0.2% | 0.0% | 0.1% | 1.4% |
| vtol | WAIT | 417924 | 57.6% | 0.0% | 37.1% | 0.2% | 1.5% | 0.0% | 0.2% | 3.4% |
| vtol | TRUCK_FALLBACK | 417924 | 71.8% | 26.7% | 0.0% | 0.0% | 0.2% | 0.0% | 0.0% | 1.2% |

(b) Named weeks: drone-delivered orders, mean door-to-door split (s) next to their truck baseline.

| Week | Rule | Base | Policy | Drone | Trucked | Completion | Pod wait | Queue | Pad service | Flight | Door-to-door | Truck | Ratio, delivered | Ratio, lower bound |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2023-10-23 | median wind | multirotor | WAIT | 2468 | 0 | 90.5% | 118 | 172 | 147 | 152 | 589 | 1919 | 0.31 | 0.91 |
| 2023-10-23 | median wind | multirotor | TRUCK_FALLBACK | 2468 | 189 | 97.4% | 118 | 172 | 147 | 152 | 589 | 1919 | 0.49 | 0.51 |
| 2023-10-23 | median wind | vtol | WAIT | 1769 | 0 | 66.0% | 9234 | 1044 | 201 | 376 | 10855 | 2212 | 4.91 | 5.65 |
| 2023-10-23 | median wind | vtol | TRUCK_FALLBACK | 1847 | 781 | 98.1% | 158 | 150 | 145 | 286 | 739 | 2094 | 1.06 | 1.05 |
| 2024-01-08 | windiest | multirotor | WAIT | 968 | 0 | 35.5% | 17944 | 2830 | 243 | 152 | 21170 | 1922 | 11.01 | 21.71 |
| 2024-01-08 | windiest | multirotor | TRUCK_FALLBACK | 691 | 748 | 52.7% | 10345 | 2649 | 232 | 152 | 13379 | 1951 | 13.20 | 13.04 |
| 2024-01-08 | windiest | vtol | WAIT | 182 | 0 | 6.8% | 25560 | 6522 | 184 | 233 | 32499 | 2016 | 16.12 | 77.79 |
| 2024-01-08 | windiest | vtol | TRUCK_FALLBACK | 1359 | 1270 | 98.1% | 155 | 345 | 168 | 184 | 853 | 1957 | 1.54 | 1.54 |
| 2025-01-20 | coldest | multirotor | WAIT | 501 | 0 | 18.4% | 70553 | 3541 | 193 | 149 | 74436 | 1891 | 39.36 | 35.46 |
| 2025-01-20 | coldest | multirotor | TRUCK_FALLBACK | 519 | 1404 | 70.5% | 6629 | 1324 | 203 | 148 | 8304 | 1880 | 6.83 | 6.74 |
| 2025-01-20 | coldest | vtol | WAIT | 189 | 0 | 7.1% | 84602 | 9269 | 193 | 208 | 94273 | 2000 | 47.13 | 63.28 |
| 2025-01-20 | coldest | vtol | TRUCK_FALLBACK | 753 | 1906 | 99.3% | 125 | 78 | 129 | 169 | 501 | 1915 | 1.97 | 1.96 |

(c) Multirotor demo (6b as published, and rerun completion-aware) against the VTOL demo: median (p10 to p90) over weeks.

| Policy | 6b published ratio | 6b completed share | Multirotor ratio, delivered | Multirotor ratio, lower bound | Multirotor completion | VTOL ratio, delivered | VTOL ratio, lower bound | VTOL completion |
|---|---|---|---|---|---|---|---|---|
| WAIT | 0.56 (0.26 to 6.61) | 97.8% (61.1% to 98.5%) | 0.59 (0.27 to 7.15) | 0.61 (0.27 to 8.02) | 97.7% (60.7% to 98.5%) | 5.16 (1.48 to 16.89) | 6.27 (1.57 to 37.43) | 63.1% (14.4% to 94.0%) |
| TRUCK_FALLBACK | 0.41 (0.26 to 6.31) | 98.2% (68.4% to 98.6%) | 0.42 (0.27 to 6.19) | 0.43 (0.27 to 6.56) | 98.1% (67.9% to 98.6%) | 0.98 (0.63 to 1.36) | 0.98 (0.62 to 1.36) | 98.5% (98.1% to 98.8%) |

Mon Valley flows of the VTOL demo over the year (the multirotor network does not reach Mon Valley; its flows there are left out). Drone: drone-delivered orders, mean door-to-door against their truck baseline; trucked: orders the fallback sent by truck.

| Policy | Origin | Destination | Drone orders | Drone (s) | Truck (s) | Ratio | Trucked orders | Trucked (s) |
|---|---|---|---|---|---|---|---|---|
| WAIT | Consumer_Fresh_Produce | Mon_Valley_Works_Edgar_Thomson_P | 19537 | 39055 | 2921 | 13.37 | 0 | - |
| WAIT | Consumer_Fresh_Produce | Josh_Steel | 22279 | 16220 | 2719 | 5.96 | 0 | - |
| WAIT | Achieva_Mailing_Service | Mon_Valley_Works_Edgar_Thomson_P | 19750 | 37986 | 2857 | 13.29 | 0 | - |
| WAIT | Achieva_Mailing_Service | Josh_Steel | 22345 | 14005 | 2688 | 5.21 | 0 | - |
| TRUCK_FALLBACK | Consumer_Fresh_Produce | Mon_Valley_Works_Edgar_Thomson_P | 11157 | 1640 | 2912 | 0.56 | 31661 | 6548 |
| TRUCK_FALLBACK | Consumer_Fresh_Produce | Josh_Steel | 19065 | 1403 | 2719 | 0.52 | 21329 | 6385 |
| TRUCK_FALLBACK | Achieva_Mailing_Service | Mon_Valley_Works_Edgar_Thomson_P | 11548 | 1575 | 2869 | 0.55 | 27937 | 6534 |
| TRUCK_FALLBACK | Achieva_Mailing_Service | Josh_Steel | 19532 | 1365 | 2688 | 0.51 | 19347 | 6373 |
