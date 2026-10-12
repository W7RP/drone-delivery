Named weeks of the 6b demo, seeds 1 to 3 pooled (requests summed over seeds).

| Week | Rule | Policy | 6b ratio | Drone | Truck fallback | Rejected | Blocked weather | Blocked energy | In flight | Waiting | Diverted/failed | Completion | Ratio, delivered | Ratio, lower bound |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2023-10-23 | median wind | WAIT | 0.30 | 3603 | 0 | 178 | 129 | 0 | 0 | 0 | 60 | 90.8% | 0.30 | 0.90 |
| 2023-10-23 | median wind | TRUCK_FALLBACK | 0.48 | 3603 | 275 | 0 | 32 | 0 | 0 | 0 | 60 | 97.7% | 0.48 | 0.50 |
| 2024-01-08 | windiest | WAIT | 11.13 | 1586 | 0 | 2216 | 138 | 0 | 0 | 0 | 30 | 39.9% | 11.20 | 19.67 |
| 2024-01-08 | windiest | TRUCK_FALLBACK | 12.81 | 1087 | 1113 | 1708 | 30 | 0 | 0 | 0 | 32 | 55.4% | 12.78 | 12.63 |
| 2025-01-20 | coldest | WAIT | 40.72 | 733 | 0 | 3133 | 0 | 74 | 0 | 15 | 15 | 18.5% | 40.68 | 36.74 |
| 2025-01-20 | coldest | TRUCK_FALLBACK | 6.89 | 776 | 2064 | 1023 | 0 | 74 | 0 | 15 | 18 | 71.5% | 6.89 | 6.81 |

Truck-fallback timeout, TRUCK_FALLBACK.

| Week | Rule | Policy | Timeout (s) | Drone | Truck fallback | Rejected | Blocked weather | Blocked energy | In flight | Waiting | Diverted/failed | Completion | Ratio, delivered | Ratio, lower bound |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2024-01-08 | windiest | TRUCK_FALLBACK | 1800 | 1612 | 1680 | 628 | 17 | 0 | 0 | 0 | 33 | 82.9% | 5.33 | 5.33 |
| 2024-01-08 | windiest | TRUCK_FALLBACK | 3600 | 1087 | 1113 | 1708 | 30 | 0 | 0 | 0 | 32 | 55.4% | 12.78 | 12.63 |
| 2024-01-08 | windiest | TRUCK_FALLBACK | 7200 | 1772 | 673 | 1356 | 133 | 0 | 0 | 0 | 36 | 61.6% | 10.80 | 11.33 |
| 2025-01-20 | coldest | TRUCK_FALLBACK | 1800 | 772 | 2101 | 986 | 0 | 74 | 0 | 15 | 22 | 72.4% | 6.06 | 6.00 |
| 2025-01-20 | coldest | TRUCK_FALLBACK | 3600 | 776 | 2064 | 1023 | 0 | 74 | 0 | 15 | 18 | 71.5% | 6.89 | 6.81 |
| 2025-01-20 | coldest | TRUCK_FALLBACK | 7200 | 778 | 1972 | 1114 | 0 | 74 | 0 | 15 | 17 | 69.3% | 8.56 | 8.43 |

With the energy route guard (step 2).

| Week | Rule | Policy | Timeout (s) | Drone | Truck fallback | Rejected | Blocked weather | Blocked energy | In flight | Waiting | Diverted/failed | Completion | Ratio, delivered | Ratio, lower bound |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2024-01-08 | windiest | WAIT | 3600 | 1647 | 0 | 2135 | 134 | 0 | 0 | 0 | 54 | 41.5% | 5.19 | 15.81 |
| 2024-01-08 | windiest | TRUCK_FALLBACK | 1800 | 2102 | 1793 | 15 | 16 | 0 | 0 | 0 | 44 | 98.1% | 1.51 | 1.51 |
| 2024-01-08 | windiest | TRUCK_FALLBACK | 3600 | 2264 | 1611 | 20 | 31 | 0 | 0 | 0 | 44 | 97.6% | 1.96 | 1.97 |
| 2024-01-08 | windiest | TRUCK_FALLBACK | 7200 | 2353 | 1441 | 29 | 101 | 0 | 0 | 0 | 46 | 95.6% | 3.29 | 3.32 |
| 2025-01-20 | coldest | WAIT | 3600 | 848 | 0 | 3045 | 0 | 54 | 1 | 0 | 22 | 21.4% | 27.52 | 26.08 |
| 2025-01-20 | coldest | TRUCK_FALLBACK | 1800 | 732 | 3225 | 0 | 0 | 8 | 1 | 0 | 4 | 99.7% | 1.64 | 1.63 |
| 2025-01-20 | coldest | TRUCK_FALLBACK | 3600 | 738 | 3199 | 0 | 0 | 17 | 1 | 0 | 15 | 99.2% | 2.41 | 2.40 |
| 2025-01-20 | coldest | TRUCK_FALLBACK | 7200 | 772 | 3147 | 5 | 0 | 32 | 1 | 0 | 13 | 98.7% | 3.95 | 3.93 |
