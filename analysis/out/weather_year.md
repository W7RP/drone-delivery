156 weeks (2023-01-02 to 2025-12-22), seeds [1, 2, 3].

| Policy | Ratio mean | p10 | Median | p90 | Flyable hours | Trucked share | Completed share | Weeks without a completed order |
|---|---|---|---|---|---|---|---|---|
| WAIT | 3.17 | 0.26 | 0.56 | 6.61 | 94% | 0% | 87% | 2 |
| TRUCK_FALLBACK | 2.14 | 0.26 | 0.41 | 6.31 | 94% | 7% | 91% | 0 |

| Week | Rule | Wind mean (m/s) | Gust max | Temp mean (C) | Snow h | Policy | Network (s) | Pod wait | Queue | Pad service | Flight | Truck, same flown orders (s) | Ratio | Flyable | Trucked | Completed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2023-10-23 | median wind | 3.1 | 10.8 | 15.2 | 0 | WAIT | 569 | 108 | 161 | 149 | 152 | 1919 | 0.30 | 90% | 0% | 91% |
| 2023-10-23 | median wind | 3.1 | 10.8 | 15.2 | 0 | TRUCK_FALLBACK | 920 | 108 | 161 | 149 | 152 | 1919 | 0.48 | 90% | 7% | 98% |
| 2024-01-08 | windiest | 6.0 | 26.2 | 1.4 | 20 | WAIT | 21390 | 18007 | 2994 | 237 | 153 | 1922 | 11.13 | 78% | 0% | 40% |
| 2024-01-08 | windiest | 6.0 | 26.2 | 1.4 | 20 | TRUCK_FALLBACK | 24915 | 10785 | 2682 | 232 | 152 | 1950 | 12.81 | 78% | 50% | 56% |
| 2025-01-20 | coldest | 3.9 | 13.9 | -8.6 | 10 | WAIT | 77039 | 73541 | 3149 | 200 | 149 | 1892 | 40.72 | 51% | 0% | 18% |
| 2025-01-20 | coldest | 3.9 | 13.9 | -8.6 | 10 | TRUCK_FALLBACK | 13220 | 6550 | 1385 | 205 | 148 | 1882 | 6.89 | 51% | 72% | 72% |

| Sensitivity (WAIT) | Ratio mean | Change | p10 | Median | p90 | Completed share |
|---|---|---|---|---|---|---|
| wind limits x0.5 | 8.28 | 5.110 | 0.64 | 3.58 | 15.97 | 72% |
| cold derate depth x1.5 | 7.12 | 3.947 | 0.26 | 0.87 | 16.16 | 76% |
| cold derate depth x0.5 | 0.80 | -2.367 | 0.26 | 0.40 | 1.66 | 96% |
| wind limits x1.5 | 3.02 | -0.149 | 0.26 | 0.53 | 5.96 | 88% |
| traffic peak delay x0.5 | 3.20 | 0.028 | 0.26 | 0.56 | 6.68 | 87% |
| traffic peak delay x1.5 | 3.14 | -0.027 | 0.26 | 0.55 | 6.55 | 87% |
| landing gust k x1.5 | 3.19 | 0.020 | 0.26 | 0.56 | 6.95 | 87% |
| landing gust k x0.5 | 3.16 | -0.010 | 0.25 | 0.55 | 6.48 | 88% |
