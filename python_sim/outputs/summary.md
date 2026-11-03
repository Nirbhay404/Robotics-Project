# Results (30 random worlds per density, same worlds for every policy)

Time is averaged over successful runs only. Clearance = gap between robot edge and nearest obstacle (m).

| Density | Policy | Success % | Collision % | Time (s) | Mean speed | Min clearance | Mean clearance | Accel (m/s²) |
|---|---|---|---|---|---|---|---|---|
| low | Fixed 0.3 m/s | 97 | 3 | 46.3 | 0.30 | 0.32 | 1.25 | 0.00 |
| low | Fixed 1.0 m/s | 100 | 0 | 14.2 | 0.98 | 0.32 | 1.24 | 0.07 |
| low | Adaptive | 100 | 0 | 20.3 | 0.70 | 0.32 | 1.14 | 0.24 |
| medium | Fixed 0.3 m/s | 100 | 0 | 47.6 | 0.30 | 0.24 | 0.74 | 0.00 |
| medium | Fixed 1.0 m/s | 97 | 3 | 14.6 | 0.98 | 0.22 | 0.74 | 0.06 |
| medium | Adaptive | 100 | 0 | 26.6 | 0.55 | 0.24 | 0.70 | 0.29 |
| high | Fixed 0.3 m/s | 87 | 10 | 50.6 | 0.30 | 0.17 | 0.50 | 0.00 |
| high | Fixed 1.0 m/s | 80 | 20 | 15.2 | 0.98 | 0.13 | 0.52 | 0.07 |
| high | Adaptive | 87 | 0 | 35.9 | 0.40 | 0.22 | 0.50 | 0.27 |
