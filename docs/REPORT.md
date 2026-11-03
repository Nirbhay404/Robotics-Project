# Adaptive Speed Control in Cluttered Environments

Robotics and Intelligent Systems project

**Team members:**
- Nirbhay Kumar (24BCS089)
- Nikhil Kumar Sinha (24BCS086)
- Naman Kumar (24BCS080)

## 1. Problem
Fixed-speed navigation is a compromise. A high fixed speed is efficient in open space but unsafe
near obstacles; a low fixed speed is safe but wastes time everywhere.

**Objective:** a speed controller that adjusts velocity from obstacle proximity and environmental
complexity, evaluated against fixed-speed navigation.

## 2. Method
The navigation stack is split in two parts so only the speed policy changes between experiments.

**Steering (`steering.py`, identical for every policy).** Gap following: from the 360° LiDAR scan,
every direction gets a "free distance" (a beam blocks a direction if the obstacle it sees lies
within `safe_radius = 0.5 m` of that direction's straight line). The robot turns toward the
direction with the best score = free distance − distance from the goal direction (+ a small
hysteresis bonus to avoid dithering).

**Speed policy (`speed_policy.py`).**

```
v = clip( v_max × proximity × density × turn , v_min , v_max )     (stop if d_front ≤ d_stop)
proximity = clip((d_front − d_stop) / (d_slow − d_stop), 0, 1)      d_front = nearest return within ±40° ahead
density   = 1 − 0.4 × min(clutter / 0.45, 1)                        clutter = fraction of beams < 2 m
turn      = 1 − 0.3 × |ω| / ω_max                                   slow down in sharp turns
```
A rate limiter (+1.0 / −3.0 m/s²) keeps the command smooth.
Parameters: v_max 1.0 m/s, v_min 0.25, d_stop 0.55 m, d_slow 1.8 m.

Baselines: fixed 1.0 m/s (fast) and fixed 0.3 m/s (slow).

## 3. Experiment
- 12 × 12 m arena, start (−5, −5), goal (5, 5), 90 s time limit.
- Three densities: low 6, medium 14, high 24 circular obstacles (radius 0.3–0.55 m).
- 30 random worlds per density; every policy runs in the same worlds.
- Robot: unicycle, 0.3 m collision radius, accel limit 1.5 m/s², brake limit 3 m/s².
- LiDAR: 120 beams, 6 m range, 20 Hz, Gaussian noise σ = 0.02 m.
- Metrics: success, collision rate, completion time (successful runs), mean clearance, mean accel.

## 4. Results (30 worlds per density)
| Density | Policy | Success % | Collision % | Time (s) | Mean speed | Min clearance | Mean clearance |
|---|---|---|---|---|---|---|---|
| low | Fixed 0.3 | 97 | 3 | 46.3 | 0.30 | 0.32 | 1.25 |
| low | Fixed 1.0 | 100 | 0 | 14.2 | 0.98 | 0.32 | 1.24 |
| low | **Adaptive** | 100 | 0 | 20.3 | 0.70 | 0.32 | 1.14 |
| medium | Fixed 0.3 | 100 | 0 | 47.6 | 0.30 | 0.24 | 0.74 |
| medium | Fixed 1.0 | 97 | 3 | 14.6 | 0.98 | 0.22 | 0.74 |
| medium | **Adaptive** | 100 | 0 | 26.6 | 0.55 | 0.24 | 0.70 |
| high | Fixed 0.3 | 87 | 10 | 50.6 | 0.30 | 0.17 | 0.50 |
| high | Fixed 1.0 | 80 | 20 | 15.2 | 0.98 | 0.13 | 0.52 |
| high | **Adaptive** | 87 | **0** | 35.9 | 0.40 | 0.22 | 0.50 |

Figures: `python_sim/outputs/comparison.png`, `example_high_seed2.png` (same world, all three
policies), `adaptive_anatomy.png` (distance, clutter and speed over time).

## 5. What the results say (honest reading)
- **Adaptive had zero collisions in all three densities** (90 runs). Fixed 1.0 m/s collided in
  3% of medium and 20% of high-density worlds.
- **Adaptive is 2.3× faster than the slow fixed policy in low density (20 s vs 46 s) and 1.4×
  faster in high density (36 s vs 51 s)**, because it only slows down where it needs to.
- **Fixed 1.0 m/s is still the fastest** (about 14–15 s). Adaptive buys safety with time; it
  does not beat the fast policy on speed. That is the expected trade-off, not a free win.
- Slightly larger clearance in the worst case (min clearance 0.22 vs 0.13 m in high density).
- Speed is less smooth (mean accel 0.24–0.29 vs 0.07 m/s² for fixed 1.0), since it keeps changing.
- Failures that are not collisions (13% in high density, all policies) are timeouts, mostly
  the reactive steering getting stuck in local traps. That limit comes from the steering, not
  from the speed policy.

## 5b. ROS 2 / Gazebo check
The same `speed_policy.py` and `steering.py` run inside the ROS 2 node (`navigator_node.py`) in
Gazebo Harmonic with ros2_control, a simulated LiDAR and the diff-drive controller.
One run each in the same high-density world (24 obstacles): adaptive reached the goal in
32.6 s, fixed 1.0 m/s in 15.2 s, both without collision. This agrees with the Python averages
(35.9 s and 15.2 s). It is a single run, so it demonstrates the pipeline; the statistics come
from the 90-world experiment.

## 6. Limitations
- Gazebo was run once per policy (low and high worlds), so physics effects such as wheel slip
  and controller lag are only lightly tested.
- Thresholds were tuned on seeds 0–14, which are included in the 30 evaluation worlds, so the
  adaptive numbers are slightly optimistic. Re-test on fresh seeds before claiming more.
- Static circular obstacles only; no moving obstacles.
- 30 worlds per density is a modest sample; differences of a few percent are not significant.

## 7. Possible extensions
Tune `d_slow` / `density_gain` systematically; add moving obstacles; replace steering with a
local planner to remove timeouts; compare against ROS 2 Nav2's velocity smoother.
