# Adaptive Speed Control in Cluttered Environments

Robotics and Intelligent Systems project, problem statement 5.

**Team:** Nirbhay Kumar (24BCS089), Nikhil Kumar Sinha (24BCS086), Naman Kumar (24BCS080)

A differential-drive robot with a 2-D LiDAR drives to a goal through a cluttered arena. A speed
policy that adapts to obstacle proximity and clutter is compared with fixed-speed navigation.
Full write-up: [docs/REPORT.md](docs/REPORT.md).

```
python_sim/                       runs on your Mac, no ROS needed
  sim.py                          world, noisy LiDAR, robot dynamics, one-episode runner
  run_experiments.py              90 worlds x 3 policies -> outputs/ (plots, csv, summary.md)
ros2_ws/src/adaptive_speed_nav/   the ROS 2 Jazzy package (same algorithm)
  adaptive_speed_nav/speed_policy.py   THE contribution: fixed vs adaptive speed
  adaptive_speed_nav/steering.py       gap-following steering (same for every policy)
  adaptive_speed_nav/navigator_node.py ROS 2 node (/scan, odom -> cmd_vel)
  urdf/ config/ worlds/ launch/ tools/ robot, controllers, Gazebo worlds, launch, world generator
docker/, docker-compose.yml       ROS 2 + Gazebo + RViz in a browser desktop
docs/REPORT.md                    method, results, how to explain it
```

`speed_policy.py` and `steering.py` have no ROS imports. The simulator and the ROS node both
import these same two files, so the results from the simulator describe the ROS algorithm.

## 1. Run the simulation on your Mac (needs only Python)
```bash
pip3 install -r requirements.txt
cd python_sim
python3 run_experiments.py            # about 10 s; use --seeds 10 for a quicker run
```
Outputs go to `python_sim/outputs/`: `summary.md`, `results.csv`, `comparison.png`,
`example_high_seed2.png`, `adaptive_anatomy.png`.

## 2. Run the real ROS 2 + Gazebo simulation (Docker)
Start Docker Desktop first, then:
```bash
docker compose up -d --build         # first build is large (several GB, 10-20 min)
```
Open http://localhost:6080, open a terminal in that desktop and run:
```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch adaptive_speed_nav simulation.launch.py density:=low mode:=adaptive
```
Compare by changing `mode:=fixed fixed_speed:=1.0` (or `0.3`) and `density:=low|medium|high`.
Add `log_csv:=/tmp/run.csv` to record a run. Watch it in RViz (`rviz2`, Fixed Frame `odom`,
add RobotModel, TF, LaserScan `/scan`). Debug topic: `ros2 topic echo /adaptive_speed/debug`.

Gazebo in Docker on a Mac uses software rendering, so it is slow. The controllers start about
15 s after launch; the navigator waits for them. Worlds are regenerated with
`python3 ros2_ws/src/adaptive_speed_nav/tools/generate_world.py <density> <seed>`.

## Status
- Python simulation and 90-world experiments: run and verified (see the report).
- ROS 2 + Gazebo + Docker: run and verified on the `low` and `high` worlds. The robot drives
  from (-5,-5) to the far corner and the navigator prints `Goal reached in X s`.
  High-density world, one run each: adaptive 32.6 s, fixed 1.0 m/s 15.2 s.
- Gazebo needs a fresh start between runs (`docker compose restart ros`), because odometry
  only resets with a new Gazebo process.
- Fixes found while running it for real (all already applied):
  1. wheel joint axis in the URDF must be `0 0 -1`, not `0 1 0` (the doc's value makes the wheels spin flat);
  2. Jazzy's diff_drive_controller takes `TwistStamped` on `/diff_drive_controller/cmd_vel`;
  3. the node subscribes with sensor-data QoS; 4. `setup.cfg` is needed so `ros2 run` finds the node.
