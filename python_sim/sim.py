"""2-D simulator: unicycle robot + noisy 2-D LiDAR + random cluttered arena.

It uses the exact same speed_policy.py / steering.py as the ROS 2 node.
"""
import os
import sys
from dataclasses import dataclass, field

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src",
                                "adaptive_speed_nav", "adaptive_speed_nav"))
from speed_policy import AdaptiveSpeed, FixedSpeed, SpeedParams  # noqa: E402
from steering import Steering  # noqa: E402

ARENA = 12.0                 # arena is [-6, 6]^2
START = np.array([-5.0, -5.0])
GOAL = np.array([5.0, 5.0])
GOAL_TOL = 0.4
ROBOT_RADIUS = 0.30          # collision radius (chassis 0.6 x 0.4 m)
DT = 0.05                    # 20 Hz control + LiDAR
N_BEAMS = 120
LIDAR_RANGE = 6.0
LIDAR_NOISE = 0.02           # std dev, m
V_ACC, V_BRAKE = 1.5, 3.0    # physical limits of the robot, m/s^2
T_MAX = 90.0

DENSITIES = {"low": 6, "medium": 14, "high": 24}


def make_world(n_obstacles, seed):
    """Random circular obstacles, keeping start and goal areas clear."""
    rng = np.random.default_rng(seed)
    obs = []
    while len(obs) < n_obstacles:
        c = rng.uniform(-5.4, 5.4, 2)
        r = rng.uniform(0.3, 0.55)
        if np.linalg.norm(c - START) < 1.5 + r or np.linalg.norm(c - GOAL) < 1.5 + r:
            continue
        if any(np.linalg.norm(c - o[:2]) < r + o[2] + 0.9 for o in obs):
            continue                      # keep gaps wide enough for the robot
        obs.append(np.array([c[0], c[1], r]))
    return np.array(obs)


BEAM_ANGLES = np.linspace(-np.pi, np.pi, N_BEAMS, endpoint=False)


def lidar_scan(x, y, th, obs, rng):
    """Vectorised ray casting against circles and the arena walls."""
    ang = th + BEAM_ANGLES
    dx, dy = np.cos(ang), np.sin(ang)
    rng_out = np.full(N_BEAMS, LIDAR_RANGE)
    half = ARENA / 2
    with np.errstate(divide="ignore", invalid="ignore"):
        for o, d, lim in ((x, dx, half), (y, dy, half)):
            t = np.where(d > 0, (lim - o) / d, np.where(d < 0, (-lim - o) / d, np.inf))
            rng_out = np.minimum(rng_out, t)
    if len(obs):
        fx, fy = x - obs[:, 0], y - obs[:, 1]                 # (M,)
        b = dx[:, None] * fx[None, :] + dy[:, None] * fy[None, :]
        c = (fx ** 2 + fy ** 2 - obs[:, 2] ** 2)[None, :]
        disc = b * b - c
        t = np.where(disc >= 0, -b - np.sqrt(np.maximum(disc, 0)), np.inf)
        t = np.where(t > 0, t, np.inf)
        rng_out = np.minimum(rng_out, t.min(axis=1))
    rng_out = np.minimum(rng_out, LIDAR_RANGE)
    rng_out = rng_out + rng.normal(0, LIDAR_NOISE, N_BEAMS)
    return np.clip(rng_out, 0.05, LIDAR_RANGE)


def clearance(x, y, obs):
    """Distance from the robot edge to the nearest obstacle or wall."""
    half = ARENA / 2
    d = min(half - abs(x), half - abs(y)) - ROBOT_RADIUS
    if len(obs):
        d = min(d, float(np.min(np.hypot(obs[:, 0] - x, obs[:, 1] - y) - obs[:, 2])) - ROBOT_RADIUS)
    return d


@dataclass
class Result:
    success: bool
    collided: bool
    time: float
    path_length: float
    mean_speed: float
    min_clearance: float
    mean_clearance: float
    speed_jerk: float             # mean |dv/dt| -> smoothness
    log: dict = field(default_factory=dict)


def run_episode(policy, n_obstacles, seed, record=False):
    obs = make_world(n_obstacles, seed)
    rng = np.random.default_rng(seed + 10_000)
    policy.reset()
    steering = Steering()
    x, y = START
    th = np.arctan2(GOAL[1] - y, GOAL[0] - x)
    v = 0.0
    t = 0.0
    path = 0.0
    speeds, clears = [], []
    log = {k: [] for k in ("t", "x", "y", "th", "v", "clear", "d_front", "clutter")}
    collided = success = False

    while t < T_MAX:
        scan = lidar_scan(x, y, th, obs, rng)
        bearing = np.arctan2(GOAL[1] - y, GOAL[0] - x) - th
        bearing = (bearing + np.pi) % (2 * np.pi) - np.pi
        w = steering.compute(scan, BEAM_ANGLES, bearing)
        v_cmd, info = policy.compute(scan, BEAM_ANGLES, w, DT)

        # robot dynamics: speed follows the command within physical accel limits
        dv = np.clip(v_cmd - v, -V_BRAKE * DT, V_ACC * DT)
        v_new = max(v + dv, 0.0)
        speeds.append(v_new)
        th += w * DT
        x += v_new * np.cos(th) * DT
        y += v_new * np.sin(th) * DT
        path += v_new * DT
        v = v_new
        t += DT

        c = clearance(x, y, obs)
        clears.append(c)
        if record:
            log["t"].append(t); log["x"].append(x); log["y"].append(y)
            log["th"].append(th); log["v"].append(v); log["clear"].append(c)
            log["d_front"].append(info.get("d_front", float(np.min(scan))))
            log["clutter"].append(info.get("clutter", float(np.mean(scan < 2.0))))
        if c <= 0.0:
            collided = True
            break
        if np.hypot(GOAL[0] - x, GOAL[1] - y) < GOAL_TOL:
            success = True
            break

    speeds = np.array(speeds)
    jerk = float(np.mean(np.abs(np.diff(speeds)) / DT)) if len(speeds) > 1 else 0.0
    return Result(success, collided, t, path, float(speeds.mean()),
                  float(np.min(clears)), float(np.mean(clears)), jerk,
                  {**log, "obs": obs} if record else {"obs": obs})


def make_policies():
    return [FixedSpeed(0.3), FixedSpeed(1.0), AdaptiveSpeed(SpeedParams())]
