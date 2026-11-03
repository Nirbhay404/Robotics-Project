"""Reactive gap-following steering (no ROS imports).

Idea: among all LiDAR directions pick the one that is both FREE (far to the nearest
obstacle, checked over a window as wide as the robot) and CLOSE to the goal direction.

    score(i) = free_i / cap  -  k_goal * |angle_i - goal_bearing| / pi  +  hysteresis

The robot turns toward the best direction. Speed is NOT decided here; that is the job
of speed_policy.py, so fixed and adaptive speed can be compared fairly.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class SteeringParams:
    safe_radius: float = 0.50   # m, half-width a free corridor must have (robot radius + margin)
    cap: float = 3.0            # m, free distance beyond this is treated as equally good
    k_goal: float = 0.9         # weight of goal alignment
    hysteresis: float = 0.15    # bonus for keeping the previous direction (anti-dither)
    k_w: float = 2.5            # heading error -> angular velocity gain
    w_max: float = 1.5          # rad/s


class Steering:
    def __init__(self, params: SteeringParams = None):
        self.p = params or SteeringParams()
        self.prev = 0.0

    def reset(self):
        self.prev = 0.0

    def compute(self, ranges, angles, goal_bearing):
        """Return angular velocity (rad/s); goal_bearing is the goal angle in the robot frame."""
        p = self.p
        n = len(ranges)

        # A beam j blocks direction i if the obstacle it sees is within safe_radius of the
        # straight line along i, i.e. |angle_i - angle_j| < asin(safe_radius / range_j).
        # So close obstacles block a wide cone, far ones a narrow cone.
        lim = np.arcsin(np.clip(p.safe_radius / np.maximum(ranges, 1e-3), 0.0, 1.0))
        delta = np.abs((angles[:, None] - angles[None, :] + np.pi) % (2 * np.pi) - np.pi)
        blocked = delta <= lim[None, :]
        free = np.where(blocked, ranges[None, :], np.inf).min(axis=1)
        free = np.where(np.isinf(free), ranges.max(), free)

        def wrap(a):
            return (a + np.pi) % (2 * np.pi) - np.pi

        score = (np.minimum(free, p.cap) / p.cap
                 - p.k_goal * np.abs(wrap(angles - goal_bearing)) / np.pi
                 + p.hysteresis * np.exp(-np.abs(wrap(angles - self.prev))))
        best = angles[int(np.argmax(score))]
        self.prev = float(best)
        return float(np.clip(p.k_w * best, -p.w_max, p.w_max))
