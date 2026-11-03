"""Speed policies for a mobile robot with a 2-D LiDAR (no ROS imports).

Two policies are compared in this project:

  FixedSpeed     always commands v_max (the "conventional" baseline)
  AdaptiveSpeed  v = v_max * proximity_factor * density_factor * turn_factor

  proximity_factor  falls from 1 to 0 as the closest obstacle in the forward cone
                    goes from d_slow down to d_stop            (obstacle proximity)
  density_factor    falls as more of the 360 deg scan is "occupied" within
                    density_radius                              (environment complexity)
  turn_factor       slows the robot while it is turning sharply (smoothness)

A final rate limiter keeps the commanded speed smooth.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class SpeedParams:
    v_max: float = 1.0          # m/s, cruise speed in open space
    v_min: float = 0.25         # m/s, creep speed (never stall unless blocked)
    d_stop: float = 0.55        # m, below this in the forward cone -> stop
    d_slow: float = 1.8         # m, above this the proximity factor is 1
    cone_deg: float = 40.0      # half-angle of the forward cone
    density_radius: float = 2.0  # m, range that counts as "near" for density
    density_ref: float = 0.45   # fraction of near beams that counts as fully cluttered
    density_gain: float = 0.4   # how strongly clutter reduces speed (0..1)
    turn_gain: float = 0.3      # how strongly turning reduces speed (0..1)
    w_max: float = 1.5          # rad/s, used to normalise the turn factor
    a_up: float = 1.0           # m/s^2 speed-up limit
    a_down: float = 3.0         # m/s^2 slow-down limit


class FixedSpeed:
    name = "fixed"

    def __init__(self, v: float):
        self.v = v
        self.name = f"fixed_{v:g}"

    def reset(self):
        pass

    def compute(self, ranges, angles, w_cmd, dt):
        return self.v, {}


class AdaptiveSpeed:
    name = "adaptive"

    def __init__(self, params: SpeedParams = None):
        self.p = params or SpeedParams()
        self.v_prev = 0.0

    def reset(self):
        self.v_prev = 0.0

    def factors(self, ranges, angles, w_cmd):
        p = self.p
        cone = np.abs(angles) <= np.deg2rad(p.cone_deg)
        d_front = float(np.min(ranges[cone])) if np.any(cone) else float(np.min(ranges))

        prox = np.clip((d_front - p.d_stop) / (p.d_slow - p.d_stop), 0.0, 1.0)
        clutter = float(np.mean(ranges < p.density_radius))
        dens = 1.0 - p.density_gain * min(clutter / p.density_ref, 1.0)
        turn = 1.0 - p.turn_gain * min(abs(w_cmd) / p.w_max, 1.0)
        return d_front, prox, dens, turn, clutter

    def compute(self, ranges, angles, w_cmd, dt):
        p = self.p
        d_front, prox, dens, turn, clutter = self.factors(ranges, angles, w_cmd)

        if d_front <= p.d_stop:
            v_target = 0.0
        else:
            v_target = float(np.clip(p.v_max * prox * dens * turn, p.v_min, p.v_max))

        # rate limit for smooth acceleration
        dv = v_target - self.v_prev
        dv = np.clip(dv, -p.a_down * dt, p.a_up * dt)
        self.v_prev = float(max(self.v_prev + dv, 0.0))

        return self.v_prev, {"d_front": d_front, "prox": prox, "dens": dens,
                             "turn": turn, "clutter": clutter, "v_target": v_target}
