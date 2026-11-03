#!/usr/bin/env python3
"""Generate a Gazebo SDF world with the SAME random obstacles as python_sim.

    python3 generate_world.py                # writes low / medium / high into ../worlds
    python3 generate_world.py medium 5       # one density, one seed
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "..", "python_sim"))
import sim  # noqa: E402

HEADER = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="cluttered_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material></visual>
      </link>
    </model>
"""

WALL = """    <model name="{n}"><static>true</static><pose>{x} {y} 0.5 0 0 0</pose><link name="link">
      <collision name="c"><geometry><box><size>{sx} {sy} 1</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>{sx} {sy} 1</size></box></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>
    </link></model>
"""

CYL = """    <model name="obstacle_{i}"><static>true</static><pose>{x:.3f} {y:.3f} 0.5 0 0 0</pose><link name="link">
      <collision name="c"><geometry><cylinder><radius>{r:.3f}</radius><length>1</length></cylinder></geometry></collision>
      <visual name="v"><geometry><cylinder><radius>{r:.3f}</radius><length>1</length></cylinder></geometry>
        <material><ambient>0.8 0.4 0.1 1</ambient><diffuse>0.8 0.4 0.1 1</diffuse></material></visual>
    </link></model>
"""


def build(density, seed):
    obs = sim.make_world(sim.DENSITIES[density], seed)
    h = sim.ARENA / 2
    out = [HEADER]
    out.append(WALL.format(n="wall_n", x=0, y=h + 0.05, sx=sim.ARENA + 0.2, sy=0.1))
    out.append(WALL.format(n="wall_s", x=0, y=-h - 0.05, sx=sim.ARENA + 0.2, sy=0.1))
    out.append(WALL.format(n="wall_e", x=h + 0.05, y=0, sx=0.1, sy=sim.ARENA))
    out.append(WALL.format(n="wall_w", x=-h - 0.05, y=0, sx=0.1, sy=sim.ARENA))
    for i, (x, y, r) in enumerate(obs):
        out.append(CYL.format(i=i, x=x, y=y, r=r))
    out.append("  </world>\n</sdf>\n")
    return "".join(out)


def main():
    wdir = os.path.join(HERE, "..", "worlds")
    os.makedirs(wdir, exist_ok=True)
    if len(sys.argv) >= 3:
        jobs = [(sys.argv[1], int(sys.argv[2]))]
    else:
        jobs = [(d, 0) for d in sim.DENSITIES]
    for dens, seed in jobs:
        path = os.path.join(wdir, f"cluttered_{dens}.sdf")
        with open(path, "w") as f:
            f.write(build(dens, seed))
        print("wrote", os.path.normpath(path))


if __name__ == "__main__":
    main()
