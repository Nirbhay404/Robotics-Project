"""Watch the robot move: fixed 1.0 vs adaptive in the same world, side by side.

    python3 animate.py                       # saves outputs/animation.gif
    python3 animate.py --show                # also opens a live window
    python3 animate.py --density medium --seed 5
"""
import argparse
import os

import matplotlib
import numpy as np

import sim


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--density", default="high", choices=list(sim.DENSITIES))
    ap.add_argument("--seed", type=int, default=2)
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    n = sim.DENSITIES[args.density]
    pols = [sim.FixedSpeed(1.0), sim.AdaptiveSpeed()]
    titles = ["Fixed 1.0 m/s", "Adaptive speed"]
    runs = [sim.run_episode(p, n, args.seed, record=True) for p in pols]
    obs = runs[0].log["obs"]

    fig, axs = plt.subplots(1, 2, figsize=(12, 6.3))
    arts = []
    for ax, title in zip(axs, titles):
        for o in obs:
            ax.add_patch(plt.Circle(o[:2], o[2], color="gray", alpha=0.7))
        ax.plot(*sim.START, "ko"); ax.plot(*sim.GOAL, "g*", ms=18)
        ax.set_xlim(-6, 6); ax.set_ylim(-6, 6); ax.set_aspect("equal"); ax.grid(alpha=0.3)
        trail, = ax.plot([], [], "-", color="#3a7bd5", lw=1.5)
        body = plt.Circle((0, 0), sim.ROBOT_RADIUS, color="#e8704c")
        ax.add_patch(body)
        head, = ax.plot([], [], "k-", lw=2)
        txt = ax.set_title(title)
        arts.append((trail, body, head, txt, title))

    skip = 2                                   # 0.1 s per frame -> 10 fps real time
    n_frames = max(len(r.log["t"]) for r in runs) // skip + 1

    def draw(f):
        i = f * skip
        out = []
        for r, (trail, body, head, txt, title) in zip(runs, arts):
            L = len(r.log["t"])
            j = min(i, L - 1)
            x, y, th = r.log["x"][j], r.log["y"][j], r.log["th"][j]
            trail.set_data(r.log["x"][: j + 1], r.log["y"][: j + 1])
            body.center = (x, y)
            head.set_data([x, x + 0.5 * np.cos(th)], [y, y + 0.5 * np.sin(th)])
            end = ""
            if j == L - 1:
                end = "  COLLISION" if r.collided else ("  GOAL" if r.success else "  TIMEOUT")
            txt.set_text(f"{title}   t={r.log['t'][j]:.1f}s  v={r.log['v'][j]:.2f} m/s{end}")
            out += [trail, body, head]
        return out

    anim = FuncAnimation(fig, draw, frames=n_frames, interval=100, blit=False)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "outputs")
    os.makedirs(out, exist_ok=True)
    gif = os.path.join(out, "animation.gif")
    anim.save(gif, writer=PillowWriter(fps=10))
    print("saved", gif)
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
