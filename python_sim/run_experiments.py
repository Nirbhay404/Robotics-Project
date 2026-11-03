"""Run fixed-speed vs adaptive-speed navigation over many random worlds.

    python run_experiments.py            # 30 seeds per density (default)
    python run_experiments.py --seeds 10

Writes results.csv, summary.md and plots into ./outputs
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import sim  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "outputs")
COLORS = {"fixed_0.3": "#4c9be8", "fixed_1": "#e8704c", "adaptive": "#3aa655"}
LABELS = {"fixed_0.3": "Fixed 0.3 m/s", "fixed_1": "Fixed 1.0 m/s", "adaptive": "Adaptive"}


def run_all(n_seeds):
    rows = []
    for dens, n_obs in sim.DENSITIES.items():
        for seed in range(n_seeds):
            for pol in sim.make_policies():
                r = sim.run_episode(pol, n_obs, seed)
                rows.append(dict(density=dens, seed=seed, policy=pol.name,
                                 success=int(r.success), collided=int(r.collided),
                                 time=r.time, path=r.path_length, mean_speed=r.mean_speed,
                                 min_clear=r.min_clearance, mean_clear=r.mean_clearance,
                                 jerk=r.speed_jerk))
        print(f"done density={dens}")
    return rows


def summarise(rows):
    pols = list(COLORS)
    out = {}
    for dens in sim.DENSITIES:
        for pol in pols:
            sel = [r for r in rows if r["density"] == dens and r["policy"] == pol]
            ok = [r for r in sel if r["success"]]
            out[(dens, pol)] = dict(
                n=len(sel),
                success=100 * np.mean([r["success"] for r in sel]),
                collision=100 * np.mean([r["collided"] for r in sel]),
                time=np.mean([r["time"] for r in ok]) if ok else float("nan"),
                speed=np.mean([r["mean_speed"] for r in sel]),
                min_clear=np.mean([r["min_clear"] for r in sel]),
                mean_clear=np.mean([r["mean_clear"] for r in sel]),
                jerk=np.mean([r["jerk"] for r in sel]))
    return out


def write_summary(summ, n_seeds):
    lines = [f"# Results ({n_seeds} random worlds per density, same worlds for every policy)\n",
             "Time is averaged over successful runs only. Clearance = gap between robot edge and nearest obstacle (m).\n",
             "| Density | Policy | Success % | Collision % | Time (s) | Mean speed | Min clearance | Mean clearance | Accel (m/s²) |",
             "|---|---|---|---|---|---|---|---|---|"]
    for (dens, pol), s in summ.items():
        lines.append(f"| {dens} | {LABELS[pol]} | {s['success']:.0f} | {s['collision']:.0f} | "
                     f"{s['time']:.1f} | {s['speed']:.2f} | {s['min_clear']:.2f} | "
                     f"{s['mean_clear']:.2f} | {s['jerk']:.2f} |")
    with open(os.path.join(OUT, "summary.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


def bar_plots(summ):
    metrics = [("success", "Success rate (%)"), ("collision", "Collision rate (%)"),
               ("time", "Completion time (s, successful runs)"),
               ("mean_clear", "Mean obstacle clearance (m)")]
    fig, axs = plt.subplots(1, 4, figsize=(18, 4))
    dens_list = list(sim.DENSITIES)
    x = np.arange(len(dens_list))
    for ax, (key, title) in zip(axs, metrics):
        for i, pol in enumerate(COLORS):
            vals = [summ[(d, pol)][key] for d in dens_list]
            ax.bar(x + (i - 1) * 0.27, vals, 0.27, label=LABELS[pol], color=COLORS[pol])
        ax.set_xticks(x); ax.set_xticklabels([f"{d}\n({sim.DENSITIES[d]} obs)" for d in dens_list])
        ax.set_title(title); ax.grid(axis="y", alpha=0.3)
    axs[0].set_ylim(0, 125)
    axs[0].legend(loc="upper center", ncol=3, fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "comparison.png"), dpi=120); plt.close(fig)


def example_episode(seed, dens="high"):
    n = sim.DENSITIES[dens]
    res = {p.name: sim.run_episode(p, n, seed, record=True) for p in sim.make_policies()}
    fig, axs = plt.subplots(1, 2, figsize=(15, 6.5))
    ax = axs[0]
    obs = next(iter(res.values())).log["obs"]
    for o in obs:
        ax.add_patch(plt.Circle(o[:2], o[2], color="gray", alpha=0.7))
    ax.plot(*sim.START, "ko"); ax.plot(*sim.GOAL, "g*", ms=18)
    for name, r in res.items():
        tag = "COLLISION" if r.collided else ("goal" if r.success else "timeout")
        ax.plot(r.log["x"], r.log["y"], color=COLORS[name], lw=2,
                label=f"{LABELS[name]} ({tag}, {r.time:.1f}s)")
        if r.collided:
            ax.plot(r.log["x"][-1], r.log["y"][-1], "x", color=COLORS[name], ms=14, mew=3)
    ax.set_xlim(-6, 6); ax.set_ylim(-6, 6); ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_title(f"Same {dens}-density world (seed {seed})"); ax.legend(loc="upper left")
    ax = axs[1]
    for name, r in res.items():
        ax.plot(r.log["t"], r.log["v"], color=COLORS[name], lw=2, label=LABELS[name])
    ax.set_xlabel("time (s)"); ax.set_ylabel("speed (m/s)"); ax.grid(alpha=0.3)
    ax.set_title("Speed over time"); ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(OUT, f"example_{dens}_seed{seed}.png"), dpi=120); plt.close(fig)


def adaptive_anatomy(seed=3, dens="medium"):
    r = sim.run_episode(sim.AdaptiveSpeed(), sim.DENSITIES[dens], seed, record=True)
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    t = r.log["t"]
    axs[0].plot(t, r.log["d_front"]); axs[0].set_ylabel("front distance (m)")
    axs[1].plot(t, r.log["clutter"]); axs[1].set_ylabel("clutter (0..1)")
    axs[2].plot(t, r.log["v"], color="#3aa655"); axs[2].set_ylabel("speed (m/s)")
    axs[2].set_xlabel("time (s)")
    for a in axs:
        a.grid(alpha=0.3)
    axs[0].set_title("Why the adaptive robot slows down: proximity and clutter drive speed")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "adaptive_anatomy.png"), dpi=120); plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=30)
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    rows = run_all(args.seeds)
    with open(os.path.join(OUT, "results.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    summ = summarise(rows)
    write_summary(summ, args.seeds)
    bar_plots(summ)
    for seed in (1, 2, 4):
        example_episode(seed, "high")
    adaptive_anatomy()
    print(f"outputs in {OUT}")


if __name__ == "__main__":
    main()
