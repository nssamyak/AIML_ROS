"""Module 6: autonomous exploration. Everything together, live.

    python 06_frontier_exploration/demo.py
    python 06_frontier_exploration/demo.py --slam                # drifting odometry + scan matching
    python 06_frontier_exploration/demo.py --controller simple   # no MPPI: straight at the path
    python 06_frontier_exploration/demo.py --size-weight 3       # prefer big frontiers
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from hbai import viz
from hbai.exploration import Explorer, ExploreConfig
from hbai.world import START, arena

p = viz.parser(__doc__)
p.add_argument('--controller', choices=['mppi', 'simple'], default='mppi')
p.add_argument('--slam', action='store_true', help='map with odometry + scan matching instead of the true pose')
p.add_argument('--size-weight', type=float, default=0.5)
p.add_argument('--distance-weight', type=float, default=1.0)
p.add_argument('--min-frontier-size', type=float, default=0.25)
p.add_argument('--draw-every', type=float, default=1.0, help='seconds of simulated time between frames')
args, plt = viz.init(p, '06_frontier_exploration')

cfg = ExploreConfig(controller=args.controller, use_slam=args.slam, size_weight=args.size_weight,
                    distance_weight=args.distance_weight, min_frontier_size=args.min_frontier_size,
                    max_time=40.0 if args.fast else 900.0)
world = arena()
ex = Explorer(world, START, cfg, seed=args.seed)

fig, (ax, ax_cov) = plt.subplots(1, 2, figsize=(14, 5.6), gridspec_kw=dict(width_ratios=[1.6, 1]))
anim = viz.Animator(fig, args, 'exploration', fps=8)
every = max(1, int(round(args.draw_every / cfg.dt)))
printed = 0


def draw(e):
    ax.clear()
    viz.show_map(ax, e.grid.to_ros(), e.geom)
    traj = np.array(e.trajectory)
    ax.plot(traj[:, 0], traj[:, 1], color='tab:blue', lw=0.8, alpha=0.6)
    for f in e.frontiers:
        ax.scatter(f.cells[:, 0], f.cells[:, 1], s=2, c='deepskyblue', zorder=3)
        ax.plot(f.x, f.y, 'o', color='orange', mec='k', ms=6, zorder=4)
    if e.blacklist:
        b = np.array(e.blacklist)
        ax.plot(b[:, 0], b[:, 1], 'x', color='red', ms=6, mew=1.5, zorder=4)
    if e.path is not None and e.state in ('navigating', 'returning home'):
        ax.plot(e.path[:, 0], e.path[:, 1], 'g-', lw=1.5, zorder=4)
        ax.plot(*e.goal, '*', color='lime', mec='k', ms=14, zorder=6)
    if args.slam:
        viz.draw_robot(ax, e.pose, color='magenta')          # where it thinks it is
    viz.draw_robot(ax, e.true_pose)
    viz.frame(ax)
    ax.set_title(f't = {e.t:5.1f} s   {e.state}   goals: {e.goals_sent}   '
                 f'coverage {e.coverage():.0%}', fontsize=10)

    ax_cov.clear()
    if e.coverage_history:
        h = np.array(e.coverage_history)
        ax_cov.plot(h[:, 0], h[:, 1] * 100, 'b-')
    ax_cov.set_ylim(0, 102)
    ax_cov.set_xlabel('time (s)')
    ax_cov.set_ylabel('% of the arena mapped')
    ax_cov.set_title('coverage   (cyan = frontier, orange = candidate,\n'
                     'green star = goal, red x = already tried)', fontsize=9)
    ax_cov.grid(alpha=0.3)


t0 = time.perf_counter()
while not ex.finished:
    ex.step()
    for when, text in ex.events[printed:]:
        print(f'[{when:6.1f} s] {text}')
    printed = len(ex.events)
    if ex.steps % every == 0:
        draw(ex)
        anim.frame()
draw(ex)
anim.frame()
anim.close()

print(f'finished: {ex.state}, {ex.t:.0f} s simulated in {time.perf_counter() - t0:.0f} s, '
      f'coverage {ex.coverage():.1%}, {ex.goals_sent} goals, {ex.bumps} bumps')
fig.tight_layout()
viz.finish(fig, args, 'exploration_final')
viz.show(args)
