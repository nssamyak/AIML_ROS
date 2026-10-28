"""Module 4: global path planning. Costmap + Dijkstra vs A* vs greedy A*.

    python 04_path_planning/demo.py
    python 04_path_planning/demo.py --goal 1.5 -1.1
    python 04_path_planning/demo.py --start 0.3 1.2 --goal 1.5 -1.1     # greedy A* is not optimal
    python 04_path_planning/demo.py --inflation 0.1                     # hug the walls
    python 04_path_planning/demo.py --animate                           # watch the search grow
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from hbai import viz
from hbai.gridmap import GridGeom
from hbai.planning import costmap, plan, path_length
from hbai.world import START, arena

p = viz.parser(__doc__)
p.add_argument('--start', type=float, nargs=2, default=list(START[:2]))
p.add_argument('--goal', type=float, nargs=2, default=[1.65, 0.55])
p.add_argument('--inflation', type=float, default=0.25, help='inflation radius (m)')
p.add_argument('--greedy-weight', type=float, default=3.0, help='heuristic weight for greedy A*')
p.add_argument('--animate', action='store_true')
args, plt = viz.init(p, '04_path_planning')

world = arena()
geom = GridGeom(world.xlim, world.ylim, 0.05)
cost = costmap(world.occupancy(geom), geom.res, inflation_radius=args.inflation)

runs = []
for name, weight in [('Dijkstra (h = 0)', 0.0), ('A* (h = distance)', 1.0),
                     (f'greedy A* (h x {args.greedy_weight:g})', args.greedy_weight)]:
    t0 = time.perf_counter()
    path, expanded = plan(geom, cost, args.start, args.goal, heuristic_weight=weight)
    ms = 1000 * (time.perf_counter() - t0)
    runs.append((name, path, expanded, ms))
    print(f'{name:22s} expanded {len(expanded):5d} cells, path {path_length(path):.2f} m, {ms:.0f} ms')


def expansion_image(expanded, upto=None):
    img = np.full(geom.shape, np.nan)
    cells = np.array(expanded[:upto]) if expanded else np.zeros((0, 2), int)
    if len(cells):
        img[cells[:, 0], cells[:, 1]] = np.arange(len(cells))
    return img


fig, axes = plt.subplots(2, 2, figsize=(13, 9))
fig.suptitle('Module 4: path planning on a costmap', fontsize=14)

ax = axes[0, 0]
shown = np.where(np.isfinite(cost), cost, np.nan)
im = ax.imshow(np.log10(shown), origin='lower', extent=geom.extent, cmap='magma_r', interpolation='nearest')
ax.imshow(np.where(np.isfinite(cost), np.nan, 1.0), origin='lower', extent=geom.extent, cmap='Greys',
          vmin=0, vmax=1, interpolation='nearest')
fig.colorbar(im, ax=ax, fraction=0.035, label='log10(cost)')
viz.frame(ax)
ax.set_title('costmap: black = lethal (robot would touch),\ncolour = inflation (expensive near walls)', fontsize=10)

for ax, (name, path, expanded, ms) in zip(axes.flat[1:], runs):
    viz.draw_world(ax, world, color='0.2')
    ax.imshow(expansion_image(expanded), origin='lower', extent=geom.extent, cmap='viridis',
              interpolation='nearest', alpha=0.85)
    if path is not None:
        ax.plot(path[:, 0], path[:, 1], 'r-', lw=2.5)
    ax.plot(*args.start, 'wo', mec='k', ms=9)
    ax.plot(*args.goal, 'r*', mec='k', ms=16)
    viz.frame(ax)
    ax.set_title(f'{name}\n{len(expanded)} cells expanded, path {path_length(path):.2f} m, {ms:.0f} ms',
                 fontsize=10)
fig.tight_layout()
viz.finish(fig, args, 'planning_comparison')

if args.animate and not args.fast:
    fig2, axes2 = plt.subplots(1, 2, figsize=(12, 4.8))
    anim = viz.Animator(fig2, args, 'search_wavefront', fps=15)
    longest = max(len(runs[0][2]), len(runs[1][2]))
    for upto in list(range(0, longest, max(1, longest // 80))) + [longest]:
        for ax, (name, path, expanded, _) in zip(axes2, runs[:2]):
            ax.clear()
            viz.draw_world(ax, world, color='0.2')
            ax.imshow(expansion_image(expanded, upto), origin='lower', extent=geom.extent, cmap='viridis',
                      vmin=0, vmax=longest, interpolation='nearest')
            if upto >= len(expanded) and path is not None:
                ax.plot(path[:, 0], path[:, 1], 'r-', lw=2.5)
            ax.plot(*args.goal, 'r*', mec='k', ms=14)
            ax.set_title(f'{name}: {min(upto, len(expanded))} cells')
        anim.frame()
    anim.close()

viz.show(args)
