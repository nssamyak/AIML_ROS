"""Module 2: building a map from scans with a Bayes filter (log-odds occupancy grid).

The robot's pose is known exactly here (module 3 removes that cheat).

    python 02_occupancy_mapping/demo.py
    python 02_occupancy_mapping/demo.py --l-free -0.1     # weaker 'free' evidence: what changes?
    python 02_occupancy_mapping/demo.py --animate
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from hbai import viz
from hbai.gridmap import GridGeom
from hbai.lidar import Lidar
from hbai.mapping import OccupancyGrid
from hbai.tour import TOUR, tour_poses
from hbai.world import arena

p = viz.parser(__doc__)
p.add_argument('--res', type=float, default=0.05, help='map resolution (m per cell)')
p.add_argument('--l-occ', type=float, default=0.85, help='log-odds added for a hit')
p.add_argument('--l-free', type=float, default=-0.4, help='log-odds added for a pass-through')
p.add_argument('--noise', type=float, default=0.01, help='lidar range noise (m)')
p.add_argument('--animate', action='store_true')
args, plt = viz.init(p, '02_occupancy_mapping')

world = arena()
poses = tour_poses(world, TOUR[:3] if args.fast else TOUR)
geom = GridGeom(world.xlim, world.ylim, args.res)
grid = OccupancyGrid(geom, l_occ=args.l_occ, l_free=args.l_free)
lidar = Lidar(noise_std=args.noise, seed=args.seed)

# Two cells to follow: one on a wall, one in open space
wall_cell = geom.to_cell(np.array([-0.6, 0.3]))
free_cell = geom.to_cell(np.array([-1.0, 0.2]))
history = {'wall': [], 'free': []}

snap_at = np.linspace(0, len(poses) - 1, 5).astype(int)[1:-1]     # 3 snapshots + the final map
snapshots = []
anim = None
if args.animate and not args.fast:
    fig_a, ax_a = plt.subplots(figsize=(8, 6))
    anim = viz.Animator(fig_a, args, 'mapping', fps=15)

for k, pose in enumerate(poses):
    if k % 2:
        continue                                     # map at 5 Hz
    grid.update(pose, lidar.angles, lidar.scan(world, pose))
    prob = grid.probability()
    history['wall'].append(prob[wall_cell])
    history['free'].append(prob[free_cell])
    if any(abs(k - s) <= 1 for s in snap_at) and len(snapshots) < 3:
        snapshots.append((k, grid.to_ros(), poses[:k + 1].copy()))
    if anim is not None and k % 6 == 0:
        ax_a.clear()
        viz.show_map(ax_a, grid.to_ros(), geom)
        ax_a.plot(poses[:k + 1, 0], poses[:k + 1, 1], 'b-', lw=0.8)
        viz.draw_robot(ax_a, pose)
        viz.frame(ax_a)
        ax_a.set_title(f'mapping... {grid.seen.sum()} cells seen')
        anim.frame()
if anim is not None:
    anim.close()
snapshots.append((len(poses) - 1, grid.to_ros(), poses.copy()))

ros = grid.to_ros()
print(f'map {geom.width} x {geom.height} cells at {geom.res} m: '
      f'{(ros == 0).sum()} free, {(ros == 100).sum()} occupied, {(ros == -1).sum()} unknown')

fig, axes = plt.subplots(2, 3, figsize=(15, 8.5))
fig.suptitle('Module 2: occupancy grid mapping', fontsize=14)
for ax, (k, values, trail) in zip(axes.flat[:4], snapshots):
    viz.show_map(ax, values, geom)
    ax.plot(trail[:, 0], trail[:, 1], 'b-', lw=0.8)
    viz.draw_robot(ax, trail[-1])
    viz.frame(ax)
    ax.set_title(f'after {k * 0.1:.0f} s  (grey = unknown)', fontsize=10)

ax = axes.flat[4]
im = ax.imshow(grid.probability(), origin='lower', extent=geom.extent, cmap='RdYlGn_r', vmin=0, vmax=1)
fig.colorbar(im, ax=ax, fraction=0.035, label='P(occupied)')
for name, cell, color in [('wall cell', wall_cell, 'red'), ('free cell', free_cell, 'green')]:
    xy = geom.to_world(*cell)
    ax.plot(*xy, 'o', mfc='none', mec=color, ms=10, mew=2)
viz.frame(ax)
ax.set_title('belief per cell: P(occupied)', fontsize=10)

ax = axes.flat[5]
ax.plot(history['wall'], 'r-', label='wall cell')
ax.plot(history['free'], 'g-', label='free cell')
ax.axhline(0.5, color='0.6', ls='--', lw=0.8)
ax.set_ylim(-0.02, 1.02)
ax.set_xlabel('scan number')
ax.set_ylabel('P(occupied)')
ax.set_title('Bayes update: each scan adds evidence\n(l_new = l_old + l_measurement)', fontsize=10)
ax.legend()

fig.tight_layout()
viz.finish(fig, args, 'mapping_overview')
viz.show(args)
