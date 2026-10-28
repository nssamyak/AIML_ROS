"""Module 1: what a 2D lidar sees, and how to turn ranges into points.

    python 01_lidar_perception/demo.py
    python 01_lidar_perception/demo.py --pose 0.5 0.8 1.57      # put the robot elsewhere
    python 01_lidar_perception/demo.py --animate                # drive around, watch the scan
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from hbai import viz
from hbai.lidar import Lidar, scan_to_points
from hbai.tour import tour_poses
from hbai.world import START, arena

p = viz.parser(__doc__)
p.add_argument('--pose', type=float, nargs=3, default=list(START), metavar=('X', 'Y', 'THETA'))
p.add_argument('--noise', type=float, default=0.01, help='range noise std (m)')
p.add_argument('--animate', action='store_true', help='drive around the arena with a live scan')
args, plt = viz.init(p, '01_lidar_perception')

world = arena()
lidar = Lidar(noise_std=args.noise, seed=args.seed)
pose = np.array(args.pose)
ranges = lidar.scan(world, pose)
hits = np.isfinite(ranges)

print(f'{lidar.n_beams} beams, {hits.sum()} hit something, '
      f'closest {np.nanmin(np.where(hits, ranges, np.nan)):.2f} m, '
      f'furthest {np.nanmax(np.where(hits, ranges, np.nan)):.2f} m')

fig = plt.figure(figsize=(12, 9))
fig.suptitle('Module 1: lidar perception', fontsize=14)

# (a) the world, the beams, the hits
ax = fig.add_subplot(2, 2, 1)
viz.draw_world(ax, world, color='0.75')
world_pts = scan_to_points(ranges, lidar.angles, pose)
for a, r in zip(lidar.angles[::8], ranges[::8]):
    r = r if np.isfinite(r) else 1.0
    ax.plot([pose[0], pose[0] + r * np.cos(pose[2] + a)], [pose[1], pose[1] + r * np.sin(pose[2] + a)],
            color='tab:orange', lw=0.4, alpha=0.6)
ax.scatter(world_pts[:, 0], world_pts[:, 1], s=3, c='red', zorder=4, label='hits')
viz.draw_robot(ax, pose)
ax.set_title('(a) world view: every 8th beam and all hits')
ax.legend(loc='lower right', fontsize=8)

# (b) the raw data: one range per angle
ax = fig.add_subplot(2, 2, 2, projection='polar')
ax.scatter(lidar.angles[hits], ranges[hits], s=3, c=np.degrees(lidar.angles[hits]), cmap='twilight')
ax.set_theta_zero_location('N')       # robot forward = up
ax.set_title('(b) raw scan: range vs beam angle\n(what /scan contains; forward is up)', fontsize=10)

# (c) robot-frame points
ax = fig.add_subplot(2, 2, 3)
robot_pts = scan_to_points(ranges, lidar.angles)
ax.scatter(robot_pts[:, 0], robot_pts[:, 1], s=3, c='red')
viz.draw_robot(ax, (0, 0, 0))
ax.axhline(0, color='0.8', lw=0.5)
ax.axvline(0, color='0.8', lw=0.5)
ax.set_aspect('equal')
ax.set_xlabel('x forward (m)')
ax.set_ylabel('y left (m)')
ax.set_title('(c) points in the ROBOT frame (lidar_link)\nx = r cos(a), y = r sin(a)', fontsize=10)

# (d) noise: scan the same spot many times
ax = fig.add_subplot(2, 2, 4)
beam = int(np.argmin(np.abs(lidar.angles)))       # the beam pointing straight ahead
truth = world.raycast(pose[:2], [pose[2] + lidar.angles[beam]])[0]
samples = np.array([lidar.scan(world, pose)[beam] for _ in range(300 if not args.fast else 50)])
samples = samples[np.isfinite(samples)]
ax.hist((samples - truth) * 100, bins=30, color='tab:purple')
ax.set_xlabel('measured - true range (cm)')
ax.set_title(f'(d) noise on the forward beam (true range {truth:.2f} m)\n'
             f'std = {np.std(samples - truth) * 100:.1f} cm', fontsize=10)

fig.tight_layout()
viz.finish(fig, args, 'lidar_overview')

# Optional: drive around and watch the scan
if args.animate and not args.fast:
    poses = tour_poses(world)[::3]
    fig2, ax2 = plt.subplots(figsize=(8, 6))
    anim = viz.Animator(fig2, args, 'lidar_drive', fps=15)
    for k, q in enumerate(poses):
        r = lidar.scan(world, q)
        pts = scan_to_points(r, lidar.angles, q)
        ax2.clear()
        viz.draw_world(ax2, world, color='0.85')
        ax2.plot(poses[:k + 1, 0], poses[:k + 1, 1], 'b-', lw=0.8, alpha=0.5)
        ax2.scatter(pts[:, 0], pts[:, 1], s=4, c='red', zorder=4)
        viz.draw_robot(ax2, q)
        ax2.set_title(f'live scan, step {k}: the robot only knows the red points')
        anim.frame()
    anim.close()

viz.show(args)
