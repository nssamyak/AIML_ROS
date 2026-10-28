"""Module 3: SLAM. Odometry drifts; scan matching pulls the pose back onto the map.

Two robots drive the same route with the same bad wheels:
  left   maps with odometry only        -> walls smear and double up
  right  corrects each pose by scan matching against its own map -> clean map

    python 03_slam_scan_matching/demo.py
    python 03_slam_scan_matching/demo.py --yaw-drift 0.15     # worse wheels
    python 03_slam_scan_matching/demo.py --no-penalty         # trust the scan only
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from hbai import robot, viz
from hbai.gridmap import GridGeom
from hbai.lidar import Lidar, scan_to_points
from hbai.mapping import OccupancyGrid
from hbai.scan_matching import likelihood_field, match
from hbai.tour import TOUR, tour_poses
from hbai.world import arena

p = viz.parser(__doc__)
p.add_argument('--scale-error', type=float, default=0.04, help='wheel size error (fraction)')
p.add_argument('--yaw-drift', type=float, default=0.06, help='heading error per metre (rad/m)')
p.add_argument('--match-every', type=int, default=3, help='scan-match every N steps')
p.add_argument('--no-penalty', action='store_true', help="don't prefer poses near the odometry guess")
args, plt = viz.init(p, '03_slam_scan_matching')

world = arena()
truth = tour_poses(world, TOUR[:3] if args.fast else TOUR)
geom = GridGeom(world.xlim, world.ylim, 0.05)
lidar = Lidar(seed=args.seed)
odom = robot.DriftingOdometry(truth[0], scale_error=args.scale_error, yaw_drift=args.yaw_drift,
                              seed=args.seed)
penalty = dict(penalty_xy=0.0, penalty_th=0.0) if args.no_penalty else {}

map_odom = OccupancyGrid(geom)       # robot A: trusts odometry
map_slam = OccupancyGrid(geom)       # robot B: odometry + scan matching
pose_odom = truth[0].copy()
pose_slam = truth[0].copy()
est_odom, est_slam, err_odom, err_slam = [pose_odom], [pose_slam], [0.0], [0.0]
match_time = []

for k in range(1, len(truth)):
    before = odom.pose.copy()
    after = odom.update(truth[k - 1], truth[k])
    delta = robot.relative(before, after)              # what the wheels say happened
    pose_odom = after
    pose_slam = robot.compose(pose_slam, delta)        # prediction from odometry

    if k % args.match_every == 0:
        ranges = lidar.scan(world, truth[k])           # the scan comes from the TRUE pose
        map_odom.update(pose_odom, lidar.angles, ranges)
        if map_slam.seen.any():
            t0 = time.perf_counter()
            field = likelihood_field(map_slam)
            pts = scan_to_points(ranges, lidar.angles)[::3]
            pose_slam, _ = match(field, geom, pts, pose_slam, **penalty)   # correction
            match_time.append(time.perf_counter() - t0)
        map_slam.update(pose_slam, lidar.angles, ranges)

    est_odom.append(pose_odom.copy())
    est_slam.append(pose_slam.copy())
    err_odom.append(np.hypot(*(pose_odom[:2] - truth[k, :2])))
    err_slam.append(np.hypot(*(pose_slam[:2] - truth[k, :2])))

est_odom, est_slam = np.array(est_odom), np.array(est_slam)
dist = np.sum(np.linalg.norm(np.diff(truth[:, :2], axis=0), axis=1))
print(f'drove {dist:.1f} m. Final position error: odometry {err_odom[-1] * 100:.1f} cm, '
      f'with scan matching {err_slam[-1] * 100:.1f} cm. '
      f'Scan match: {1000 * np.mean(match_time):.0f} ms each.')

fig, axes = plt.subplots(1, 3, figsize=(17, 5.2), gridspec_kw=dict(width_ratios=[1, 1, 0.8]))
fig.suptitle('Module 3: SLAM front end, odometry vs scan matching', fontsize=14)
for ax, grid, est, title in [(axes[0], map_odom, est_odom, 'map from ODOMETRY only'),
                             (axes[1], map_slam, est_slam, 'map with SCAN MATCHING')]:
    viz.show_map(ax, grid.to_ros(), geom)
    ax.plot(truth[:, 0], truth[:, 1], 'g-', lw=1.2, label='true path')
    ax.plot(est[:, 0], est[:, 1], 'm--', lw=1.2, label='robot thinks')
    viz.frame(ax)
    ax.set_title(title)
    ax.legend(loc='lower right', fontsize=8)

ax = axes[2]
t = np.arange(len(truth)) * 0.1
ax.plot(t, np.array(err_odom) * 100, 'm-', label='odometry only')
ax.plot(t, np.array(err_slam) * 100, 'b-', label='with scan matching')
ax.set_xlabel('time (s)')
ax.set_ylabel('position error (cm)')
ax.set_title('odometry error grows without limit;\nscan matching keeps it bounded')
ax.legend()
fig.tight_layout()
viz.finish(fig, args, 'slam_comparison')
viz.show(args)
