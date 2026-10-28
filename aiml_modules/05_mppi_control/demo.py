"""Module 5: MPPI control. Follow the planned path by trying hundreds of futures.

    python 05_mppi_control/demo.py
    python 05_mppi_control/demo.py --temperature 1.0     # too soft: noise averages out, robot crawls
    python 05_mppi_control/demo.py --temperature 0.005   # too hard: jerky
    python 05_mppi_control/demo.py --twirl 0             # no penalty for spinning
    python 05_mppi_control/demo.py --samples 50          # fewer samples
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from hbai import robot, viz
from hbai.gridmap import GridGeom
from hbai.mppi import MPPI
from hbai.planning import costmap, distance_field, plan
from hbai.world import START, arena

p = viz.parser(__doc__)
p.add_argument('--goal', type=float, nargs=2, default=[1.65, 0.55])
p.add_argument('--samples', type=int, default=400)
p.add_argument('--horizon', type=int, default=20, help='steps of 0.1 s')
p.add_argument('--temperature', type=float, default=0.05)
p.add_argument('--twirl', type=float, default=3.0, help='weight of the twirling critic')
p.add_argument('--no-anim', action='store_true', help='skip the animation, just the summary plot')
args, plt = viz.init(p, '05_mppi_control')

world = arena()
geom = GridGeom(world.xlim, world.ylim, 0.05)
occ = world.occupancy(geom)
path, _ = plan(geom, costmap(occ, geom.res), START, args.goal)
ctrl = MPPI(geom, distance_field(occ, geom.res, 0.4), samples=args.samples, horizon=args.horizon,
            temperature=args.temperature, weights=dict(twirl=args.twirl), seed=args.seed)

pose = np.array(START)
poses, cmds, think, ess = [pose], [], [], []
max_steps = 150 if args.fast else 800
animate = not args.no_anim and not args.fast
if animate:
    fig_a, ax_a = plt.subplots(figsize=(9, 7))
    anim = viz.Animator(fig_a, args, 'mppi_drive', fps=12)

for k in range(max_steps):
    t0 = time.perf_counter()
    cmd = ctrl.command(pose, path, args.goal)
    think.append(time.perf_counter() - t0)
    w = ctrl.last['weights']
    ess.append(1.0 / np.sum(w ** 2))         # "effective number of samples" that matter
    pose = robot.step(pose, cmd, 0.1)
    poses.append(pose)
    cmds.append(cmd)

    if animate and k % 2 == 0:
        ax_a.clear()
        viz.draw_world(ax_a, world, color='0.35')
        ax_a.plot(path[:, 0], path[:, 1], 'k--', lw=1, label='global path (A*)')
        traj = ctrl.last['traj']
        show = np.argsort(w)[::-1][:60]                       # best 60 samples
        for i in show[::-1]:
            ax_a.plot(traj[i, :, 0], traj[i, :, 1], color=(1 - min(1, w[i] * 20), 0.3, 0.8),
                      lw=0.6, alpha=0.25 + 0.75 * min(1, w[i] * 20))
        plan_xy = ctrl.last['plan']
        ax_a.plot(plan_xy[:, 0], plan_xy[:, 1], 'g-', lw=2.5, label='MPPI plan (weighted average)')
        trail = np.array(poses)
        ax_a.plot(trail[:, 0], trail[:, 1], 'b-', lw=1)
        viz.draw_robot(ax_a, pose)
        ax_a.plot(*args.goal, 'r*', ms=15, mec='k')
        ax_a.set_title(f't = {k / 10:.1f} s   cmd vx {cmd[0]:+.2f}  vy {cmd[1]:+.2f}  wz {cmd[2]:+.2f}   '
                       f'samples that matter: {ess[-1]:.0f} / {args.samples}', fontsize=10)
        ax_a.legend(loc='lower right', fontsize=8)
        anim.frame()

    if np.hypot(pose[0] - args.goal[0], pose[1] - args.goal[1]) < 0.08:
        break
if animate:
    anim.close()

reached = np.hypot(pose[0] - args.goal[0], pose[1] - args.goal[1]) < 0.08
poses, cmds = np.array(poses), np.array(cmds)
clearance = world.distance(poses[:, :2]) - robot.RADIUS
print(f'{"reached the goal" if reached else "did NOT reach the goal"} in {len(cmds) / 10:.1f} s, '
      f'closest to a wall {clearance.min() * 100:.1f} cm, total rotation {np.abs(cmds[:, 2]).sum() * 0.1:.1f} rad, '
      f'{1000 * np.mean(think):.1f} ms per control step')

fig, axes = plt.subplots(1, 2, figsize=(14, 5), gridspec_kw=dict(width_ratios=[1.1, 1]))
fig.suptitle(f'Module 5: MPPI (K = {args.samples}, T = {args.horizon}, temperature = {args.temperature})',
             fontsize=13)
ax = axes[0]
viz.draw_world(ax, world, color='0.35')
ax.plot(path[:, 0], path[:, 1], 'k--', lw=1, label='global path')
sc = ax.scatter(poses[:-1, 0], poses[:-1, 1], c=np.hypot(cmds[:, 0], cmds[:, 1]), cmap='viridis', s=6,
                vmin=0, vmax=robot.MAX_SPEED)
fig.colorbar(sc, ax=ax, fraction=0.035, label='speed (m/s)')
for q in poses[::40]:
    viz.draw_robot(ax, q, color='tab:cyan')
ax.plot(*args.goal, 'r*', ms=15, mec='k')
ax.legend(loc='lower right', fontsize=8)
ax.set_title('driven path (robot drawn every 4 s: it strafes, it does not turn)', fontsize=10)

ax = axes[1]
t = np.arange(len(cmds)) / 10
ax.plot(t, cmds[:, 0], label='vx (forward)')
ax.plot(t, cmds[:, 1], label='vy (sideways)')
ax.plot(t, cmds[:, 2], label='wz (turn)', alpha=0.7)
ax.set_xlabel('time (s)')
ax.set_ylabel('command')
ax2 = ax.twinx()
ax2.plot(t, ess, color='0.6', lw=0.8, ls=':', label='samples that matter')
ax2.set_ylabel('effective samples', color='0.5')
ax.legend(loc='upper left', fontsize=8)
ax.set_title('commands over time (holonomic: vx and vy together)', fontsize=10)
fig.tight_layout()
viz.finish(fig, args, 'mppi_summary')
viz.show(args)
