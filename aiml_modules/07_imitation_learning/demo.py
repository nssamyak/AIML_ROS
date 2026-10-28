"""Module 7: machine learning. Teach a small neural network to drive by imitating MPPI.

  1. The EXPERT (MPPI + A* path, knows the map) drives many short trips.
     We record (what the lidar saw, where the goal was) -> (command it chose).
  2. BEHAVIOUR CLONING: train a network on that data (supervised learning).
  3. DAgger: let the NETWORK drive, ask the expert what it WOULD have done in
     the places the network ends up, add that to the data, retrain.
  4. Compare expert, cloned and DAgger policies on new trips.

    python 07_imitation_learning/demo.py
    python 07_imitation_learning/demo.py --dagger-rounds 0        # behaviour cloning only
    python 07_imitation_learning/demo.py --train-trips 10         # less data: what happens?
    python 07_imitation_learning/demo.py --hidden 8               # tiny network
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from hbai import robot, viz
from hbai.gridmap import GridGeom
from hbai.learning import MLP, N_FEATURES, network_policy, run_episode
from hbai.lidar import Lidar
from hbai.mppi import MPPI
from hbai.planning import costmap, distance_field, path_length, plan
from hbai.world import arena

p = viz.parser(__doc__)
p.add_argument('--train-trips', type=int, default=40)
p.add_argument('--dagger-rounds', type=int, default=3)
p.add_argument('--dagger-trips', type=int, default=15)
p.add_argument('--test-trips', type=int, default=30)
p.add_argument('--epochs', type=int, default=150)
p.add_argument('--hidden', type=int, default=64, help='neurons per hidden layer')
args, plt = viz.init(p, '07_imitation_learning')
if args.fast:
    args.train_trips, args.dagger_rounds, args.dagger_trips, args.test_trips, args.epochs = 6, 1, 3, 4, 20

world = arena()
geom = GridGeom(world.xlim, world.ylim, 0.05)
occ = world.occupancy(geom)
cost = costmap(occ, geom.res)
dist = distance_field(occ, geom.res, 0.4)
lidar = Lidar(seed=args.seed)
rng = np.random.default_rng(args.seed)


def sample_trip():
    """A start and goal 0.6 - 1.6 m apart, reachable without a big detour."""
    while True:
        start = world.random_free_point(rng, clearance=0.2)
        goal = world.random_free_point(rng, clearance=0.2)
        d = np.linalg.norm(goal - start)
        if not 0.6 < d < 1.6:
            continue
        path, _ = plan(geom, cost, start, goal)
        if path is not None and path_length(path) < 1.3 * d:
            return start, goal, path


def make_expert(path, seed):
    """MPPI that knows the map and the A* path. No turning (wz = 0), like the student."""
    mppi = MPPI(geom, dist, samples=256, horizon=15, wmax=0.0, noise_std=(0.2, 0.2, 0.0), seed=seed)
    return lambda pose, ranges, goal: mppi.command(pose, path, goal)


def collect(trips, policy_for=None):
    """Run trips and label every visited state with the expert's command."""
    X, Y = [], []
    for i in range(trips):
        start, goal, path = sample_trip()
        expert = make_expert(path, seed=int(rng.integers(1 << 30)))
        driver = expert if policy_for is None else policy_for
        result = run_episode(world, lidar, driver, start, goal, max_steps=250, labeler=expert)
        X += result['X']
        Y += result['Y']
    return np.array(X), np.array(Y)


def evaluate(policies, trips):
    stats = {name: dict(success=0, collision=0, timeout=0, steps=[], think=[]) for name in policies}
    examples = []
    for i in range(trips):
        start, goal, path = sample_trip()
        row = (start, goal, {})
        for name, make in policies.items():
            res = run_episode(world, lidar, make(path, i), start, goal, max_steps=250)
            s = stats[name]
            s[res['status']] += 1
            s['think'].append(res['think_ms'])
            if res['status'] == 'success':
                s['steps'].append(res['steps'])
            row[2][name] = res['traj']
        if len(examples) < 6:
            examples.append(row)
    return stats, examples


t_start = time.perf_counter()

# ---- 1. expert demonstrations
print(f'1) expert drives {args.train_trips} trips ...')
X, Y = collect(args.train_trips)
print(f'   {len(X)} examples: {N_FEATURES} inputs -> 2 outputs (vx, vy)')

# ---- 2. behaviour cloning
idx = rng.permutation(len(X))
n_val = max(1, len(X) // 7)
val, tr = idx[:n_val], idx[n_val:]
model = MLP((N_FEATURES, args.hidden, args.hidden, 2), seed=args.seed)
print(f'2) training a {model.n_params}-parameter network for {args.epochs} epochs ...')
hist = model.fit(X[tr], Y[tr], epochs=args.epochs, val=(X[val], Y[val]), seed=args.seed)
histories = [('behaviour cloning', hist)]
bc_model = MLP((N_FEATURES, args.hidden, args.hidden, 2))
bc_model.W, bc_model.b = [w.copy() for w in model.W], [b.copy() for b in model.b]
print(f'   validation error (MSE): {hist["val"][-1]:.4f}')

# ---- 3. DAgger
student = network_policy(model)
for r in range(args.dagger_rounds):
    Xd, Yd = collect(args.dagger_trips, policy_for=student)
    X, Y = np.vstack([X, Xd]), np.vstack([Y, Yd])
    h = model.fit(X, Y, epochs=max(10, args.epochs // 3), val=(X[val], Y[val]), seed=args.seed + r)
    histories.append((f'DAgger round {r + 1}', h))
    print(f'3) DAgger round {r + 1}: +{len(Xd)} examples where the STUDENT drove, now {len(X)} total')

# ---- 4. evaluation on new trips
print(f'4) testing on {args.test_trips} new trips ...')
policies = {'expert (MPPI)': lambda path, i: make_expert(path, seed=1000 + i),
            'cloned network': lambda path, i: network_policy(bc_model)}
if args.dagger_rounds:
    policies['DAgger network'] = lambda path, i: network_policy(model)
stats, examples = evaluate(policies, args.test_trips)

print(f'\n{"policy":16s} {"success":>8s} {"crash":>6s} {"timeout":>8s} {"avg time":>9s} {"thinking":>10s}')
for name, s in stats.items():
    n = args.test_trips
    avg = f'{np.mean(s["steps"]) / 10:.1f} s' if s['steps'] else '-'
    print(f'{name:16s} {s["success"] / n:8.0%} {s["collision"] / n:6.0%} {s["timeout"] / n:8.0%} '
          f'{avg:>9s} {np.mean(s["think"]):8.3f} ms')
print(f'(total {time.perf_counter() - t_start:.0f} s)')
if args.save:
    model.save(args.out / 'student_weights.npz')

# ---- figures
fig = plt.figure(figsize=(15, 9))
fig.suptitle('Module 7: learning to drive by imitation', fontsize=14)

ax = fig.add_subplot(2, 3, 1)
offset = 0
for name, h in histories:
    e = np.arange(offset, offset + len(h['train']))
    ax.plot(e, h['train'], label=f'{name} (train)')
    offset += len(h['train'])
val_curve = np.concatenate([h['val'] for _, h in histories])
ax.plot(val_curve, 'k:', lw=1, label='validation')
ax.set_yscale('log')
ax.set_xlabel('epoch')
ax.set_ylabel('mean squared error')
ax.set_title('training: error between network and expert', fontsize=10)
ax.legend(fontsize=7)

ax = fig.add_subplot(2, 3, 2)
names = list(stats)
bottom = np.zeros(len(names))
for outcome, color in [('success', 'tab:green'), ('collision', 'tab:red'), ('timeout', 'tab:orange')]:
    vals = np.array([stats[n][outcome] / args.test_trips * 100 for n in names])
    ax.bar(range(len(names)), vals, bottom=bottom, color=color, label=outcome)
    bottom += vals
ax.set_xticks(range(len(names)))
ax.set_xticklabels([n.replace(' ', '\n', 1) for n in names], fontsize=8)
ax.set_ylabel('% of test trips')
ax.set_title('results on trips never seen in training', fontsize=10)
ax.legend(fontsize=8)

ax = fig.add_subplot(2, 3, 3)
think = [np.mean(stats[n]['think']) for n in names]
ax.bar(range(len(names)), think, color=['tab:gray', 'tab:blue', 'tab:purple'][:len(names)])
ax.set_yscale('log')
ax.set_xticks(range(len(names)))
ax.set_xticklabels([n.replace(' ', '\n', 1) for n in names], fontsize=8)
ax.set_ylabel('ms per decision (log scale)')
ax.set_title('thinking time per command', fontsize=10)

colors = {'expert (MPPI)': 'k', 'cloned network': 'tab:blue', 'DAgger network': 'tab:purple'}
for k, (start, goal, trajs) in enumerate(examples[:3]):
    ax = fig.add_subplot(2, 3, 4 + k)
    viz.draw_world(ax, world, color='0.4')
    for name, traj in trajs.items():
        ax.plot(traj[:, 0], traj[:, 1], color=colors[name], lw=1.5, label=name)
    ax.plot(*start, 'o', color='white', mec='k', ms=8)
    ax.plot(*goal, '*', color='red', mec='k', ms=14)
    ax.set_title(f'test trip {k + 1}', fontsize=10)
    if k == 0:
        ax.legend(fontsize=7, loc='lower right')
fig.tight_layout()
viz.finish(fig, args, 'imitation_learning')
viz.show(args)
