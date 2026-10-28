"""Machine learning: a small neural network that learns to drive by imitation.

The teacher (expert) is the MPPI controller, which needs the global path,
the map, and ~5 ms of computation per step. The student is a small neural
network that only sees the current lidar scan and where the goal is, and
answers in microseconds. Small enough to run on the ESP32.

    features(scan, pose, goal)  ->  39 numbers in [-1, 1]
    MLP: 39 -> 64 -> 64 -> 2    ->  (vx, vy) / max_speed

Training is ordinary supervised learning: minimise the mean squared error
between the network's command and the expert's command (behaviour cloning).
"""
import time

import numpy as np

from hbai import robot

SECTORS = 36          # lidar summarised into 36 sectors of 10 degrees
SENSE_RANGE = 1.5     # m, distances are clipped to this


def features(ranges, pose, goal):
    """What the network sees: nearest obstacle per sector + goal direction, all scaled to [-1, 1]."""
    r = np.where(np.isfinite(ranges), ranges, SENSE_RANGE)
    r = np.minimum(r, SENSE_RANGE).reshape(SECTORS, -1).min(axis=1) / SENSE_RANGE
    dx, dy = goal[0] - pose[0], goal[1] - pose[1]
    c, s = np.cos(pose[2]), np.sin(pose[2])
    g = np.array([c * dx + s * dy, -s * dx + c * dy])          # goal in the robot frame
    dist = np.linalg.norm(g)
    g = g / max(dist, 1e-9) * min(dist, SENSE_RANGE) / SENSE_RANGE
    return np.concatenate([r, g, [min(dist, SENSE_RANGE) / SENSE_RANGE]])


N_FEATURES = SECTORS + 3


class MLP:
    """Fully connected network: ReLU hidden layers, tanh output, trained with Adam."""

    def __init__(self, sizes=(N_FEATURES, 64, 64, 2), seed=0):
        rng = np.random.default_rng(seed)
        self.W = [rng.normal(0, np.sqrt(2.0 / a), (a, b)) for a, b in zip(sizes[:-1], sizes[1:])]
        self.b = [np.zeros(b) for b in sizes[1:]]
        params = self.W + self.b
        self._adam = [np.zeros_like(p) for p in params] + [np.zeros_like(p) for p in params]
        self._t = 0

    def forward(self, X):
        acts = [X]
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            z = acts[-1] @ W + b
            acts.append(np.tanh(z) if i == len(self.W) - 1 else np.maximum(z, 0.0))
        return acts

    def predict(self, X):
        return self.forward(np.atleast_2d(X))[-1]

    def train_step(self, X, Y, lr=1e-3, beta1=0.9, beta2=0.999, eps=1e-8):
        """One gradient step on a mini-batch. Returns the batch loss (MSE)."""
        acts = self.forward(X)
        out = acts[-1]
        loss = float(np.mean((out - Y) ** 2))

        # Backpropagation: push the error back through each layer (chain rule)
        grad = 2.0 * (out - Y) / Y.size * (1.0 - out ** 2)           # through tanh
        gW, gb = [], []
        for i in reversed(range(len(self.W))):
            gW.insert(0, acts[i].T @ grad)
            gb.insert(0, grad.sum(axis=0))
            if i > 0:
                grad = (grad @ self.W[i].T) * (acts[i] > 0)           # through ReLU

        # Adam: per-parameter step sizes from running averages of the gradients
        self._t += 1
        params, grads = self.W + self.b, gW + gb
        n = len(params)
        for k, (p, g) in enumerate(zip(params, grads)):
            m, v = self._adam[k], self._adam[n + k]
            m[:] = beta1 * m + (1 - beta1) * g
            v[:] = beta2 * v + (1 - beta2) * g * g
            m_hat = m / (1 - beta1 ** self._t)
            v_hat = v / (1 - beta2 ** self._t)
            p -= lr * m_hat / (np.sqrt(v_hat) + eps)
        return loss

    def fit(self, X, Y, epochs=100, batch=128, lr=1e-3, val=None, seed=0):
        rng = np.random.default_rng(seed)
        history = {'train': [], 'val': []}
        for _ in range(epochs):
            order = rng.permutation(len(X))
            losses = [self.train_step(X[i], Y[i], lr) for i in np.array_split(order, max(1, len(X) // batch))]
            history['train'].append(float(np.mean(losses)))
            if val is not None:
                history['val'].append(float(np.mean((self.predict(val[0]) - val[1]) ** 2)))
        return history

    def save(self, path):
        np.savez(path, *self.W, *self.b)

    @property
    def n_params(self):
        return sum(p.size for p in self.W + self.b)


# ---------------------------------------------------------------------- episodes
def run_episode(world, lidar, policy, start, goal, max_steps=300, dt=0.1, labeler=None,
                goal_tolerance=0.1):
    """Drive `policy(pose, ranges, goal) -> (vx, vy, wz)` from start to goal.

    labeler(pose, ranges, goal) -> expert command, recorded for every visited state
    (used to build training data). Returns a dict with the result and the data.
    """
    pose = np.array([*start[:2], 0.0])
    X, Y, traj, think = [], [], [pose.copy()], []
    status = 'timeout'
    for _ in range(max_steps):
        ranges = lidar.scan(world, pose)
        if labeler is not None:
            X.append(features(ranges, pose, goal))
            Y.append(np.asarray(labeler(pose, ranges, goal))[:2] / robot.MAX_SPEED)
        t0 = time.perf_counter()
        cmd = np.asarray(policy(pose, ranges, goal), dtype=float)
        think.append(time.perf_counter() - t0)
        pose = robot.step(pose, cmd, dt)
        traj.append(pose.copy())
        if world.distance(pose[:2])[0] < robot.RADIUS:
            status = 'collision'
            break
        if np.hypot(pose[0] - goal[0], pose[1] - goal[1]) < goal_tolerance:
            status = 'success'
            break
    return dict(status=status, traj=np.array(traj), X=X, Y=Y, steps=len(traj) - 1,
                think_ms=1000 * float(np.mean(think)))


def network_policy(model):
    """Wrap a trained network as a driving policy."""
    def policy(pose, ranges, goal):
        vx, vy = model.predict(features(ranges, pose, goal))[0] * robot.MAX_SPEED
        return np.array([vx, vy, 0.0])
    return policy
