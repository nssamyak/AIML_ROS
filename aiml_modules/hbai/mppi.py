"""MPPI: Model Predictive Path Integral control, with the omni (holonomic) motion model.

Same idea as Nav2's nav2_mppi_controller with motion_model: "Omni".

Every control cycle:
  1. SAMPLE   K random control sequences (vx, vy, wz for T steps) around the
              current plan, by adding Gaussian noise.
  2. ROLL OUT simulate where each sequence would take the robot.
  3. SCORE    each trajectory with critics:
                collision  hits a wall                  -> huge cost
                obstacle   gets close to walls          -> cost
                path       strays from the global path  -> cost
                follow     ends far from a point ahead on the path -> cost
                goal       (near the goal) ends far from it -> cost
                twirl      spins for no reason          -> cost
  4. AVERAGE  the sequences, weighting each by exp(-cost / temperature).
              Good ones count a lot, bad ones almost nothing.
  5. ACT      send the first command, shift the plan by one step, repeat.

The temperature decides how "soft" the average is. Too high and every sample
counts the same, so the noise cancels out and the robot crawls. Too low and
only the single best sample counts, so the motion gets jerky. (Try it in
module 5.)

No gradients and no learning: just "try many futures, keep a soft best".
"""
import numpy as np

from hbai.robot import MAX_SPEED, RADIUS


class MPPI:

    def __init__(self, geom, dist_field, robot_radius=RADIUS, samples=400, horizon=20, dt=0.1,
                 vmax=MAX_SPEED, wmax=1.0, noise_std=(0.2, 0.2, 0.1), temperature=0.05,
                 lookahead=0.4, goal_zone=0.4, weights=None, seed=None):
        self.geom = geom
        self.dist_field = dist_field
        self.radius = robot_radius
        self.K, self.T, self.dt = samples, horizon, dt
        self.vmax, self.wmax = vmax, wmax
        self.std = np.array(noise_std, dtype=float)
        self.temperature = temperature
        self.lookahead = lookahead
        self.goal_zone = goal_zone
        self.w = dict(collision=1000.0, obstacle=200.0, path=5.0, follow=10.0, goal=8.0, twirl=3.0)
        self.w.update(weights or {})
        self.rng = np.random.default_rng(seed)
        self.u = np.zeros((self.T, 3))          # current plan (warm start)
        self.last = {}                          # rollouts, weights, ... for plotting

    def set_map(self, dist_field):
        self.dist_field = dist_field

    def reset(self):
        self.u[:] = 0.0

    # ------------------------------------------------------------------ helpers
    def _clip(self, u):
        u[..., 2] = np.clip(u[..., 2], -self.wmax, self.wmax)
        speed = np.hypot(u[..., 0], u[..., 1])
        scale = np.minimum(1.0, self.vmax / np.maximum(speed, 1e-9))
        u[..., 0] *= scale
        u[..., 1] *= scale
        return u

    def rollout(self, state, controls):
        """Simulate (K, T, 3) control sequences from `state`. Returns (K, T, 3) poses."""
        x0, y0, th0 = state
        wz = controls[..., 2]
        heading = th0 + np.cumsum(wz * self.dt, axis=1)
        heading_before = np.concatenate([np.full((len(controls), 1), th0), heading[:, :-1]], axis=1)
        c, s = np.cos(heading_before), np.sin(heading_before)
        x = x0 + np.cumsum((controls[..., 0] * c - controls[..., 1] * s) * self.dt, axis=1)
        y = y0 + np.cumsum((controls[..., 0] * s + controls[..., 1] * c) * self.dt, axis=1)
        return np.stack([x, y, heading], axis=-1)

    def _path_window(self, state, path):
        """Nearest path point, a target `lookahead` metres further, and the local piece of path."""
        d = np.linalg.norm(path - state[:2], axis=1)
        i0 = int(np.argmin(d))
        seg = np.linalg.norm(np.diff(path[i0:], axis=0), axis=1)
        ahead = np.concatenate([[0.0], np.cumsum(seg)])
        i_target = i0 + int(np.searchsorted(ahead, self.lookahead))
        i_target = min(i_target, len(path) - 1)
        reach = self.lookahead + self.vmax * self.T * self.dt
        i_end = i0 + int(np.searchsorted(ahead, reach)) + 1
        return path[i_target], path[i0:min(i_end, len(path))]

    # ------------------------------------------------------------------ control
    def command(self, state, path, goal):
        """One control cycle. Returns the (vx, vy, wz) to send now."""
        state = np.asarray(state, dtype=float)
        noise = self.rng.normal(0.0, 1.0, (self.K, self.T, 3)) * self.std
        controls = self._clip(self.u[None] + noise)
        controls[0] = self.u                     # always keep the current plan as a candidate
        traj = self.rollout(state, controls)
        xy = traj[..., :2]

        # ---- critics
        clearance = self.geom.lookup(self.dist_field, xy, outside=0.0) - self.radius
        cost = self.w['collision'] * (clearance <= 0).any(axis=1)
        cost += self.w['obstacle'] * (np.maximum(0.0, 0.10 - clearance) ** 2).sum(axis=1)

        goal = np.asarray(goal, dtype=float)[:2]
        if np.linalg.norm(state[:2] - goal) < self.goal_zone:
            cost += self.w['goal'] * np.linalg.norm(xy[:, -1] - goal, axis=1)
        else:
            target, local = self._path_window(state, path)
            cost += self.w['follow'] * np.linalg.norm(xy[:, -1] - target, axis=1)
            d = np.linalg.norm(xy[:, :, None, :] - local[None, None], axis=-1).min(axis=2)
            cost += self.w['path'] * d.mean(axis=1)
        cost += self.w['twirl'] * np.abs(controls[..., 2]).mean(axis=1)

        # ---- soft-min average
        weights = np.exp(-(cost - cost.min()) / self.temperature)
        weights /= weights.sum()
        self.u = self._clip((weights[:, None, None] * controls).sum(axis=0))
        cmd = self.u[0].copy()

        self.last = dict(traj=traj, weights=weights, cost=cost, plan=self.rollout(state, self.u[None])[0])
        self.u = np.roll(self.u, -1, axis=0)     # shift the plan for the next cycle
        self.u[-1] = self.u[-2]
        return cmd
