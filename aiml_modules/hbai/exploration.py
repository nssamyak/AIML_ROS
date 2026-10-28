"""The whole autonomy loop in one place: sense -> map -> decide -> plan -> act.

It mirrors the real robot:
    lidar + mapping        slam_toolbox            (mapping.py, scan_matching.py)
    frontier choice        frontier_explorer node  (frontiers.py + Explorer.decide)
    global path            Nav2 planner_server     (planning.py)
    path following         Nav2 controller_server  (mppi.py)

Everything runs in one synchronous loop with a fixed time step, so it is easy
to pause, inspect and plot.
"""
import math
from dataclasses import dataclass

import numpy as np

from hbai import robot
from hbai.frontiers import find_frontiers
from hbai.gridmap import GridGeom
from hbai.lidar import Lidar, scan_to_points
from hbai.mapping import OccupancyGrid
from hbai.mppi import MPPI
from hbai.planning import costmap, distance_field, plan
from hbai.scan_matching import likelihood_field, match
from hbai.world import inside_arena

IDLE, NAVIGATING, RETURNING, DONE = 'idle', 'navigating', 'returning home', 'done'


@dataclass
class ExploreConfig:
    dt: float = 0.1                    # s per simulation step (10 Hz, like the lidar)
    map_res: float = 0.05              # m per map cell
    max_time: float = 900.0            # s of simulated time before giving up
    decide_period: float = 1.0         # s between frontier updates (explorer update_rate)
    min_frontier_size: float = 0.25    # m
    clearance: float = 0.12            # m, frontier goals stay this far from walls
    distance_weight: float = 1.0       # score = distance_weight * distance - size_weight * size
    size_weight: float = 0.5
    blacklist_radius: float = 0.3      # m
    progress_timeout: float = 15.0     # s
    frontier_lost_radius: float = 0.25 # m
    goal_tolerance: float = 0.12       # m
    empty_checks_to_finish: int = 3
    return_home: bool = True
    controller: str = 'mppi'           # 'mppi' or 'simple' (straight at the path, no obstacle checks)
    use_slam: bool = False             # False: mapping uses the true pose. True: odometry + scan matching
    mapping_range: float = 4.0         # m, ignore lidar returns further than this for mapping


class Explorer:

    def __init__(self, world, start, config=None, seed=0):
        self.world = world
        self.cfg = cfg = config or ExploreConfig()
        self.geom = GridGeom(world.xlim, world.ylim, cfg.map_res)
        self.grid = OccupancyGrid(self.geom)
        self.lidar = Lidar(seed=seed)
        self.mppi = MPPI(self.geom, np.ones(self.geom.shape), seed=seed)

        self.true_pose = np.array(start, dtype=float)     # where the robot really is
        self.pose = self.true_pose.copy()                 # where the robot THINKS it is
        self.home = self.true_pose.copy()
        self.odom = robot.DriftingOdometry(start, seed=seed) if cfg.use_slam else None

        self.state = IDLE
        self.goal = None
        self.path = None
        self.frontiers = []
        self.blacklist = []
        self.empty_checks = 0
        self.best_dist = math.inf
        self.last_progress = 0.0
        self.decide_now = True
        self.next_decision = 0.0
        self.goals_sent = 0

        self.t = 0.0
        self.steps = 0
        self.bumps = 0
        self.scan = None
        self.trajectory = [self.true_pose.copy()]
        self.coverage_history = []
        self.events = []

        # For measuring coverage: free cells inside the arena (from the true world)
        truth_occ = world.occupancy(self.geom)
        inside = inside_arena(self.geom.centers().reshape(-1, 2)).reshape(self.geom.shape)
        self.truth_free = ~truth_occ & inside

    # ------------------------------------------------------------------ helpers
    def say(self, text):
        self.events.append((round(self.t, 1), text))

    def coverage(self):
        return float((self.grid.free() & self.truth_free).sum() / self.truth_free.sum())

    def is_blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < self.cfg.blacklist_radius for bx, by in self.blacklist)

    def global_costmap(self):
        return costmap(self.grid.occupied(), self.geom.res, unknown=~self.grid.seen)

    @property
    def finished(self):
        return self.state == DONE or self.t >= self.cfg.max_time

    # ------------------------------------------------------------------ 1. sense + map
    def sense(self):
        ranges = self.lidar.scan(self.world, self.true_pose)
        self.scan = ranges
        if self.cfg.use_slam:
            if self.steps % 5 == 0:
                if self.grid.seen.any():
                    field = likelihood_field(self.grid)
                    points = scan_to_points(ranges, self.lidar.angles)[::3]
                    self.pose, _ = match(field, self.geom, points, self.pose)
                self.grid.update(self.pose, self.lidar.angles, ranges, self.cfg.mapping_range)
        elif self.steps % 2 == 0:
            self.grid.update(self.pose, self.lidar.angles, ranges, self.cfg.mapping_range)

    # ------------------------------------------------------------------ 2. decide
    def decide(self):
        cfg = self.cfg
        self.frontiers = find_frontiers(self.grid.to_ros(), self.geom.res, self.geom.x0, self.geom.y0,
                                        clearance=cfg.clearance, min_size=cfg.min_frontier_size)
        cost = self.global_costmap()

        if self.state == NAVIGATING:
            cells = [f.cells for f in self.frontiers]
            nearest = (np.min(np.linalg.norm(np.vstack(cells) - self.goal, axis=1)) if cells else math.inf)
            if nearest > cfg.frontier_lost_radius:
                self.say('frontier already explored, picking the next one')
                self.state = IDLE
            elif self.t - self.last_progress > cfg.progress_timeout:
                self.say('no progress, skipping this frontier')
                self.blacklist.append(tuple(self.goal))
                self.state = IDLE
            else:
                path, _ = plan(self.geom, cost, self.pose, self.goal)    # replan on the new map
                if path is None:
                    self.say('frontier became unreachable')
                    self.blacklist.append(tuple(self.goal))
                    self.state = IDLE
                else:
                    self.path = path

        elif self.state == RETURNING:
            path, _ = plan(self.geom, cost, self.pose, self.home)
            if path is not None:
                self.path = path

        if self.state == IDLE:
            self.choose_frontier(cost)

    def choose_frontier(self, cost):
        cfg = self.cfg
        candidates = [f for f in self.frontiers if not self.is_blacklisted(f.x, f.y)]
        if not candidates:
            self.empty_checks += 1
            if self.empty_checks >= cfg.empty_checks_to_finish:
                self.finish(cost)
            return
        self.empty_checks = 0

        def score(f):
            return cfg.distance_weight * math.hypot(f.x - self.pose[0], f.y - self.pose[1]) - cfg.size_weight * f.size

        for f in sorted(candidates, key=score)[:5]:
            path, _ = plan(self.geom, cost, self.pose, (f.x, f.y))
            if path is None:
                self.blacklist.append((f.x, f.y))
                continue
            self.goal = np.array([f.x, f.y])
            self.path = path
            self.state = NAVIGATING
            self.best_dist = math.inf
            self.last_progress = self.t
            self.goals_sent += 1
            self.mppi.reset()
            self.say(f'{len(candidates)} frontiers, going to ({f.x:.2f}, {f.y:.2f}), {f.size:.2f} m long')
            return

    def finish(self, cost):
        self.say(f'exploration complete after {self.goals_sent} goals, coverage {self.coverage():.0%}')
        if self.cfg.return_home:
            path, _ = plan(self.geom, cost, self.pose, self.home)
            if path is not None:
                self.goal, self.path, self.state = self.home[:2].copy(), path, RETURNING
                self.best_dist, self.last_progress = math.inf, self.t
                self.mppi.reset()
                self.say('driving back to the start')
                return
        self.state = DONE

    # ------------------------------------------------------------------ 3. act
    def control(self):
        if self.state not in (NAVIGATING, RETURNING) or self.path is None:
            return np.zeros(3)
        d = math.hypot(self.goal[0] - self.pose[0], self.goal[1] - self.pose[1])
        if d < self.best_dist - 0.05:
            self.best_dist, self.last_progress = d, self.t
        if d < self.cfg.goal_tolerance:
            if self.state == NAVIGATING:
                self.say('reached frontier')
                self.blacklist.append(tuple(self.goal))
                self.state = IDLE
                self.decide_now = True
            else:
                self.say('back at the start. Done!')
                self.state = DONE
            return np.zeros(3)
        if self.cfg.controller == 'mppi':
            return self.mppi.command(self.pose, self.path, self.goal)
        return self.simple_control()

    def simple_control(self, lookahead=0.3):
        """Drive straight at a point `lookahead` m along the path. Ignores obstacles!"""
        d = np.linalg.norm(self.path - self.pose[:2], axis=1)
        i = int(np.argmin(d))
        ahead = np.cumsum(np.r_[0, np.linalg.norm(np.diff(self.path[i:], axis=0), axis=1)])
        target = self.path[min(i + int(np.searchsorted(ahead, lookahead)), len(self.path) - 1)]
        v = target - self.pose[:2]
        dist = np.linalg.norm(v)
        speed = min(robot.MAX_SPEED, dist)                  # slow down at the end
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        world_v = v / max(dist, 1e-9) * speed
        return np.array([c * world_v[0] + s * world_v[1], -s * world_v[0] + c * world_v[1], 0.0])

    # ------------------------------------------------------------------ one step
    def step(self):
        self.sense()
        if self.decide_now or self.t >= self.next_decision:
            self.decide()
            self.decide_now = False
            self.next_decision = self.t + self.cfg.decide_period
        if self.steps % 3 == 0:     # local obstacle map for the controller (unknown = free)
            self.mppi.set_map(distance_field(self.grid.occupied(), self.geom.res, 0.4))

        cmd = self.control()
        moved = robot.step(self.true_pose, cmd, self.cfg.dt)
        if self.world.distance(moved[:2])[0] < robot.RADIUS * 0.9:
            self.bumps += 1                                  # hit something: wheels slip, no motion
            moved = self.true_pose.copy()
        if self.odom is not None:
            before = self.odom.pose.copy()
            after = self.odom.update(self.true_pose, moved)
            self.pose = robot.compose(self.pose, robot.relative(before, after))
        else:
            self.pose = moved.copy()
        self.true_pose = moved

        self.t += self.cfg.dt
        self.steps += 1
        self.trajectory.append(self.true_pose.copy())
        if self.steps % int(round(1.0 / self.cfg.dt)) == 0:
            self.coverage_history.append((self.t, self.coverage()))

    def run(self, callback=None):
        while not self.finished:
            self.step()
            if callback is not None:
                callback(self)
        return self
