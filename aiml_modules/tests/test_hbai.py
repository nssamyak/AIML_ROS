"""Unit tests for the hbai algorithms.   python -m pytest tests"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import pytest

from hbai import robot
from hbai.frontiers import find_frontiers
from hbai.gridmap import GridGeom
from hbai.learning import MLP, features
from hbai.lidar import Lidar, scan_to_points
from hbai.mapping import OccupancyGrid
from hbai.mppi import MPPI
from hbai.planning import LETHAL, costmap, distance_field, plan, path_length
from hbai.scan_matching import likelihood_field, match
from hbai.world import START, World, arena


@pytest.fixture
def box_world():
    """A 2 x 2 m room centred on the origin."""
    walls = [(0, 1, 2, 0.02, 0), (0, -1, 2, 0.02, 0), (1, 0, 0.02, 2, 0), (-1, 0, 0.02, 2, 0)]
    return World(walls, [(0.5, 0.5, 0.1)], xlim=(-1.2, 1.2), ylim=(-1.2, 1.2))


def test_raycast_exact(box_world):
    r = box_world.raycast((0, 0), np.array([0.0, np.pi / 2, np.pi]))
    assert r == pytest.approx([0.99, 0.99, 0.99])
    # towards the circle at (0.5, 0.5) r=0.1
    d = box_world.raycast((0, 0), np.array([np.pi / 4]))[0]
    assert d == pytest.approx(np.hypot(0.5, 0.5) - 0.1)


def test_distance_and_inside(box_world):
    assert box_world.distance([[0, 0]])[0] == pytest.approx(np.hypot(0.5, 0.5) - 0.1)
    assert box_world.distance([[0.5, 0.5]])[0] == 0.0
    assert box_world.inside([[0.5, 0.5], [0, 0]]).tolist() == [True, False]


def test_scan_to_points_frames():
    angles = np.array([0.0, np.pi / 2])
    pts = scan_to_points(np.array([1.0, 2.0]), angles, pose=(1.0, 1.0, np.pi / 2))
    assert pts == pytest.approx(np.array([[1.0, 2.0], [-1.0, 1.0]]))


def test_compose_relative_inverse():
    a = np.array([0.3, -0.2, 0.7])
    b = np.array([1.1, 0.4, -2.0])
    assert robot.compose(a, robot.relative(a, b)) == pytest.approx(b)


def test_mapping_marks_wall_and_free(box_world):
    geom = GridGeom(box_world.xlim, box_world.ylim, 0.05)
    grid = OccupancyGrid(geom)
    lidar = Lidar(noise_std=0.0)
    for _ in range(5):
        grid.update((0, 0, 0), lidar.angles, lidar.scan(box_world, (0, 0, 0)))
    ros = grid.to_ros()
    assert ros[geom.to_cell(np.array([0.0, 0.0]))] == 0
    assert ros[geom.to_cell(np.array([0.99, 0.0]))] == 100
    assert ros[geom.to_cell(np.array([1.15, 0.0]))] == -1          # behind the wall


def test_scan_matching_recovers_offset(box_world):
    geom = GridGeom(box_world.xlim, box_world.ylim, 0.05)
    grid = OccupancyGrid(geom)
    lidar = Lidar(noise_std=0.0)
    true = np.array([0.1, -0.2, 0.0])
    for _ in range(3):
        grid.update(true, lidar.angles, lidar.scan(box_world, true))
    pts = scan_to_points(lidar.scan(box_world, true), lidar.angles)
    found, _ = match(likelihood_field(grid), geom, pts, true + [0.06, -0.05, 0.05])
    assert np.hypot(*(found[:2] - true[:2])) < 0.03
    assert abs(found[2] - true[2]) < 0.03


def test_astar_matches_dijkstra_and_avoids_walls():
    world = arena()
    geom = GridGeom(world.xlim, world.ylim, 0.05)
    cost = costmap(world.occupancy(geom), geom.res)
    p_a, exp_a = plan(geom, cost, START, (1.65, 0.55), heuristic_weight=1)
    p_d, exp_d = plan(geom, cost, START, (1.65, 0.55), heuristic_weight=0)
    assert path_length(p_a) == pytest.approx(path_length(p_d), rel=1e-6)
    assert len(exp_a) < len(exp_d)
    assert world.distance(p_a).min() > robot.RADIUS


def test_costmap_lethal_near_walls():
    occ = np.zeros((20, 20), bool)
    occ[10, :] = True
    cost = costmap(occ, 0.05)
    assert cost[10, 5] == LETHAL and cost[11, 5] == LETHAL
    assert cost[0, 5] == 1.0


def test_frontiers_found_in_partial_map(box_world):
    geom = GridGeom(box_world.xlim, box_world.ylim, 0.05)
    grid = OccupancyGrid(geom)
    lidar = Lidar(noise_std=0.0)
    front = np.abs(lidar.angles) < np.pi / 2                 # only look forward: the back stays unknown
    grid.update((0, 0, 0), lidar.angles[front], lidar.scan(box_world, (0, 0, 0))[front])
    fr = find_frontiers(grid.to_ros(), geom.res, geom.x0, geom.y0, clearance=0.0)
    assert len(fr) >= 1
    assert any(abs(f.x) < 0.1 and f.size > 1.0 for f in fr)  # the long frontier along x = 0, bordering the unseen back


def test_mppi_drives_to_goal_in_open_room(box_world):
    geom = GridGeom(box_world.xlim, box_world.ylim, 0.05)
    occ = box_world.occupancy(geom)
    ctrl = MPPI(geom, distance_field(occ, geom.res, 0.4), seed=0)
    path = np.linspace([-0.6, -0.6], [0.6, -0.6], 30)
    pose = np.array([-0.6, -0.6, 0.0])
    for _ in range(200):
        pose = robot.step(pose, ctrl.command(pose, path, path[-1]), 0.1)
    assert np.hypot(*(pose[:2] - path[-1])) < 0.1


def test_mlp_learns_a_simple_function():
    rng = np.random.default_rng(0)
    X = rng.uniform(-1, 1, (512, 3))
    Y = np.tanh(np.stack([X[:, 0] - X[:, 1], 0.5 * X[:, 2]], axis=1))
    model = MLP((3, 16, 2), seed=0)
    hist = model.fit(X, Y, epochs=150, lr=3e-3)
    assert hist['train'][-1] < 0.1 * hist['train'][0]


def test_features_shape_and_range():
    f = features(np.full(360, 3.0), (0, 0, 0), (5.0, 0.0))
    assert f.shape == (39,)
    assert np.all(np.abs(f) <= 1.0)
    assert f[36] == pytest.approx(1.0)      # goal straight ahead
