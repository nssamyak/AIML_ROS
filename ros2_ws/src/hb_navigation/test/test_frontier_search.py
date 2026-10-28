import numpy as np

from hb_navigation.frontier_search import cluster_cells, dilate, find_frontiers, frontier_mask

FREE, WALL, UNKNOWN = 0, 100, -1


def test_frontier_is_free_next_to_unknown():
    grid = np.full((5, 6), FREE)
    grid[:, 4:] = UNKNOWN
    mask = frontier_mask(grid)
    assert mask[:, 3].all()          # column right before the unknown area
    assert not mask[:, :3].any()
    assert not mask[:, 4:].any()     # unknown cells are never frontiers


def test_no_frontier_in_fully_known_map():
    grid = np.full((10, 10), FREE)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = WALL
    assert not frontier_mask(grid).any()
    assert find_frontiers(grid, 0.05, 0.0, 0.0) == []


def test_clearance_removes_cells_near_walls():
    grid = np.full((10, 10), FREE)
    grid[:, 8:] = UNKNOWN
    grid[0:3, 6] = WALL
    mask = frontier_mask(grid, clearance_cells=1)
    assert not mask[0:4, 7].any()    # beside the wall
    assert mask[5:, 7].all()


def test_dilate():
    m = np.zeros((5, 5), bool)
    m[2, 2] = True
    assert dilate(m, 1).sum() == 9


def test_clusters_split_by_gap():
    m = np.zeros((10, 10), bool)
    m[0:3, 0] = True
    m[6:9, 9] = True
    sizes = sorted(len(c) for c in cluster_cells(m))
    assert sizes == [3, 3]


def test_find_frontiers_world_coordinates():
    res = 0.1
    grid = np.full((20, 20), FREE)
    grid[:, 15:] = UNKNOWN                     # unknown on the right
    frontiers = find_frontiers(grid, res, origin_x=-1.0, origin_y=-1.0, clearance=0.0, min_size=0.5)
    assert len(frontiers) == 1
    f = frontiers[0]
    assert abs(f.x - (-1.0 + 14.5 * res)) < 1e-9   # column 14, cell centre
    assert -1.0 < f.y < 1.0
    assert abs(f.size - 20 * res) < 1e-9


def test_small_frontiers_ignored():
    grid = np.full((20, 20), FREE)
    grid[0, 0] = UNKNOWN
    assert find_frontiers(grid, 0.05, 0, 0, clearance=0.0, min_size=0.25) == []
