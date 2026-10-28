"""hbai: the "brain" of the HB bot, as small, readable Python.

Every algorithm the real robot runs (through ROS 2) is re-implemented here
in plain numpy, together with a tiny 2D simulator of the workshop arena.
No ROS, no Gazebo: it runs anywhere Python runs.

    world.py          the arena (same as hb_gazebo/worlds/arena.sdf) + exact ray casting
    lidar.py          2D lidar model
    robot.py          holonomic motion, odometry with drift
    gridmap.py        grid geometry helpers (metres <-> cells)
    mapping.py        occupancy grid mapping (Bayes filter in log-odds)
    scan_matching.py  correlative scan matching (the core of slam_toolbox)
    planning.py       costmaps + A* / Dijkstra (what Nav2's planner does)
    mppi.py           MPPI controller, omni motion model (Nav2's controller)
    frontiers.py      frontier detection (same code as the ROS explorer)
    exploration.py    the full sense -> map -> decide -> plan -> act loop
    learning.py       a small neural network trained by imitation (numpy only)
    viz.py            plotting helpers that work with or without a screen
"""
