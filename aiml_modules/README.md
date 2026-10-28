# HB bot: the AI/ML modules

Seven short, runnable modules that explain the "brain" of the HB bot: how
a robot with nothing but a 2D lidar maps an unknown place and finds its way
around it by itself, plus one module where it **learns** to drive.

Everything is plain Python (**numpy + matplotlib**) with a built-in 2D
simulator of the workshop arena. No ROS, no Gazebo, no GPU. It runs the same
on **Linux, macOS, Windows and a Raspberry Pi**.

| # | Module | AI/ML idea | Real robot equivalent |
|---|---|---|---|
| 1 | [Lidar perception](01_lidar_perception/) | sensing, coordinate frames, noise | `rplidar_node`, `/scan`, TF |
| 2 | [Occupancy mapping](02_occupancy_mapping/) | Bayesian inference (log-odds Bayes filter) | `slam_toolbox` map |
| 3 | [SLAM & scan matching](03_slam_scan_matching/) | probabilistic state estimation, search | `slam_toolbox` localisation |
| 4 | [Path planning](04_path_planning/) | graph search: Dijkstra, A\*, heuristics | Nav2 `planner_server` |
| 5 | [MPPI control](05_mppi_control/) | sampling-based optimisation | Nav2 `controller_server` (MPPI) |
| 6 | [Frontier exploration](06_frontier_exploration/) | autonomous agents, decision making | `frontier_explorer` node |
| 7 | [Imitation learning](07_imitation_learning/) | neural networks, supervised learning, distribution shift, DAgger | (a learned controller small enough for the ESP32) |

```
   ┌──────────────── the sense → think → act loop ────────────────┐
   │                                                              │
 lidar ─► map (2) ─► where am I? (3) ─► where to go? (6) ─► path (4) ─► wheel commands (5)
 (1)                                                                    ▲
                                          learned shortcut (7) ─────────┘
```

Each module folder has a `README.md` (the idea, the maths, where it lives in
the ROS 2 stack, exercises) and a `demo.py` that shows it working. The
algorithms themselves are in [`hbai/`](hbai/), one short file each, written
to be read.

## Setup

You need **Python 3.9 or newer**.

**Linux / macOS / Raspberry Pi** (bash or zsh):

```bash
cd aiml_modules
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python 01_lidar_perception/demo.py
```

**Windows** (PowerShell):

```powershell
cd aiml_modules
py -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python 01_lidar_perception\demo.py
```

Notes:

* **Windows:** if PowerShell refuses to run `Activate.ps1`, run
  `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` once (or use
  `.venv\Scripts\activate.bat` from Command Prompt).
* **Linux:** for plot windows you need Tk: `sudo apt install python3-tk`.
  On Ubuntu/Debian/Raspberry Pi OS you can also skip the venv and use
  `sudo apt install python3-numpy python3-matplotlib`.
* **macOS with Homebrew Python:** `brew install python-tk` for plot windows.
  The python.org installer already includes it.
* **No screen** (SSH, a headless Pi, Docker, CI): nothing to do. The demos
  notice and save PNGs/GIFs to `output/<module>/` instead of opening windows.

## Running

```bash
python 05_mppi_control/demo.py                    # opens windows
python 05_mppi_control/demo.py --save             # windows + save PNG/GIF to output/
python 05_mppi_control/demo.py --no-show          # just save (headless)
python 05_mppi_control/demo.py --help             # every module has knobs to play with
```

Check that everything works on your machine (about 1–2 minutes, no windows):

```bash
python run_all.py
python -m pytest tests          # optional: pip install pytest
```

Or with Docker, from this folder (on Windows PowerShell use `${PWD}` instead of `"$PWD"`):

```bash
docker run --rm -v "$PWD":/app -w /app python:3.12-slim sh -c "pip install -q -r requirements.txt && python run_all.py"
```

## Suggested workshop flow

About 3–4 hours, before students touch the real robot's ROS code:

1. **Modules 1–2** (45 min): what a lidar gives you, and how evidence becomes a map.
2. **Module 3** (30 min): why the robot can't trust its wheels, and how SLAM fixes it.
3. **Modules 4–5** (60 min): planning vs control. Global "where to go" vs
   local "what to do this instant".
4. **Module 6** (30 min): put it together: an autonomous agent.
5. **Module 7** (45 min): machine learning, and why robots make it hard.

Then run the same algorithms for real: `ros2 launch hb_bringup sim_autonomy.launch.py`
(see the main [README](../README.md)). The frontier code in module 6 is
literally the same file the robot uses.

## Files

```
aiml_modules/
├── README.md               this file
├── requirements.txt        numpy, matplotlib
├── run_all.py              runs every demo headless (smoke test)
├── hbai/                   the algorithms (import these in your own experiments)
├── 01_lidar_perception/    README.md + demo.py
├── ...
├── 07_imitation_learning/
├── tests/                  unit tests for hbai
└── output/                 figures and GIFs land here
```
