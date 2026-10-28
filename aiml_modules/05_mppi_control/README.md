# Module 5 · MPPI control (sampling-based optimisation)

**Question:** the planner gave a path. What velocity do we send to the
wheels *right now*, 10 times a second, without hitting anything?

## The idea

**Imagine many futures, keep a soft best.**

Every 0.1 s, MPPI (Model Predictive Path Integral):

1. **Samples** K = 400 random control sequences: small random changes to its
   current plan of (vx, vy, wz) for the next 2 s.
2. **Simulates** where each would take the robot (the "model": for a
   holonomic robot, just rotate the velocity into the world frame and add it up).
3. **Scores** each trajectory with *critics*: hit a wall? close to a wall?
   far from the path? not making progress? spinning for no reason?
4. **Averages** all sequences, weighted by `exp(−cost / temperature)`.
   Good sequences dominate; bad ones contribute almost nothing.
5. **Acts:** sends the first command, shifts the plan, repeats.

No gradients, no training: just lots of cheap guesses, done in parallel
(numpy here; SIMD on the robot).

## The maths

```
u_k = u_plan + ε_k,             ε_k ~ Normal(0, σ)               k = 1..K
x_k = rollout(x_now, u_k)                                        (motion model)
c_k = Σ critics(x_k, u_k)
w_k = exp(−(c_k − min c) / λ) / Σ_j exp(−(c_j − min c) / λ)       (soft-min)
u_plan ← Σ_k w_k · u_k
```

The **temperature λ** matters:

* too high → all weights equal → the random noise averages out → the robot
  crawls (try `--temperature 1.0`);
* too low → only the single best sample counts → jittery motion.

The demo shows *effective samples* (`1 / Σ w²`): how many samples really
shape the decision.

## Where this lives in the real robot

| Here (`hbai/mppi.py`) | Real robot (`nav2_params.yaml`, `FollowPath`) |
|---|---|
| `samples=400`, `horizon=20`, `dt=0.1` | `batch_size: 1000`, `time_steps: 20`, `model_dt: 0.1` |
| omni rollout (vx, vy, wz) | `motion_model: "Omni"` |
| `temperature` | `temperature` |
| critics: collision, obstacle | `CostCritic` |
| critics: path, follow | `PathAlignCritic`, `PathFollowCritic` |
| critic: goal | `GoalCritic` |
| critic: twirl | `TwirlingCritic` |

The ROS stack first used a different controller (DWB). It sampled only a few
constant velocities and got stuck dithering in a doorway. MPPI's sampled
*sequences* fixed it.

## Run it

```bash
python 05_mppi_control/demo.py
python 05_mppi_control/demo.py --temperature 1.0
python 05_mppi_control/demo.py --temperature 0.005
python 05_mppi_control/demo.py --twirl 0
python 05_mppi_control/demo.py --samples 30
```

## What to look for

* The fan of sampled futures in the animation; brighter means higher weight.
  The green line is the averaged plan.
* The robot **strafes** around corners without turning (vx and vy change
  together, wz stays near 0). That's the holonomic advantage.
* Thinking time per step, printed at the end.

## Exercises

1. Run with `--temperature` values 0.005, 0.05, 0.3 and 1.0. Plot time to
   goal against temperature.
2. `--twirl 0`: why does the robot start to spin, even though spinning
   doesn't help it get anywhere?
3. Lower `--samples` until it fails. Why do fewer samples fail?
4. Add a new critic in `MPPI.command()` that prefers driving forward
   (penalise vy). When would a real robot want that? (Hint: where is the lidar blind?)
5. **Bonus:** MPPI cost per step grows with K × T. Measure it, and estimate
   whether K = 1000, T = 20 fits in 100 ms on a Raspberry Pi 4 (≈ 4× slower
   than a laptop).
