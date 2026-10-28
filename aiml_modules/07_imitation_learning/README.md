# Module 7 · Machine learning: learning to drive by imitation

**Question:** modules 1–6 are hand-written algorithms. Can a robot *learn*
part of its behaviour from examples instead?

## The idea

MPPI (module 5) drives well, but it needs the map, a planned path and
~2–5 ms of computation per step. Can a small **neural network** learn to do
the same from just the lidar scan and the goal direction, fast enough for a
microcontroller?

We use **imitation learning**: the expert demonstrates, the student copies.

```
features (39 numbers)                       network                 output
  36 lidar sectors: nearest obstacle    →   39 → 64 → 64 → 2   →   (vx, vy)
  goal direction + distance                 (6,850 parameters)
```

## Step 1 · Behaviour cloning (supervised learning)

Record the expert driving ~40 trips: thousands of `(features, expert command)`
pairs. Train the network to minimise the **mean squared error** between its
command and the expert's, with backpropagation and the Adam optimiser
(all in `hbai/learning.py`, about 60 lines of numpy).

This is ordinary supervised learning, the same as recognising digits.

## Step 2 · The catch: distribution shift

The training data only shows the **expert's** situations: always centred on
the path, never too close to a wall. When the *network* drives, it makes a
small error, ends up somewhere the expert never was, has no idea what to do
there, makes a bigger error... and crashes or gets lost.

This is the most important lesson about ML in robotics: **the robot's own
actions change the data it sees.**

## Step 3 · DAgger (Dataset Aggregation)

Let the **network** drive, but at every step ask the expert "what would you
do here?". Add those labels to the data and retrain. Now the network has
examples of how to recover from its own mistakes. Repeat a few times.

## Typical result (default settings)

| policy | success | thinking time |
|---|---|---|
| expert (MPPI) | ~100% | ~2 ms |
| cloned network | ~40–60% | ~0.06 ms |
| DAgger network | ~75–90% | ~0.06 ms |

Your numbers vary with the random seed: try `--seed 1`, `--seed 2`.

## Where this would fit in the real robot

The ROS stack doesn't use a learned controller, deliberately: MPPI is
predictable and has no training data to collect. But the student network is
small enough (6,850 weights ≈ 27 kB as float32) to run **on the ESP32**,
which makes it a fallback that keeps the robot moving safely if the Pi or
Nav2 stops. The trained weights are saved to
`output/07_imitation_learning/student_weights.npz`.

## Run it

```bash
python 07_imitation_learning/demo.py                     # ~1-2 minutes
python 07_imitation_learning/demo.py --dagger-rounds 0   # cloning only
python 07_imitation_learning/demo.py --train-trips 10
python 07_imitation_learning/demo.py --hidden 8
```

## What to look for

* **Training curves:** error drops fast, then flattens. The dotted validation
  error tells you if the network is memorising instead of learning.
* **Results bar:** the gap between cloned and DAgger is distribution shift.
* **Thinking time:** the network is ~30× faster than its teacher.
* **Trips:** the cloned network drifts, gets near a wall, and doesn't know
  how to come back.

## Exercises

1. `--train-trips 10` vs `80`. Does more expert data fix distribution shift
   on its own?
2. `--hidden 8` and `--hidden 256`. Watch the validation curve. Which one overfits?
3. Remove the lidar from the features (keep only the goal) in `features()`.
   What does the network learn? Why does it crash?
4. The network never turns (wz = 0). Add wz as a third output. What else has
   to change?
5. **Bonus:** write the forward pass in C for the ESP32. It's three matrix
   multiplies, two ReLUs and a tanh. How many microseconds does it take?
6. **Discussion:** reinforcement learning would let the robot learn *without*
   an expert, by trial and error. Why is that much harder on a real robot?
