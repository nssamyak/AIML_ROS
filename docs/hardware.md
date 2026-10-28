# Hardware guide

How the physical HB bot is built and brought up: parts, wiring, ESP32
firmware, Raspberry Pi setup, and the calibration steps you need before you
let it drive itself.

## 1. Architecture

```
            Raspberry Pi 4/5  (Ubuntu 24.04 + ROS 2 Jazzy)
 ┌───────────────────────────────────────────────────────────────┐
 │ rplidar_node ──/scan──► slam_toolbox ──/map──► frontier_explorer
 │                              │                       │ goal
 │                              ▼                       ▼
 │                       TF map→odom              Nav2 (planner,
 │                                                 controller, BT)
 │                                                      │ /cmd_vel
 │  robot_state_publisher (URDF → TF)                   ▼
 │                                                 kiwi_drive
 │                      /odom, TF odom→base ◄──────┘     │ wheel targets
 │                                                       ▼ (rad/s)
 │                                                 esp32_bridge
 └───────────────────────────────────────────────────────┬───────┘
                                              USB serial │ V,… / S,…
                                   ┌─────────────────────▼───────┐
                                   │ ESP32: 3 × (encoder → PID → PWM)
                                   │        watchdog: stop if Pi goes quiet
                                   └──────┬──────────┬──────────┬┘
                                       TB6612     TB6612       │
                                     M0 (left) M1 (right)  M2 (back)
```

The split is deliberate:

* **ESP32**: hard real-time work. It counts encoder pulses, runs a PID loop per
  wheel at 100 Hz, and stops the motors if the Pi goes quiet for 0.5 s.
* **Raspberry Pi**: everything else, all in ROS 2. The Pi never touches a GPIO.

The `kiwi_drive` node, SLAM, Nav2 and the explorer are **the exact same code**
in Gazebo and on the robot. Only the bottom layer is swapped: Gazebo's
ros2_control in simulation, `esp32_bridge` on the robot. Both serve the same
two topics: `/wheel_velocity_controller/commands` and `/joint_states`.

## 2. Parts list (per robot)

| Part | Suggested | Notes |
|---|---|---|
| Computer | Raspberry Pi 4 (4 GB) or Pi 5 | Pi 5 gives more headroom for Nav2 |
| Storage | 32 GB+ microSD, A2 class | Ubuntu Server 24.04 LTS 64-bit |
| Microcontroller | ESP32 DevKit V1 (ESP32-WROOM-32) | the pin map assumes WROOM, not WROVER |
| Motor drivers | 2 × TB6612FNG | L298N also works, but it wastes ~2 V |
| Motors | 3 × geared DC motors **with quadrature hall encoders** | N20 (small) or JGA25-370 |
| Wheels | 3 × omni wheels | match `wheel_radius` in the config |
| Lidar | Slamtec RPLidar A1 / C1 | any 2D lidar with a ROS 2 driver works |
| Battery | 2S or 3S LiPo / Li-ion pack | with a fuse and a switch |
| 5 V supply | 5 V / 3 A buck converter (5 A for Pi 5) | powers the Pi; don't use the motor rail |
| Misc | standoffs, jumper wires, 4 × 10 kΩ resistors | pull-ups for encoder pins 34–39 |

## 3. Match the software to YOUR robot

The provided model is the small eYRC holonomic bot: 19 mm wheel radius,
52 mm from centre to wheel. A Pi + lidar build will usually be bigger.
Measure your robot and update **three places** with the same numbers:

| What | URDF `hb_description/urdf/hb_bot.urdf.xacro` | `hb_control/config/kiwi_drive.yaml` | Nav2 `hb_navigation/config/nav2_params.yaml` |
|---|---|---|---|
| Wheel radius | `wheel_radius` | `wheel_radius` | – |
| Centre → wheel distance | `wheel_base_radius` | `wheel_base_radius` | – |
| Wheel angles | `*_wheel_angle` | `wheel_angles_deg` | – |
| Robot size | `chassis_radius` | – | `robot_radius` (both costmaps) |
| Lidar height | `standoff_height` etc. | – | – |

In the firmware `config.h`, set `ENCODER_PPR` and `GEAR_RATIO` for your motors.

## 4. Wiring

Default pins from `firmware/esp32_kiwi_base/config.h`:

| Signal | Left wheel (0) | Right wheel (1) | Back wheel (2) |
|---|---|---|---|
| PWM (TB6612 PWMx) | GPIO 25 | GPIO 33 | GPIO 22 |
| IN1 (TB6612 xIN1) | GPIO 26 | GPIO 32 | GPIO 21 |
| IN2 (TB6612 xIN2) | GPIO 27 | GPIO 23 | GPIO 19 |
| Encoder A | GPIO 34 | GPIO 36 (VP) | GPIO 13 |
| Encoder B | GPIO 35 | GPIO 39 (VN) | GPIO 18 |

* TB6612 **STBY** → GPIO 4 on both boards (or tie it to 3.3 V and set `MOTOR_STBY -1`).
* TB6612 **VM** → battery +, **VCC** → ESP32 3.3 V, **GND** → common ground.
* Encoders: power them from **3.3 V**, so their outputs never exceed 3.3 V.
  GPIO 34/35/36/39 have no internal pull-ups, so add 10 kΩ from each to 3.3 V
  unless your encoder board already has them.
* ESP32 → Pi with a USB cable. That cable also powers the ESP32.
* Lidar → Pi USB.
* **Common ground** between the battery, motor drivers, ESP32 and Pi is mandatory.

Avoid the ESP32 strapping pins (0, 2, 5, 12, 15) for encoders. A wrong level
there at boot stops the board from starting.

## 5. ESP32 firmware

Arduino IDE: install the "esp32 by Espressif" board package, open
`firmware/esp32_kiwi_base/esp32_kiwi_base.ino`, pick **ESP32 Dev Module**, upload.

PlatformIO:

```bash
cd firmware && pio run -t upload && pio device monitor
```

Talk to it from the Serial Monitor (115200 baud, newline line ending):

| Type | Effect |
|---|---|
| `?` | print configuration and PID gains |
| `M,120,0,0` | left wheel at PWM 120 for 3 s (open loop, no PID) |
| `V,5,0,0` | left wheel at 5 rad/s with PID; stops after 0.5 s unless repeated |
| `P,8,40,0,8` | set kp, ki, kd, kf |
| `R` | reset encoder positions |

It streams `S,<ms>,<pos0>,<pos1>,<pos2>,<vel0>,<vel1>,<vel2>` at 50 Hz.

## 6. Calibration, in this order

Prop the robot up so the wheels spin freely in the air.

1. **Motor direction.** Send `M,120,0,0`, then `M,0,120,0`, then `M,0,0,120`.
   Each wheel must turn **clockwise when you look at it from outside the robot**.
   If one doesn't, flip its `MOTOR_DIR` (or swap its two motor wires).
2. **Encoder direction.** While a wheel runs with a positive command, its
   position in the `S,` lines must go **up**. If it goes down, flip `ENC_DIR`.
3. **Counts per revolution.** Send `R`, turn one wheel exactly 10 times by
   hand, and check its position reads about 62.8 rad (10 × 2π). If not, fix
   `ENCODER_PPR` / `GEAR_RATIO`.
4. **Feed-forward.** Send `M,150,0,0` and note the steady speed v (rad/s).
   Set `PID_KF = 150 / v`. This alone gets you close to the right speed.
5. **PID.** Send `V,8,8,8` repeatedly (or use `wheel_test`) and watch the speeds.
   Raise `KP` until it reacts quickly without oscillating, then `KI` to remove
   the remaining error. Save the gains in `config.h`.
6. **Whole robot, on the floor:**
   ```bash
   ros2 launch hb_bringup robot.launch.py
   ros2 run hb_control wheel_test --motions
   ```
   It should go forward, backward, strafe left, strafe right, then rotate CCW
   and CW. If forward makes it go sideways, a wheel is wired to the wrong
   channel (check the wheel order: left, right, back).
7. **Odometry.** Drive 1 m forward with teleop and compare with a tape measure.
   If it's off by a constant ratio, adjust `wheel_radius`. Rotate 10 turns in
   place: if the heading is off, adjust `wheel_base_radius`.
8. **Lidar orientation.** In RViz, with fixed frame `base_footprint`, put a box
   in front of the robot. It must appear at +x (in front). If it appears behind,
   relaunch with `lidar_yaw:=3.14159`.

## 7. Raspberry Pi setup

1. Flash **Ubuntu Server 24.04 LTS (64-bit)** with Raspberry Pi Imager. Set up
   Wi-Fi and SSH in the imager.
2. Install ROS 2 Jazzy (ros-base is enough on the robot):
   ```bash
   sudo apt update && sudo apt install -y software-properties-common curl
   sudo add-apt-repository universe
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list
   sudo apt update && sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep
   ```
3. Build the workspace. `--skip-keys` leaves the simulator out of the Pi install:
   ```bash
   cd ~/Athena/ros2_ws
   sudo rosdep init; rosdep update
   rosdep install --from-paths src -y --ignore-src --skip-keys "ros_gz_sim ros_gz_bridge gz_ros2_control rviz2 joint_state_publisher_gui"
   colcon build --symlink-install
   echo "source ~/Athena/ros2_ws/install/setup.bash" >> ~/.bashrc
   ```
   (`hb_gazebo` builds fine without Gazebo installed: it's only launch files and a world.)
4. Serial permissions: `sudo usermod -aG dialout $USER`, then log out and back in.
5. Stable device names. The ESP32 and the RPLidar often use the same USB chip
   (CP2102), so use their stable paths:
   ```bash
   ls -l /dev/serial/by-id/
   ros2 launch hb_bringup robot.launch.py \
       esp32_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_..._0001-if00-port0 \
       lidar_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_..._0002-if00-port0
   ```
6. Laptop + Pi on the same network: set the same `export ROS_DOMAIN_ID=<n>` on
   both (use a different number per team in the workshop). Run RViz on the
   laptop (`ros2 launch hb_bringup rviz.launch.py`), not on the Pi.

## 8. Running on the robot

```bash
# on the Pi
ros2 launch hb_bringup robot_autonomy.launch.py          # explores by itself
# on the laptop
ros2 launch hb_bringup rviz.launch.py
# when it's done, save the map (on either machine)
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_room
```

Safety: always test new code with the wheels in the air first, and keep a
hand on the power switch. If the Pi crashes, the ESP32 watchdog stops the
motors within 0.5 s.
