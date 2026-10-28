// ============================================================================
//  HB bot ESP32 base controller: configuration
//  Edit this file to match YOUR wiring, motors and encoders.
// ============================================================================
#pragma once

// Wheel order everywhere (firmware, ROS, URDF):
//   0 = left  (front-left,  60 deg)
//   1 = right (front-right, -60 deg)
//   2 = back  (180 deg)
#define NUM_WHEELS 3

// ---------------------------------------------------------------- serial link
#define SERIAL_BAUD 115200

// ---------------------------------------------------------------- motor driver
// TB6612FNG wiring (also works for L298N: PWM -> ENA/ENB, IN1/IN2 -> IN1..IN4).
//                          left right back
const int MOTOR_PWM[NUM_WHEELS] = {25, 33, 22};
const int MOTOR_IN1[NUM_WHEELS] = {26, 32, 21};
const int MOTOR_IN2[NUM_WHEELS] = {27, 23, 19};
#define MOTOR_STBY 4            // TB6612 STBY pin; set to -1 if tied to 3.3V (or using L298N)

#define PWM_FREQ 20000          // Hz, above hearing range
#define PWM_BITS 8              // duty 0..255
#define PWM_MAX 255
#define PWM_MIN 0               // raise (e.g. 25) if motors hum but don't turn at low speed

// Flip to -1 if a wheel turns the wrong way for a POSITIVE command.
// Positive = clockwise when you look at the wheel from OUTSIDE the robot.
const int MOTOR_DIR[NUM_WHEELS] = {1, 1, 1};

// ---------------------------------------------------------------- encoders
// Quadrature hall encoders. GPIO 34/35/36/39 are input-only and have NO internal
// pull-ups: add 10k resistors to 3.3V if your encoder board doesn't have them.
// Power the encoders from 3.3V so their outputs are 3.3V logic.
const int ENC_A[NUM_WHEELS] = {34, 36, 13};
const int ENC_B[NUM_WHEELS] = {35, 39, 18};

// Flip to -1 if the reported wheel position goes DOWN for a positive command.
const int ENC_DIR[NUM_WHEELS] = {1, 1, 1};

// Counts per WHEEL revolution = pulses per motor rev x 4 (quadrature) x gear ratio.
// Example: JGA25-370 with 11 PPR encoder and 1:30 gearbox -> 11 * 4 * 30 = 1320
#define ENCODER_PPR 11
#define GEAR_RATIO 30.0f
#define COUNTS_PER_REV (ENCODER_PPR * 4.0f * GEAR_RATIO)

// ---------------------------------------------------------------- control
#define CONTROL_HZ 100          // PID loop rate
#define TELEMETRY_HZ 50         // S,... messages to the Raspberry Pi
#define CMD_TIMEOUT_MS 500      // stop if no V command for this long
#define TEST_TIMEOUT_MS 3000    // M (raw PWM test) commands last this long
#define MAX_WHEEL_SPEED 30.0f   // rad/s, targets are clamped to this
#define SPEED_FILTER 0.5f       // 0..1, low-pass on measured speed (1 = no filter)

// PID gains per wheel (output is PWM duty). Tune with the P command, e.g. "P,8,40,0,8".
//   KF  feed-forward: PWM per rad/s. Measure: run "M,150,0,0", read speed v -> KF = 150 / v
#define PID_KP 8.0f
#define PID_KI 40.0f
#define PID_KD 0.0f
#define PID_KF 8.0f

#define STATUS_LED 2            // on-board LED: lit while receiving commands (-1 to disable)
