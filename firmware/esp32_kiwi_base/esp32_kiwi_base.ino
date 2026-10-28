// ============================================================================
//  HB bot ESP32 base controller
//
//  Job: keep each of the 3 wheels at the speed the Raspberry Pi asks for,
//  and report back how far and how fast each wheel actually turned.
//  All the robot maths (kinematics, odometry, navigation) lives on the Pi.
//
//  Serial protocol, one text line per message, 115200 baud:
//    Pi -> ESP32
//      V,<w0>,<w1>,<w2>        wheel speed targets in rad/s (send at least every 0.5 s)
//      M,<p0>,<p1>,<p2>        raw motor PWM -255..255, for testing from the Serial Monitor
//      P,<kp>,<ki>,<kd>,<kf>   set PID gains (all wheels)
//      R                       reset encoder positions
//      ?                       print the configuration
//    ESP32 -> Pi
//      S,<ms>,<p0>,<p1>,<p2>,<v0>,<v1>,<v2>   wheel angle (rad) and speed (rad/s)
//      #<text>                                info / errors
//
//  Wheel order: 0 = left (front-left), 1 = right (front-right), 2 = back.
//  Positive speed = wheel turns clockwise seen from outside the robot.
// ============================================================================
#include <Arduino.h>
#include "config.h"

// ---------------------------------------------------------------- encoders
volatile int32_t encCount[NUM_WHEELS] = {0};
volatile uint8_t encState[NUM_WHEELS] = {0};

// Quadrature decoding table: index = (previous AB << 2) | current AB
static const int8_t QUAD_TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void IRAM_ATTR encoderISR(void *arg) {
  const int i = (int)(intptr_t)arg;
  const uint8_t s = (digitalRead(ENC_A[i]) << 1) | digitalRead(ENC_B[i]);
  encCount[i] += QUAD_TABLE[(encState[i] << 2) | s];
  encState[i] = s;
}

// ---------------------------------------------------------------- state
enum Mode { MODE_STOP, MODE_VELOCITY, MODE_PWM_TEST };
Mode mode = MODE_STOP;

float targetSpeed[NUM_WHEELS] = {0};     // rad/s
int testPwm[NUM_WHEELS] = {0};
float measuredSpeed[NUM_WHEELS] = {0};   // rad/s, filtered
int32_t lastCount[NUM_WHEELS] = {0};
uint32_t lastCommandMs = 0;

struct Pid {
  float kp = PID_KP, ki = PID_KI, kd = PID_KD, kf = PID_KF;
  float integral = 0, prevError = 0;
  void reset() { integral = 0; prevError = 0; }
};
Pid pid[NUM_WHEELS];

const float RAD_PER_COUNT = 2.0f * PI / COUNTS_PER_REV;
const uint32_t CONTROL_PERIOD_US = 1000000UL / CONTROL_HZ;
const uint32_t TELEMETRY_PERIOD_US = 1000000UL / TELEMETRY_HZ;

// ---------------------------------------------------------------- helpers
static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline int clampi(int x, int lo, int hi) { return x < lo ? lo : (x > hi ? hi : x); }

// ---------------------------------------------------------------- motors
void setupPwm(int i) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(MOTOR_PWM[i], PWM_FREQ, PWM_BITS);
#else
  ledcSetup(i, PWM_FREQ, PWM_BITS);
  ledcAttachPin(MOTOR_PWM[i], i);
#endif
}

void writePwm(int i, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PWM[i], duty);
#else
  ledcWrite(i, duty);
#endif
}

// pwm in -PWM_MAX..PWM_MAX, positive = clockwise seen from outside
void setMotor(int i, int pwm) {
  pwm = clampi(pwm * MOTOR_DIR[i], -PWM_MAX, PWM_MAX);
  int duty = abs(pwm);
  if (duty > 0 && PWM_MIN > 0) duty = PWM_MIN + (long)duty * (PWM_MAX - PWM_MIN) / PWM_MAX;
  digitalWrite(MOTOR_IN1[i], pwm > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_IN2[i], pwm < 0 ? HIGH : LOW);
  writePwm(i, duty);
}

void stopAll() {
  for (int i = 0; i < NUM_WHEELS; i++) {
    targetSpeed[i] = 0;
    testPwm[i] = 0;
    pid[i].reset();
    setMotor(i, 0);
  }
  mode = MODE_STOP;
}

// ---------------------------------------------------------------- control loop
int32_t readCount(int i) {
  return ENC_DIR[i] * encCount[i];   // 32-bit aligned reads are atomic on the ESP32
}

void controlStep(float dt) {
  for (int i = 0; i < NUM_WHEELS; i++) {
    const int32_t c = readCount(i);
    const float raw = (c - lastCount[i]) * RAD_PER_COUNT / dt;
    lastCount[i] = c;
    measuredSpeed[i] += SPEED_FILTER * (raw - measuredSpeed[i]);
  }

  // Watchdog: if the Pi goes quiet, stop. Never drive blind.
  const uint32_t timeout = (mode == MODE_PWM_TEST) ? TEST_TIMEOUT_MS : CMD_TIMEOUT_MS;
  if (mode != MODE_STOP && millis() - lastCommandMs > timeout) {
    stopAll();
    Serial.println("#command timeout, motors stopped");
  }

  for (int i = 0; i < NUM_WHEELS; i++) {
    if (mode == MODE_PWM_TEST) {
      setMotor(i, testPwm[i]);
      continue;
    }
    if (mode != MODE_VELOCITY || targetSpeed[i] == 0.0f) {
      pid[i].reset();
      setMotor(i, 0);
      continue;
    }
    // PID + feed-forward on wheel speed
    Pid &p = pid[i];
    const float error = targetSpeed[i] - measuredSpeed[i];
    const float derivative = (error - p.prevError) / dt;
    p.prevError = error;
    float out = p.kf * targetSpeed[i] + p.kp * error + p.integral + p.kd * derivative;
    // Anti-windup: only integrate while the output isn't saturated
    if (fabsf(out) < (float)PWM_MAX) {
      p.integral += p.ki * error * dt;
      p.integral = clampf(p.integral, -PWM_MAX, PWM_MAX);
    }
    setMotor(i, (int)clampf(out, -PWM_MAX, PWM_MAX));
  }

#if STATUS_LED >= 0
  digitalWrite(STATUS_LED, mode == MODE_STOP ? LOW : HIGH);
#endif
}

void sendTelemetry() {
  char line[128];
  int n = snprintf(line, sizeof(line), "S,%lu", (unsigned long)millis());
  for (int i = 0; i < NUM_WHEELS; i++)
    n += snprintf(line + n, sizeof(line) - n, ",%.4f", readCount(i) * RAD_PER_COUNT);
  for (int i = 0; i < NUM_WHEELS; i++)
    n += snprintf(line + n, sizeof(line) - n, ",%.3f", measuredSpeed[i]);
  Serial.println(line);
}

// ---------------------------------------------------------------- serial commands
void printConfig() {
  Serial.printf("#counts/rev %.1f, control %d Hz, telemetry %d Hz, timeout %d ms\n",
                COUNTS_PER_REV, CONTROL_HZ, TELEMETRY_HZ, CMD_TIMEOUT_MS);
  Serial.printf("#PID kp %.3f ki %.3f kd %.3f kf %.3f\n", pid[0].kp, pid[0].ki, pid[0].kd, pid[0].kf);
}

// Parse up to `count` comma separated floats after the command letter
int parseFloats(char *args, float *out, int count) {
  int n = 0;
  for (char *tok = strtok(args, ","); tok && n < count; tok = strtok(nullptr, ","))
    out[n++] = atof(tok);
  return n;
}

void handleLine(char *line) {
  const char cmd = line[0];
  char *args = (line[1] == ',') ? line + 2 : line + 1;
  float v[4];

  switch (cmd) {
    case 'V':
      if (parseFloats(args, v, NUM_WHEELS) == NUM_WHEELS) {
        for (int i = 0; i < NUM_WHEELS; i++)
          targetSpeed[i] = clampf(v[i], -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
        mode = MODE_VELOCITY;
        lastCommandMs = millis();
      }
      break;
    case 'M':
      if (parseFloats(args, v, NUM_WHEELS) == NUM_WHEELS) {
        for (int i = 0; i < NUM_WHEELS; i++) testPwm[i] = (int)v[i];
        mode = MODE_PWM_TEST;
        lastCommandMs = millis();
      }
      break;
    case 'P':
      if (parseFloats(args, v, 4) == 4) {
        for (int i = 0; i < NUM_WHEELS; i++) {
          pid[i].kp = v[0]; pid[i].ki = v[1]; pid[i].kd = v[2]; pid[i].kf = v[3];
          pid[i].reset();
        }
        printConfig();
      }
      break;
    case 'R':
      noInterrupts();
      for (int i = 0; i < NUM_WHEELS; i++) encCount[i] = 0;
      interrupts();
      for (int i = 0; i < NUM_WHEELS; i++) lastCount[i] = 0;
      Serial.println("#encoders reset");
      break;
    case '?':
      printConfig();
      break;
    default:
      break;
  }
}

void readSerial() {
  static char buf[96];
  static size_t len = 0;
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        handleLine(buf);
        len = 0;
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;  // line too long: drop it
    }
  }
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(SERIAL_BAUD);

  if (MOTOR_STBY >= 0) {
    pinMode(MOTOR_STBY, OUTPUT);
    digitalWrite(MOTOR_STBY, HIGH);
  }
#if STATUS_LED >= 0
  pinMode(STATUS_LED, OUTPUT);
#endif

  for (int i = 0; i < NUM_WHEELS; i++) {
    pinMode(MOTOR_IN1[i], OUTPUT);
    pinMode(MOTOR_IN2[i], OUTPUT);
    setupPwm(i);
    setMotor(i, 0);

    // 34..39 have no pull-ups; INPUT_PULLUP is harmless there and helps on other pins
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
    encState[i] = (digitalRead(ENC_A[i]) << 1) | digitalRead(ENC_B[i]);
    attachInterruptArg(digitalPinToInterrupt(ENC_A[i]), encoderISR, (void *)(intptr_t)i, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(ENC_B[i]), encoderISR, (void *)(intptr_t)i, CHANGE);
  }

  Serial.println("#hb_kiwi_base ready");
  printConfig();
}

void loop() {
  static uint32_t lastControlUs = micros();
  static uint32_t lastTelemetryUs = micros();

  readSerial();

  const uint32_t now = micros();
  if (now - lastControlUs >= CONTROL_PERIOD_US) {
    const float dt = (now - lastControlUs) * 1e-6f;
    lastControlUs = now;
    controlStep(dt);
  }
  if (now - lastTelemetryUs >= TELEMETRY_PERIOD_US) {
    lastTelemetryUs = now;
    sendTelemetry();
  }
}
