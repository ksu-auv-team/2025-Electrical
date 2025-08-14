/**
 * @file imu.ino
 * @brief BNO086 tilt-corrected acceleration and velocity integration on RP2350, with serial output:
 *        {X_Vel,Y_Vel,Z_Vel,Roll,Pitch,Yaw}
 * @details
 *   - Initializes BNO08x over I2C on RP2350 (Pico 2).
 *   - Enables Rotation Vector and Linear Acceleration at ~100 Hz.
 *   - Converts linear acceleration (sensor frame) to world frame via quaternion.
 *   - High-pass filters acceleration via running average and integrates velocity with leakage.
 *   - Computes Euler angles (roll, pitch, yaw) from quaternion.
 *   - Prints velocities and angles scaled to 0–256 (127 ~ center) in the exact format required.
 *
 * @note Adjust I2C pins/address and scaling constants for your platform.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

// ===================== User Config =====================

#ifndef PIN_I2C_SDA
#define PIN_I2C_SDA 4      ///< I2C SDA pin (default Pico2 I2C0)
#endif
#ifndef PIN_I2C_SCL
#define PIN_I2C_SCL 5      ///< I2C SCL pin (default Pico2 I2C0)
#endif

static const uint8_t  BNO_ADDR = 0x4B;       ///< BNO08x I2C address (0x4A if ADDR low)
static const uint32_t I2C_HZ   = 400000;     ///< I2C clock speed (Hz)
static const float    SAMPLE_PERIOD_MS = 10.0f; ///< Target sample period (ms)

// If your mounting requires axis flips, set this to 1
#define FLIP_ROTATION_VECTOR_AXES 0

// Filter parameters (tune for your platform)
static const float AVG_RATE = 0.0001f;   ///< Low-pass for mean acc (to remove bias)
static const float LEAKAGE  = 0.004f;    ///< Leakage on velocity (helps bound drift)

// Scaling for serial output
// Map velocities from [-VEL_MAX, +VEL_MAX] m/s → [0, 256]
// Tune VEL_MAX to the expected maximum absolute speed on each axis.
static const float VEL_MAX = 2.0f;       ///< m/s, symmetric range for mapping

// ===================== Globals =====================

Adafruit_BNO08x bno;  ///< Global BNO08x instance

// Running averages and velocity (world frame)
float avgacc[3] = {0, 0, 0};
float vel[3]    = {0, 0, 0};

// Last processed time (ms) for integration
uint32_t last_process_ms = 0;

// Latest samples container
struct LatestSamples {
  float qw = 1, qx = 0, qy = 0, qz = 0;   ///< Quaternion (sensor→world)
  bool  haveQuat = false;

  float la_x = 0, la_y = 0, la_z = 0;     ///< Linear acceleration (sensor frame, m/s^2)
  bool  haveLA = false;
} latest;

// ===================== Helpers =====================

/**
 * @brief Normalize a quaternion in-place.
 */
static void normalizeQuat(float &w, float &x, float &y, float &z) {
  float n2 = w*w + x*x + y*y + z*z;
  if (n2 <= 0.0f) { w = 1; x = y = z = 0; return; }
  float inv = 1.0f / sqrtf(n2);
  w *= inv; x *= inv; y *= inv; z *= inv;
}

/**
 * @brief Rotate a vector from sensor/body frame to world frame using quaternion.
 */
static void rotateBodyToWorld(float w, float x, float y, float z,
                              float vx, float vy, float vz,
                              float out[3]) {
  // Ensure unit quaternion
  normalizeQuat(w, x, y, z);

  // Rotation matrix (world = R * body)
  float xx = x*x, yy = y*y, zz = z*z;
  float xy = x*y, xz = x*z, yz = y*z;
  float wx = w*x, wy = w*y, wz = w*z;

  float r00 = 1.0f - 2.0f*(yy + zz);
  float r01 = 2.0f*(xy + wz);
  float r02 = 2.0f*(xz - wy);

  float r10 = 2.0f*(xy - wz);
  float r11 = 1.0f - 2.0f*(xx + zz);
  float r12 = 2.0f*(yz + wx);

  float r20 = 2.0f*(xz + wy);
  float r21 = 2.0f*(yz - wx);
  float r22 = 1.0f - 2.0f*(xx + yy);

  out[0] = r00 * vx + r01 * vy + r02 * vz;
  out[1] = r10 * vx + r11 * vy + r12 * vz;
  out[2] = r20 * vx + r21 * vy + r22 * vz;
}

/**
 * @brief Convert quaternion (sensor→world) to Euler angles roll, pitch, yaw in degrees.
 * @details
 *  - roll  = rotation about X (world)
 *  - pitch = rotation about Y (world)
 *  - yaw   = rotation about Z (world)
 */
static void quatToEulerDeg(float w, float x, float y, float z,
                           float &roll_deg, float &pitch_deg, float &yaw_deg) {
  normalizeQuat(w, x, y, z);

  // From quaternion to Euler (Tait-Bryan ZYX: yaw-pitch-roll)
  float sinr_cosp = 2.0f * (w * x + y * z);
  float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
  float roll_rad  = atan2f(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (w * y - z * x);
  float pitch_rad;
  if (fabsf(sinp) >= 1.0f)
    pitch_rad = copysignf(M_PI / 2.0f, sinp); // use 90 deg if out of range
  else
    pitch_rad = asinf(sinp);

  float siny_cosp = 2.0f * (w * z + x * y);
  float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  float yaw_rad   = atan2f(siny_cosp, cosy_cosp);

  roll_deg  = roll_rad  * 180.0f / (float)M_PI;
  pitch_deg = pitch_rad * 180.0f / (float)M_PI;
  yaw_deg   = yaw_rad   * 180.0f / (float)M_PI;
}

/**
 * @brief Map a value in [inMin, inMax] to [0, 256], clamped.
 * @note Printed output is integer 0..256 inclusive. 127 is near center for symmetric ranges.
 */
static uint16_t mapTo256(float v, float inMin, float inMax) {
  if (inMax <= inMin) return 128; // avoid div by zero
  float t = (v - inMin) / (inMax - inMin);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  // scale to 0..256 (257 steps). Round to nearest.
  int val = (int)lroundf(t * 256.0f);
  if (val < 0) val = 0;
  if (val > 256) val = 256;
  return (uint16_t)val;
}

/**
 * @brief Enable the required BNO08x reports.
 */
static bool enableReports() {
  bool ok = true;
  ok &= bno.enableReport(SH2_ROTATION_VECTOR,     (uint32_t)(SAMPLE_PERIOD_MS * 1000.0f));
  ok &= bno.enableReport(SH2_LINEAR_ACCELERATION, (uint32_t)(SAMPLE_PERIOD_MS * 1000.0f));
  return ok;
}

/**
 * @brief Initialize I²C and the BNO08x device.
 */
static bool initBNO08x() {
  // I2C wiring & speed
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(I2C_HZ);

  // Start BNO08x
  if (!bno.begin_I2C(BNO_ADDR, &Wire)) {
    Serial.println(F("Failed to initialize BNO08x over I2C"));
    return false;
  }

  if (!enableReports()) {
    Serial.println(F("Could not enable required BNO08x reports"));
    return false;
  }

  return true;
}

// ===================== Arduino =====================

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for USB */ }

  if (!initBNO08x()) {
    Serial.println(F("Sensor configuration failed; halting."));
    for (;;)
      delay(1000);
  }

  last_process_ms = millis();
  Serial.println(F("BNO08x ready."));
}

void loop() {
  // If device soft-resets, re-enable our reports.
  if (bno.wasReset()) {
    Serial.println(F("BNO08x reset detected; re-enabling reports..."));
    if (!enableReports()) {
      Serial.println(F("Re-enable failed."));
    }
  }

  // Drain all available events this loop
  sh2_SensorValue_t ev;
  while (bno.getSensorEvent(&ev)) {
    switch (ev.sensorId) {
      case SH2_ROTATION_VECTOR: {
        float qw = ev.un.rotationVector.real;
        float qx = ev.un.rotationVector.i;
        float qy = ev.un.rotationVector.j;
        float qz = ev.un.rotationVector.k;

#if FLIP_ROTATION_VECTOR_AXES
        qx = -qx; qy = -qy; qz = -qz;
#endif
        latest.qw = qw; latest.qx = qx; latest.qy = qy; latest.qz = qz;
        latest.haveQuat = true;
        break;
      }
      case SH2_LINEAR_ACCELERATION: {
        latest.la_x = ev.un.linearAcceleration.x;
        latest.la_y = ev.un.linearAcceleration.y;
        latest.la_z = ev.un.linearAcceleration.z;
        latest.haveLA = true;
        break;
      }
      default:
        break;
    }
  }

  // Only compute when we have fresh data from both streams
  if (latest.haveQuat && latest.haveLA) {
    const uint32_t now_ms = millis();
    float dt = (now_ms - last_process_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 0.2f) {
      // Guard against timer wrap/pauses—skip integration if too large
      dt = 0.0f;
    }
    last_process_ms = now_ms;

    // Convert linear accel (sensor/body) to world frame
    float acc_world[3];
    rotateBodyToWorld(latest.qw, latest.qx, latest.qy, latest.qz,
                      latest.la_x, latest.la_y, latest.la_z,
                      acc_world);

    // Remove slow-varying bias and integrate with leakage
    for (int i = 0; i < 3; ++i) {
      avgacc[i] = AVG_RATE * acc_world[i] + (1.0f - AVG_RATE) * avgacc[i];
      float acc_hp = acc_world[i] - avgacc[i];
      vel[i] += dt * acc_hp;
      vel[i] -= LEAKAGE * vel[i];
    }

    // Euler angles (deg)
    float roll_deg, pitch_deg, yaw_deg;
    quatToEulerDeg(latest.qw, latest.qx, latest.qy, latest.qz,
                   roll_deg, pitch_deg, yaw_deg);

    // Map to 0..256 with ~127 center
    // Velocities: [-VEL_MAX, +VEL_MAX] → [0, 256]
    uint16_t vx_u = mapTo256(vel[0], -VEL_MAX, +VEL_MAX);
    uint16_t vy_u = mapTo256(vel[1], -VEL_MAX, +VEL_MAX);
    uint16_t vz_u = mapTo256(vel[2], -VEL_MAX, +VEL_MAX);

    // Angles: [-180, +180] → [0, 256]
    uint16_t roll_u  = mapTo256(roll_deg,  -180.0f, +180.0f);
    uint16_t pitch_u = mapTo256(pitch_deg, -180.0f, +180.0f);
    uint16_t yaw_u   = mapTo256(yaw_deg,   -180.0f, +180.0f);

    // Print in required format: {X_Vel,Y_Vel,Z_Vel,Roll,Pitch,Yaw}\n
    // Each value is an integer 0..256 (clamped)
    Serial.print('{');
    Serial.print(vx_u); Serial.print(',');
    Serial.print(vy_u); Serial.print(',');
    Serial.print(vz_u); Serial.print(',');
    Serial.print(roll_u);  Serial.print(',');
    Serial.print(pitch_u); Serial.print(',');
    Serial.print(yaw_u);
    Serial.println('}');

    // Clear flags so we require new data next loop
    latest.haveQuat = false;
    latest.haveLA   = false;
  }
}
