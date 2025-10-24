/**
 * @file imu_depth_list.ino
 * @brief Print IMU linear acceleration (world frame), Euler angles, and depth as a single list.
 *
 * @details
 *   Serial output (one line per update, no labels):
 *     [ax, ay, az, roll_deg, pitch_deg, yaw_deg, depth_m]
 *
 *   - BNO08x:
 *       * Reports: Rotation Vector (quaternion) + Linear Acceleration.
 *       * Linear acceleration (sensor frame) is rotated to world frame via the quaternion.
 *       * Euler angles are computed (roll, pitch, yaw in degrees) using ZYX convention.
 *   - MS5837:
 *       * Depth in meters; FLUID_DENSITY set to 997 (freshwater). Use 1029 for seawater.
 *
 * @note
 *   - Baud: 115200.
 *   - If your BNO08x I2C address is 0x4A, change BNO_ADDR below.
 *   - For boards that support custom I2C pins (e.g., RP2040), set PIN_I2C_SDA/PIN_I2C_SCL.
 */

#include <Arduino.h>
#include <Wire.h>

// ---------- MS5837 (Depth) ----------
#include "MS5837.h"

// ---------- BNO08x (IMU) ----------
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

// ===================== User Config =====================

#ifndef PIN_I2C_SDA
#define PIN_I2C_SDA -1     ///< Set to a valid pin on boards that support setSDA(); leave -1 to skip.
#endif
#ifndef PIN_I2C_SCL
#define PIN_I2C_SCL -1     ///< Set to a valid pin on boards that support setSCL(); leave -1 to skip.
#endif

static const uint8_t  BNO_ADDR        = 0x4B;      ///< 0x4B (ADDR high) or 0x4A (ADDR low)
static const uint32_t I2C_HZ          = 400000;    ///< I2C clock speed
static const uint32_t RV_SAMPLE_US    = 10000;     ///< ~100 Hz rotation vector
static const uint32_t LA_SAMPLE_US    = 10000;     ///< ~100 Hz linear acceleration
static const float    PRINT_PERIOD_MS = 50.0f;     ///< ~20 Hz print rate

static const float FLUID_DENSITY = 997.0f;         ///< kg/m^3 (freshwater). Use 1029.0 for seawater.

// If your physical mounting requires axis inversion for the rotation vector, set to 1.
#define FLIP_ROTATION_VECTOR_AXES 0

// ===================== Globals =====================

// Depth sensor
MS5837 depthSensor;

// IMU
Adafruit_BNO08x bno;

// Latest IMU data
struct LatestIMU {
  // Quaternion (sensor->world)
  float qw = 1, qx = 0, qy = 0, qz = 0;
  bool  haveQuat = false;

  // Linear acceleration (sensor frame), m/s^2
  float la_x = 0, la_y = 0, la_z = 0;
  bool  haveLA = false;

  // Derived
  float roll_deg = 0, pitch_deg = 0, yaw_deg = 0; // Euler ZYX
  float acc_world[3] = {0, 0, 0};                 // ax, ay, az (world frame)
} imu;

// Latest depth (m)
float depth_m = 0.0f;

// Timers
uint32_t lastPrintMs = 0;

// ===================== Helpers =====================

/**
 * @brief Normalize a quaternion in-place.
 * @param w Quaternion w
 * @param x Quaternion x
 * @param y Quaternion y
 * @param z Quaternion z
 */
static void normalizeQuat(float &w, float &x, float &y, float &z) {
  const float n2 = w*w + x*x + y*y + z*z;
  if (n2 <= 0.0f) { w = 1.0f; x = y = z = 0.0f; return; }
  const float inv = 1.0f / sqrtf(n2);
  w *= inv; x *= inv; y *= inv; z *= inv;
}

/**
 * @brief Rotate vector from body (sensor) frame to world frame using quaternion.
 * @param w,x,y,z Unit quaternion (sensor->world)
 * @param vx,vy,vz Vector in body/sensor frame
 * @param out3 Output world-frame vector
 */
static void rotateBodyToWorld(float w, float x, float y, float z,
                              float vx, float vy, float vz,
                              float out3[3]) {
  normalizeQuat(w, x, y, z);

  const float xx = x*x, yy = y*y, zz = z*z;
  const float xy = x*y, xz = x*z, yz = y*z;
  const float wx = w*x, wy = w*y, wz = w*z;

  const float r00 = 1.0f - 2.0f*(yy + zz);
  const float r01 = 2.0f*(xy + wz);
  const float r02 = 2.0f*(xz - wy);

  const float r10 = 2.0f*(xy - wz);
  const float r11 = 1.0f - 2.0f*(xx + zz);
  const float r12 = 2.0f*(yz + wx);

  const float r20 = 2.0f*(xz + wy);
  const float r21 = 2.0f*(yz - wx);
  const float r22 = 1.0f - 2.0f*(xx + yy);

  out3[0] = r00*vx + r01*vy + r02*vz;
  out3[1] = r10*vx + r11*vy + r12*vz;
  out3[2] = r20*vx + r21*vy + r22*vz;
}

/**
 * @brief Convert quaternion to Euler ZYX (roll, pitch, yaw) in degrees.
 * @param w,x,y,z Quaternion components
 * @param[out] roll_deg  Rotation about X (deg)
 * @param[out] pitch_deg Rotation about Y (deg)
 * @param[out] yaw_deg   Rotation about Z (deg)
 */
static void quatToEulerDeg(float w, float x, float y, float z,
                           float &roll_deg, float &pitch_deg, float &yaw_deg) {
  normalizeQuat(w, x, y, z);

  const float sinr_cosp = 2.0f * (w * x + y * z);
  const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
  const float roll_rad  = atan2f(sinr_cosp, cosr_cosp);

  const float sinp = 2.0f * (w * y - z * x);
  const float pitch_rad = (fabsf(sinp) >= 1.0f)
                        ? copysignf(1.57079632679f, sinp)   // pi/2
                        : asinf(sinp);

  const float siny_cosp = 2.0f * (w * z + x * y);
  const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  const float yaw_rad   = atan2f(siny_cosp, cosy_cosp);

  roll_deg  = roll_rad  * 57.2957795131f;   // 180/pi
  pitch_deg = pitch_rad * 57.2957795131f;
  yaw_deg   = yaw_rad   * 57.2957795131f;
}

/**
 * @brief Enable BNO08x reports.
 * @return true on success
 */
static bool enableBNOReports() {
  bool ok = true;
  ok &= bno.enableReport(SH2_ROTATION_VECTOR,     RV_SAMPLE_US);
  ok &= bno.enableReport(SH2_LINEAR_ACCELERATION, LA_SAMPLE_US);
  return ok;
}

/**
 * @brief Initialize I²C in a board-safe manner.
 */
static void safeWireBegin() {
#if (PIN_I2C_SDA >= 0) && (PIN_I2C_SCL >= 0)
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
#endif
  Wire.begin();
#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_SAM_DUE)
  Wire.setClock(I2C_HZ);
#endif
}

/**
 * @brief Initialize MS5837 (depth).
 * @return true on success
 */
static bool initDepth() {
  if (!depthSensor.init()) return false;
  depthSensor.setModel(MS5837::MS5837_30BA);
  depthSensor.setFluidDensity(FLUID_DENSITY);
  return true;
}

/**
 * @brief Initialize BNO08x (IMU).
 * @return true on success
 */
static bool initIMU() {
  if (!bno.begin_I2C(BNO_ADDR, &Wire)) return false;
  if (!enableBNOReports()) return false;
  return true;
}

// ===================== Arduino Lifecycle =====================

/**
 * @brief Arduino setup: initialize I²C, sensors, and Serial.
 */
void setup() {
  Serial.begin(115200);
  safeWireBegin();

  // Silent retries to keep console output clean (no extra text).
  while (!initDepth()) { delay(100); }
  while (!initIMU())   { delay(100); }

  lastPrintMs = millis();
}

/**
 * @brief Poll IMU events, update quaternion, acceleration, and derived outputs.
 */
static void serviceIMU() {
  if (bno.wasReset()) {
    (void)enableBNOReports(); // re-enable silently
  }

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
        imu.qw = qw; imu.qx = qx; imu.qy = qy; imu.qz = qz;
        imu.haveQuat = true;
        break;
      }
      case SH2_LINEAR_ACCELERATION: {
        imu.la_x = ev.un.linearAcceleration.x;
        imu.la_y = ev.un.linearAcceleration.y;
        imu.la_z = ev.un.linearAcceleration.z;
        imu.haveLA = true;
        break;
      }
      default:
        break;
    }
  }

  if (imu.haveQuat && imu.haveLA) {
    // World-frame acceleration from sensor-frame LA via quaternion.
    rotateBodyToWorld(imu.qw, imu.qx, imu.qy, imu.qz,
                      imu.la_x, imu.la_y, imu.la_z,
                      imu.acc_world);

    // Euler angles from quaternion.
    quatToEulerDeg(imu.qw, imu.qx, imu.qy, imu.qz,
                   imu.roll_deg, imu.pitch_deg, imu.yaw_deg);
  }
}

/**
 * @brief Read depth sensor.
 */
static void serviceDepth() {
  depthSensor.read();
  depth_m = depthSensor.depth();
}

/**
 * @brief Main loop: service sensors and print the single list.
 *
 * Output order (no labels):
 *   [ax, ay, az, roll_deg, pitch_deg, yaw_deg, depth_m]
 */
void loop() {
  serviceIMU();
  serviceDepth();

  const uint32_t now = millis();
  if ((now - lastPrintMs) >= (uint32_t)PRINT_PERIOD_MS) {
    lastPrintMs = now;

    if (imu.haveQuat && imu.haveLA) {
      Serial.print('[');
      Serial.print(imu.acc_world[0], 4); Serial.print(',');
      Serial.print(imu.acc_world[1], 4); Serial.print(',');
      Serial.print(imu.acc_world[2], 4); Serial.print(',');
      Serial.print(imu.roll_deg, 2);     Serial.print(',');
      Serial.print(imu.pitch_deg, 2);    Serial.print(',');
      Serial.print(imu.yaw_deg, 2);      Serial.print(',');
      Serial.print(depth_m, 3);
      Serial.println(']');

      // Require fresh samples next print interval
      imu.haveQuat = false;
      imu.haveLA   = false;
    }
  }
}

