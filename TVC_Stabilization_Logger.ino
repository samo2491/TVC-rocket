/*
  TVC_Stabilization_Logger.ino
  Complete Arduino Mega sketch for a two-axis TVC stabilization system with flight data logging.

  Hardware:
  - Arduino Mega 2560
  - MPU6050 (I2C)
  - BMP280 (I2C)
  - NEO-6M GPS (SoftwareSerial)
  - MicroSD module (SPI)
  - 2x SG90 servos (Pitch + Roll)
*/

// ==============================
// Includes & Library Imports
// ==============================
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>
#include <math.h>

// ==============================
// Debug Flag
// ==============================
#define DEBUG true

// ==============================
// Pin Definitions & Constants
// ==============================
const uint8_t SERVO_PITCH_PIN = 5;   // TVC pitch axis servo
const uint8_t SERVO_ROLL_PIN  = 6;   // TVC roll axis servo
const uint8_t SD_CS_PIN       = 10;  // SD card CS pin

// GPS via SoftwareSerial (RX, TX from Arduino perspective)
const uint8_t GPS_RX_PIN = 8;
const uint8_t GPS_TX_PIN = 9;

// MPU6050 I2C address and registers
const uint8_t MPU_ADDR          = 0x68;
const uint8_t MPU_REG_PWR_MGMT1 = 0x6B;
const uint8_t MPU_REG_ACCEL_X   = 0x3B;

// Servo mechanical limits
const float SERVO_NEUTRAL_DEG   = 90.0f;
const float SERVO_MAX_DEFLECT   = 30.0f; // +/- from neutral
const float SERVO_MIN_DEG       = SERVO_NEUTRAL_DEG - SERVO_MAX_DEFLECT;
const float SERVO_MAX_DEG       = SERVO_NEUTRAL_DEG + SERVO_MAX_DEFLECT;

// Control and logging rates
const uint32_t IMU_LOOP_US      = 10000UL; // 100 Hz stabilization loop
const uint32_t LOG_INTERVAL_MS  = 50UL;    // 20 Hz logging
const uint32_t DEBUG_INTERVAL_MS = 100UL;  // 10 Hz debug print

// Calibration
const uint16_t CALIB_SAMPLES    = 200;

// Complementary filter blend
const float COMPLEMENTARY_ALPHA = 0.98f;

// PID gains (TUNE THESE FOR YOUR VEHICLE)
const float KP_PITCH = 2.20f;
const float KI_PITCH = 0.25f;
const float KD_PITCH = 0.08f;

const float KP_ROLL  = 2.20f;
const float KI_ROLL  = 0.25f;
const float KD_ROLL  = 0.08f;

// PID anti-windup integrator limits
const float PID_I_LIMIT = 25.0f;

// Desired attitude targets (vertical flight)
const float TARGET_PITCH_DEG = 0.0f;
const float TARGET_ROLL_DEG  = 0.0f;

// BMP280 sea-level pressure estimate (Pa). Can be adjusted for location/weather.
const float SEA_LEVEL_PRESSURE_PA = 101325.0f;

// ==============================
// Global Objects
// ==============================
Servo servoPitch;
Servo servoRoll;

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
Adafruit_BMP280 bmp;

File logFile;
char logFileName[13] = "FLIGHT00.CSV"; // 8.3 filename

// ==============================
// Sensor, Filter, and PID Variables
// ==============================
// Raw IMU values
int16_t rawAx, rawAy, rawAz;
int16_t rawGx, rawGy, rawGz;

// Calibrated IMU values (offset removed)
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

// IMU offsets
float accXOffset = 0.0f, accYOffset = 0.0f, accZOffset = 0.0f;
float gyroXOffset = 0.0f, gyroYOffset = 0.0f, gyroZOffset = 0.0f;

// Orientation estimate
float pitchDeg = 0.0f;
float rollDeg  = 0.0f;
float yawDeg   = 0.0f; // integrated gyro yaw (drifts without magnetometer)

// PID states
float pitchIntegral = 0.0f;
float rollIntegral  = 0.0f;
float prevPitchError = 0.0f;
float prevRollError  = 0.0f;

// Servo command states
float servoPitchCmd = SERVO_NEUTRAL_DEG;
float servoRollCmd  = SERVO_NEUTRAL_DEG;

// Timing trackers
uint32_t lastImuMicros  = 0;
uint32_t lastLogMillis  = 0;
uint32_t lastDebugMillis = 0;
uint32_t lastFlushMillis = 0;

// Barometer baseline altitude (ground-relative)
float baroBaselineAltM = 0.0f;

// ==============================
// Utility Functions
// ==============================
float constrainFloat(float value, float minV, float maxV) {
  if (value < minV) return minV;
  if (value > maxV) return maxV;
  return value;
}

void writeMPURegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readMPURaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACCEL_X);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  if (Wire.available() >= 14) {
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // skip temperature bytes
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
  }
}

void initMPU6050() {
  // Wake MPU6050 from sleep
  writeMPURegister(MPU_REG_PWR_MGMT1, 0x00);

  // Optional: default full-scale ranges are used
  // Accel: +/-2g, Gyro: +/-250 dps (highest sensitivity)
}

void calibrateMPU6050() {
  long sumAx = 0, sumAy = 0, sumAz = 0;
  long sumGx = 0, sumGy = 0, sumGz = 0;

  for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readMPURaw(ax, ay, az, gx, gy, gz);

    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    sumGx += gx;
    sumGy += gy;
    sumGz += gz;

    delay(5);
  }

  // Offsets in raw units
  accXOffset  = (float)sumAx / CALIB_SAMPLES;
  accYOffset  = (float)sumAy / CALIB_SAMPLES;
  // Keep Z such that stationary level reads +1g (16384 LSB in +/-2g mode)
  accZOffset  = ((float)sumAz / CALIB_SAMPLES) - 16384.0f;

  gyroXOffset = (float)sumGx / CALIB_SAMPLES;
  gyroYOffset = (float)sumGy / CALIB_SAMPLES;
  gyroZOffset = (float)sumGz / CALIB_SAMPLES;
}

void createUniqueLogFile() {
  for (uint8_t i = 1; i <= 99; i++) {
    snprintf(logFileName, sizeof(logFileName), "FLIGHT%02u.CSV", i);
    if (!SD.exists(logFileName)) {
      logFile = SD.open(logFileName, FILE_WRITE);
      if (logFile) {
        logFile.println(F("Timestamp(ms),Pitch(deg),Roll(deg),Yaw(deg),AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Servo1(deg),Servo2(deg),GPS_Lat,GPS_Lon,GPS_Alt(m),GPS_Speed(km/h),Baro_Alt(m),Temperature(C)"));
        logFile.flush();
      }
      return;
    }
  }
}

// ==============================
// Setup
// ==============================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  gpsSerial.begin(9600);

  // Attach servos and move to neutral
  servoPitch.attach(SERVO_PITCH_PIN);
  servoRoll.attach(SERVO_ROLL_PIN);
  servoPitch.write((int)SERVO_NEUTRAL_DEG);
  servoRoll.write((int)SERVO_NEUTRAL_DEG);

  // Initialize MPU6050
  initMPU6050();
  delay(100);

  // Calibrate MPU offsets while rocket is stationary
  calibrateMPU6050();

  // Initialize BMP280
  if (!bmp.begin(0x76)) {
    // fallback to alternative BMP280 address
    if (!bmp.begin(0x77)) {
#if DEBUG
      Serial.println(F("BMP280 init failed."));
#endif
    }
  }

  // Capture baseline (ground) altitude
  baroBaselineAltM = bmp.readAltitude(SEA_LEVEL_PRESSURE_PA);

  // Initialize SD card
  if (SD.begin(SD_CS_PIN)) {
    createUniqueLogFile();
#if DEBUG
    Serial.print(F("Logging to: "));
    Serial.println(logFileName);
#endif
  } else {
#if DEBUG
    Serial.println(F("SD init failed. Logging disabled."));
#endif
  }

  lastImuMicros   = micros();
  lastLogMillis   = millis();
  lastDebugMillis = millis();
  lastFlushMillis = millis();
}

// ==============================
// Main Loop
// ==============================
void loop() {
  // --------------------------------
  // Always parse incoming GPS bytes
  // --------------------------------
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // --------------------------------
  // Stabilization loop @ 100 Hz
  // --------------------------------
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastImuMicros) >= IMU_LOOP_US) {
    float dt = (nowUs - lastImuMicros) / 1000000.0f;
    lastImuMicros = nowUs;

    // -------- IMU read --------
    readMPURaw(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

    // Remove offsets (raw units)
    accX = (float)rawAx - accXOffset;
    accY = (float)rawAy - accYOffset;
    accZ = (float)rawAz - accZOffset;

    gyroX = (float)rawGx - gyroXOffset;
    gyroY = (float)rawGy - gyroYOffset;
    gyroZ = (float)rawGz - gyroZOffset;

    // Convert gyro raw to deg/s (131 LSB per deg/s for +/-250 dps)
    float gyroX_dps = gyroX / 131.0f;
    float gyroY_dps = gyroY / 131.0f;
    float gyroZ_dps = gyroZ / 131.0f;

    // Accel angle estimates
    float accPitchDeg = atan2f(accY, sqrtf(accX * accX + accZ * accZ)) * 180.0f / PI;
    float accRollDeg  = atan2f(-accX, accZ) * 180.0f / PI;

    // Complementary filter fusion
    pitchDeg = COMPLEMENTARY_ALPHA * (pitchDeg + gyroX_dps * dt) + (1.0f - COMPLEMENTARY_ALPHA) * accPitchDeg;
    rollDeg  = COMPLEMENTARY_ALPHA * (rollDeg  + gyroY_dps * dt) + (1.0f - COMPLEMENTARY_ALPHA) * accRollDeg;

    // Yaw from gyro integration only (no mag correction)
    yawDeg += gyroZ_dps * dt;

    // -------- PID compute --------
    float pitchError = TARGET_PITCH_DEG - pitchDeg;
    float rollError  = TARGET_ROLL_DEG  - rollDeg;

    pitchIntegral += pitchError * dt;
    rollIntegral  += rollError * dt;

    // Clamp integrator for anti-windup
    pitchIntegral = constrainFloat(pitchIntegral, -PID_I_LIMIT, PID_I_LIMIT);
    rollIntegral  = constrainFloat(rollIntegral,  -PID_I_LIMIT, PID_I_LIMIT);

    float pitchDerivative = (pitchError - prevPitchError) / dt;
    float rollDerivative  = (rollError  - prevRollError) / dt;

    prevPitchError = pitchError;
    prevRollError  = rollError;

    float pitchOutput = (KP_PITCH * pitchError) + (KI_PITCH * pitchIntegral) + (KD_PITCH * pitchDerivative);
    float rollOutput  = (KP_ROLL  * rollError)  + (KI_ROLL  * rollIntegral)  + (KD_ROLL  * rollDerivative);

    // -------- Servo write --------
    // Output directly offsets neutral servo angle, then clamp to safe travel range.
    servoPitchCmd = constrainFloat(SERVO_NEUTRAL_DEG + pitchOutput, SERVO_MIN_DEG, SERVO_MAX_DEG);
    servoRollCmd  = constrainFloat(SERVO_NEUTRAL_DEG + rollOutput,  SERVO_MIN_DEG, SERVO_MAX_DEG);

    servoPitch.write((int)servoPitchCmd);
    servoRoll.write((int)servoRollCmd);
  }

  // --------------------------------
  // Logging loop @ 20 Hz
  // --------------------------------
  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastLogMillis) >= LOG_INTERVAL_MS) {
    lastLogMillis = nowMs;

    // -------- Barometer read --------
    float tempC = bmp.readTemperature();
    float baroAltM = bmp.readAltitude(SEA_LEVEL_PRESSURE_PA) - baroBaselineAltM;

    // -------- GPS read --------
    float gpsLat   = gps.location.isValid() ? (float)gps.location.lat() : 0.0f;
    float gpsLon   = gps.location.isValid() ? (float)gps.location.lng() : 0.0f;
    float gpsAltM  = gps.altitude.isValid() ? (float)gps.altitude.meters() : 0.0f;
    float gpsSpdKmh = gps.speed.isValid() ? (float)gps.speed.kmph() : 0.0f;

    // -------- SD log --------
    if (logFile) {
      // Single-row CSV write; flush occasionally to reduce blocking latency.
      logFile.print(nowMs);
      logFile.print(','); logFile.print(pitchDeg, 3);
      logFile.print(','); logFile.print(rollDeg, 3);
      logFile.print(','); logFile.print(yawDeg, 3);
      logFile.print(','); logFile.print(accX, 2);
      logFile.print(','); logFile.print(accY, 2);
      logFile.print(','); logFile.print(accZ, 2);
      logFile.print(','); logFile.print(gyroX, 2);
      logFile.print(','); logFile.print(gyroY, 2);
      logFile.print(','); logFile.print(gyroZ, 2);
      logFile.print(','); logFile.print(servoPitchCmd, 1);
      logFile.print(','); logFile.print(servoRollCmd, 1);
      logFile.print(','); logFile.print(gpsLat, 6);
      logFile.print(','); logFile.print(gpsLon, 6);
      logFile.print(','); logFile.print(gpsAltM, 2);
      logFile.print(','); logFile.print(gpsSpdKmh, 2);
      logFile.print(','); logFile.print(baroAltM, 2);
      logFile.print(','); logFile.println(tempC, 2);

      // Flush every 1 second to reduce data loss risk while limiting write stalls.
      if ((uint32_t)(nowMs - lastFlushMillis) >= 1000UL) {
        lastFlushMillis = nowMs;
        logFile.flush();
      }
    }

#if DEBUG
    // -------- Debug output --------
    if ((uint32_t)(nowMs - lastDebugMillis) >= DEBUG_INTERVAL_MS) {
      lastDebugMillis = nowMs;
      Serial.print(F("Pitch: "));
      Serial.print(pitchDeg, 2);
      Serial.print(F("  Roll: "));
      Serial.print(rollDeg, 2);
      Serial.print(F("  S1: "));
      Serial.print(servoPitchCmd, 1);
      Serial.print(F("  S2: "));
      Serial.print(servoRollCmd, 1);
      Serial.print(F("  Alt(m): "));
      Serial.println(baroAltM, 2);
    }
#endif
  }
}
