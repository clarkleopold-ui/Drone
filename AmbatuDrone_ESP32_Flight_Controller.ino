#include <Arduino.h>
#include "BluetoothSerial.h"
#include <cstdio>
#include <Wire.h>

// AmbatuDrone direct ESP32 flight-controller baseline
//
// Computer / PS4 app -> Bluetooth Classic SPP -> this ESP32 -> four ESCs
//                                             -> MPU-6050 over I2C
//
// This sketch replaces BOTH:
//   bridgeMotorControl_Bluetooth.ino
//   AmbatuDrone_ESP32_Bluetooth_Bridge.ino
//
// Target:
//   Original ESP32-WROOM-32D development board
//   Arduino-ESP32 board platform 3.x
//
// Servos remain intentionally dormant.

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled for the selected ESP32 board.
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Bluetooth Classic SPP is unavailable. Select an original ESP32 board, not an ESP32-C3/S3.
#endif

BluetoothSerial SerialBT;

// ---------- Connection ----------
constexpr char DEVICE_NAME[] = "AmbatuDrone";
constexpr uint32_t USB_BAUD = 115200;
constexpr size_t MAX_LINE_LENGTH = 180;

struct SerialLineState {
  char buffer[MAX_LINE_LENGTH];
  size_t index;
  bool overflow;
};

SerialLineState usbRx = {};
SerialLineState bluetoothRx = {};

// ---------- Pin assignment ----------
// These are signal pins only. ESC power does not pass through the ESP32.
constexpr uint8_t MOTOR_PINS[4] = {25, 26, 27, 32};
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t MPU_ADDRESS = 0x68;

// ---------- ESC output ----------
constexpr uint32_t ESC_FREQUENCY_HZ = 50;
constexpr uint8_t ESC_PWM_RESOLUTION_BITS = 16;
constexpr uint32_t ESC_PWM_MAX_DUTY =
  (1UL << ESC_PWM_RESOLUTION_BITS) - 1UL;
constexpr uint32_t ESC_PERIOD_US = 1000000UL / ESC_FREQUENCY_HZ;

constexpr int THROTTLE_MIN = 1000;
constexpr int THROTTLE_MAX = 2000;
constexpr int ARM_THROTTLE_MAX_PERCENT = 5;
constexpr int CONTROL_CORRECTION_PERCENT = 15;

bool motorOutputReady[4] = {false, false, false, false};
bool motorOutputsHealthy = false;

// ---------- Timing and safety ----------
constexpr uint32_t FAILSAFE_MS = 500;
constexpr uint32_t CONTROL_PERIOD_US = 4000;  // 250 Hz
constexpr uint32_t TELEMETRY_PERIOD_MS = 40;  // 25 Hz

uint32_t lastPacketMs = 0;
uint32_t lastControlUs = 0;
uint32_t lastTelemetryMs = 0;

bool haveValidPacket = false;
bool failsafeActive = true;
bool motorArmed = false;
bool throttleNotLow = false;
bool imuConfigured = false;
bool imuHealthy = false;
bool bluetoothHealthy = false;

// ---------- Live controller and flight data ----------
struct FlightData {
  int lx;
  int ly;
  int rx;
  int ry;
  int cross;
  int circle;
  int square;
  int triangle;
  int l1;
  int r1;

  int motorPwm[4];
  int throttlePercent;
  int throttle;
  int pitch;
  int roll;
  int yaw;
  int pitchOffset;
  int rollOffset;
  int yawOffset;
  int correctionRange;

  int16_t accelerometerX;
  int16_t accelerometerY;
  int16_t accelerometerZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;

  float accX;
  float accY;
  float accZ;
  float rateRoll;
  float ratePitch;
  float rateYaw;
  float angleRoll;
  float anglePitch;
  float angleYaw;
  float yawRate;
  float rateCalibrationRoll;
  float rateCalibrationPitch;
  float rateCalibrationYaw;
  float kalmanAngleRoll;
  float kalmanAnglePitch;
  float kalmanUncertaintyRoll;
  float kalmanUncertaintyPitch;
};

FlightData data = {};

// ---------- ESC helpers ----------
uint32_t microsecondsToDuty(int pulseUs) {
  const uint32_t limited = constrain(
    pulseUs,
    THROTTLE_MIN,
    THROTTLE_MAX
  );

  return (uint32_t)(
    ((uint64_t)limited * ESC_PWM_MAX_DUTY + ESC_PERIOD_US / 2U)
    / ESC_PERIOD_US
  );
}

void writeMotorMicroseconds(size_t motorIndex, int pulseUs) {
  if (motorIndex >= 4 || !motorOutputReady[motorIndex]) {
    return;
  }

  ledcWrite(
    MOTOR_PINS[motorIndex],
    microsecondsToDuty(pulseUs)
  );
}

void stopAllMotors() {
  data.throttle = THROTTLE_MIN;
  data.throttlePercent = 0;
  data.correctionRange = 0;
  data.pitchOffset = 0;
  data.rollOffset = 0;
  data.yawOffset = 0;

  for (size_t i = 0; i < 4; i++) {
    data.motorPwm[i] = THROTTLE_MIN;
    writeMotorMicroseconds(i, THROTTLE_MIN);
  }
}

bool initializeMotorOutputs() {
  bool allReady = true;

  for (size_t i = 0; i < 4; i++) {
    motorOutputReady[i] = ledcAttach(
      MOTOR_PINS[i],
      ESC_FREQUENCY_HZ,
      ESC_PWM_RESOLUTION_BITS
    );
    allReady = allReady && motorOutputReady[i];

    if (motorOutputReady[i]) {
      writeMotorMicroseconds(i, THROTTLE_MIN);
    }
  }

  return allReady;
}

// ---------- Command parser ----------
bool readIntField(const char* source, const char* key, int& output) {
  char token[32];
  snprintf(token, sizeof(token), "\"%s\":", key);

  const char* start = strstr(source, token);
  if (start == nullptr) {
    return false;
  }

  start += strlen(token);
  while (*start == ' ' || *start == '\t') {
    start++;
  }

  char* end = nullptr;
  const long value = strtol(start, &end, 10);
  if (end == start) {
    return false;
  }

  while (*end == ' ' || *end == '\t') {
    end++;
  }

  if (*end != ',' && *end != '}') {
    return false;
  }

  output = (int)value;
  return true;
}

bool parseControllerLine(const char* line) {
  const size_t length = strlen(line);
  if (length < 2 || line[0] != '{' || line[length - 1] != '}') {
    return false;
  }

  int lx;
  int ly;
  int rx;
  int ry;
  int cross;
  int circle;
  int square;
  int triangle;
  int l1;
  int r1;

  if (
    !readIntField(line, "lx", lx) ||
    !readIntField(line, "ly", ly) ||
    !readIntField(line, "rx", rx) ||
    !readIntField(line, "ry", ry) ||
    !readIntField(line, "cross", cross) ||
    !readIntField(line, "circle", circle) ||
    !readIntField(line, "square", square) ||
    !readIntField(line, "triangle", triangle) ||
    !readIntField(line, "l1", l1) ||
    !readIntField(line, "r1", r1)
  ) {
    return false;
  }

  // Commit only after the complete packet is valid. A malformed packet never
  // refreshes the communication failsafe.
  data.lx = constrain(lx, -100, 100);
  data.ly = constrain(ly, -100, 100);
  data.rx = constrain(rx, -100, 100);
  data.ry = constrain(ry, -100, 100);
  data.cross = constrain(cross, 0, 1);
  data.circle = constrain(circle, 0, 1);
  data.square = constrain(square, 0, 1);
  data.triangle = constrain(triangle, 0, 1);
  data.l1 = constrain(l1, 0, 1);
  data.r1 = constrain(r1, 0, 1);

  haveValidPacket = true;
  lastPacketMs = millis();
  return true;
}

void processSerialInput(Stream& input, SerialLineState& state) {
  while (input.available() > 0) {
    const int incoming = input.read();
    if (incoming < 0) {
      break;
    }

    const char character = (char)incoming;
    if (character == '\n') {
      if (!state.overflow && state.index > 0) {
        state.buffer[state.index] = '\0';
        parseControllerLine(state.buffer);
      }
      state.index = 0;
      state.overflow = false;
    } else if (character != '\r' && !state.overflow) {
      if (state.index < MAX_LINE_LENGTH - 1) {
        state.buffer[state.index++] = character;
      } else {
        state.index = 0;
        state.overflow = true;
      }
    }
  }
}

// ---------- MPU-6050 ----------
bool writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool initializeImu() {
  return
    writeMpuRegister(0x6B, 0x00) &&
    writeMpuRegister(0x1A, 0x05) &&
    writeMpuRegister(0x1C, 0x10) &&
    writeMpuRegister(0x1B, 0x08);
}

bool readImuSample(float deltaSeconds, bool applyCalibration) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write((uint8_t)0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t received =
    Wire.requestFrom(MPU_ADDRESS, (uint8_t)14, (uint8_t)true);
  if (received != 14 || Wire.available() < 14) {
    return false;
  }

  data.accelerometerX = ((int16_t)Wire.read() << 8) | Wire.read();
  data.accelerometerY = ((int16_t)Wire.read() << 8) | Wire.read();
  data.accelerometerZ = ((int16_t)Wire.read() << 8) | Wire.read();

  // MPU temperature is not used by the flight loop.
  Wire.read();
  Wire.read();

  data.gyroX = ((int16_t)Wire.read() << 8) | Wire.read();
  data.gyroY = ((int16_t)Wire.read() << 8) | Wire.read();
  data.gyroZ = ((int16_t)Wire.read() << 8) | Wire.read();

  data.accX = (float)data.accelerometerX / 4096.0f;
  data.accY = (float)data.accelerometerY / 4096.0f;
  data.accZ = (float)data.accelerometerZ / 4096.0f;

  data.rateRoll = (float)data.gyroX / 65.5f;
  data.ratePitch = (float)data.gyroY / 65.5f;
  data.rateYaw = (float)data.gyroZ / 65.5f;

  if (applyCalibration) {
    data.rateRoll -= data.rateCalibrationRoll;
    data.ratePitch -= data.rateCalibrationPitch;
    data.rateYaw -= data.rateCalibrationYaw;
  }

  data.angleRoll = atan2(
    data.accY,
    sqrt(data.accX * data.accX + data.accZ * data.accZ)
  ) * RAD_TO_DEG;

  data.anglePitch = -atan2(
    data.accX,
    sqrt(data.accY * data.accY + data.accZ * data.accZ)
  ) * RAD_TO_DEG;

  if (applyCalibration) {
    data.angleYaw += data.rateYaw * deltaSeconds;
    data.yawRate = data.rateYaw;
  }

  return true;
}

bool calibrateImu() {
  constexpr int CALIBRATION_SAMPLES = 2000;
  int validSamples = 0;
  float rollSum = 0.0f;
  float pitchSum = 0.0f;
  float yawSum = 0.0f;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    if (readImuSample(0.0f, false)) {
      rollSum += data.rateRoll;
      pitchSum += data.ratePitch;
      yawSum += data.rateYaw;
      validSamples++;
    }
    delay(1);
  }

  if (validSamples < CALIBRATION_SAMPLES * 9 / 10) {
    return false;
  }

  data.rateCalibrationRoll = rollSum / validSamples;
  data.rateCalibrationPitch = pitchSum / validSamples;
  data.rateCalibrationYaw = yawSum / validSamples;
  data.rateRoll = 0.0f;
  data.ratePitch = 0.0f;
  data.rateYaw = 0.0f;
  data.yawRate = 0.0f;
  data.angleYaw = 0.0f;
  return true;
}

void kalmanUpdate(
  float& state,
  float& uncertainty,
  float rate,
  float measurement,
  float deltaSeconds
) {
  state += deltaSeconds * rate;
  uncertainty += deltaSeconds * deltaSeconds * 4.0f * 4.0f;

  const float gain = uncertainty / (uncertainty + 3.0f * 3.0f);
  state += gain * (measurement - state);
  uncertainty *= (1.0f - gain);
  state = constrain(state, -60.0f, 60.0f);
}

// ---------- Motor mixer ----------
void updateMotors() {
  throttleNotLow = false;

  if (data.cross || !data.r1) {
    motorArmed = false;
    stopAllMotors();
    return;
  }

  if (!imuHealthy || !motorOutputsHealthy) {
    motorArmed = false;
    stopAllMotors();
    return;
  }

  data.throttlePercent = constrain(data.ly, 0, 100);

  // Every transition from disarmed to armed must happen at 5% or lower.
  if (!motorArmed) {
    if (data.throttlePercent > ARM_THROTTLE_MAX_PERCENT) {
      throttleNotLow = true;
      stopAllMotors();
      return;
    }
    motorArmed = true;
  }

  data.pitch = constrain(data.ry, -100, 100);
  data.roll = constrain(data.rx, -100, 100);
  data.yaw = constrain(data.lx, -100, 100);

  data.throttle = map(
    data.throttlePercent,
    0,
    100,
    THROTTLE_MIN,
    THROTTLE_MAX
  );

  data.correctionRange =
    ((long)(data.throttle - THROTTLE_MIN)
      * CONTROL_CORRECTION_PERCENT) / 100L;

  data.pitchOffset = map(
    data.pitch,
    -100,
    100,
    -data.correctionRange,
    data.correctionRange
  );
  data.rollOffset = map(
    data.roll,
    -100,
    100,
    -data.correctionRange,
    data.correctionRange
  );
  data.yawOffset = map(
    data.yaw,
    -100,
    100,
    -data.correctionRange,
    data.correctionRange
  );

  const long limitedPitchAngle =
    (long)constrain(data.kalmanAnglePitch, -80.0f, 80.0f);
  const long limitedRollAngle =
    (long)constrain(data.kalmanAngleRoll, -80.0f, 80.0f);

  const int pitchGyro = map(
    limitedPitchAngle,
    -80,
    80,
    -data.correctionRange,
    data.correctionRange
  );
  const int rollGyro = map(
    limitedRollAngle,
    -80,
    80,
    -data.correctionRange,
    data.correctionRange
  );

  // Motor order and signs match the corrected Mega baseline.
  data.motorPwm[0] =
    data.throttle + pitchGyro + rollGyro
    - data.pitchOffset + data.rollOffset + data.yawOffset;

  data.motorPwm[1] =
    data.throttle - pitchGyro + rollGyro
    + data.pitchOffset + data.rollOffset - data.yawOffset;

  data.motorPwm[2] =
    data.throttle + pitchGyro - rollGyro
    - data.pitchOffset - data.rollOffset - data.yawOffset;

  data.motorPwm[3] =
    data.throttle - pitchGyro - rollGyro
    + data.pitchOffset - data.rollOffset + data.yawOffset;

  // When there is no manual yaw command, oppose the measured yaw rate.
  if (data.lx == 0) {
    const long limitedYawRate =
      (long)constrain(data.yawRate, -125.0f, 125.0f);
    const int yawGyro = map(
      limitedYawRate,
      -125,
      125,
      -data.correctionRange,
      data.correctionRange
    );

    data.motorPwm[0] += yawGyro;
    data.motorPwm[1] -= yawGyro;
    data.motorPwm[2] -= yawGyro;
    data.motorPwm[3] += yawGyro;
  }

  for (size_t i = 0; i < 4; i++) {
    data.motorPwm[i] = constrain(
      data.motorPwm[i],
      THROTTLE_MIN,
      THROTTLE_MAX
    );
    writeMotorMicroseconds(i, data.motorPwm[i]);
  }
}

// ---------- Telemetry ----------
void sendTelemetry(Print& output) {
  output.print("ACK motor_pwm=");
  output.print(data.motorPwm[0]);
  output.print(" ");
  output.print(data.motorPwm[1]);
  output.print(" ");
  output.print(data.motorPwm[2]);
  output.print(" ");
  output.print(data.motorPwm[3]);
  output.print(" dir=+ - - +");
  output.print(" servo1=0 servo2=0 servo3=0 servo4=0");
  output.print(" lx=");
  output.print(data.lx);
  output.print(" ly=");
  output.print(data.ly);
  output.print(" rx=");
  output.print(data.rx);
  output.print(" ry=");
  output.print(data.ry);
  output.print(" Roll [deg]: ");
  output.print(data.kalmanAngleRoll, 2);
  output.print(" Pitch [deg]: ");
  output.print(data.kalmanAnglePitch, 2);
  output.print(" Yaw [deg]: ");
  output.print(data.angleYaw, 2);
  output.print(" Yaw Rate [deg/s]: ");
  output.print(data.yawRate, 2);

  if (data.cross) {
    output.print(" KILL");
  }
  if (!data.r1 || !motorArmed) {
    output.print(" MOTOR_LOCKED");
  }
  if (throttleNotLow) {
    output.print(" THROTTLE_NOT_LOW");
  }
  if (failsafeActive) {
    output.print(" FAILSAFE");
  }
  if (!imuHealthy) {
    output.print(" IMU_ERROR");
  }
  if (!motorOutputsHealthy) {
    output.print(" PWM_ERROR");
  }
  if (!bluetoothHealthy) {
    output.print(" BT_ERROR");
  }
  output.print(" SERVOS_DORMANT");
  output.println();
}

void initializeFlightData() {
  data.kalmanUncertaintyRoll = 4.0f;
  data.kalmanUncertaintyPitch = 4.0f;
  stopAllMotors();
}

// ---------- Setup ----------
void setup() {
  Serial.begin(USB_BAUD);
  delay(100);

  initializeFlightData();
  motorOutputsHealthy = initializeMotorOutputs();
  stopAllMotors();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  Wire.setTimeOut(10);

  Serial.println("Keep the MPU-6050 still: calibrating...");
  imuConfigured = initializeImu();
  imuHealthy = imuConfigured && calibrateImu();

  // Keep minimum ESC pulses active during the normal ESC startup interval.
  stopAllMotors();
  delay(3000);

  bluetoothHealthy = SerialBT.begin(DEVICE_NAME);

  haveValidPacket = false;
  failsafeActive = true;
  motorArmed = false;
  throttleNotLow = false;
  lastControlUs = micros();
  lastTelemetryMs = millis();

  Serial.println("ESP32 flight controller ready");
  if (bluetoothHealthy) {
    SerialBT.println("ESP32 flight controller ready");
  }
}

// ---------- Main loop ----------
void loop() {
  // USB remains available for a wired bench test. Bluetooth is the normal
  // wireless path. Both use the same newline-terminated JSON packets.
  processSerialInput(Serial, usbRx);
  if (bluetoothHealthy) {
    processSerialInput(SerialBT, bluetoothRx);
  }

  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastControlUs;
  if (elapsedUs < CONTROL_PERIOD_US) {
    return;
  }

  const float deltaSeconds = constrain(
    elapsedUs * 0.000001f,
    0.001f,
    0.020f
  );
  lastControlUs = nowUs;

  if (imuConfigured) {
    imuHealthy = readImuSample(deltaSeconds, true);
  } else {
    imuHealthy = false;
  }

  if (imuHealthy) {
    kalmanUpdate(
      data.kalmanAngleRoll,
      data.kalmanUncertaintyRoll,
      data.rateRoll,
      data.angleRoll,
      deltaSeconds
    );
    kalmanUpdate(
      data.kalmanAnglePitch,
      data.kalmanUncertaintyPitch,
      data.ratePitch,
      data.anglePitch,
      deltaSeconds
    );
  }

  failsafeActive =
    !haveValidPacket || (millis() - lastPacketMs > FAILSAFE_MS);

  if (failsafeActive) {
    data.r1 = 0;
    motorArmed = false;
    throttleNotLow = false;
    stopAllMotors();
  } else {
    // Triangle and servo states remain intentionally disabled.
    updateMotors();
  }

  if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = millis();
    sendTelemetry(Serial);
    if (bluetoothHealthy) {
      sendTelemetry(SerialBT);
    }
  }
}
