#include <Arduino.h>
#include <ADC.h>
#include <IntervalTimer.h>
#include <Servo.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <cstdio>
#include <cstring>

/*
  AmbatuDrone Teensy 4.1 combined flight controller and motor-health logger
  Version: 2026-08-09

  BENCH-TEST BUILD. REMOVE ALL PROPELLERS.

  Command paths:
    Computer app -> Teensy USB Serial
    Computer app -> ground Feather -> RFM95 -> airborne Feather -> Teensy Serial1

  Flight hardware:
    ESC 1..4: pins 3, 4, 5, 6
    MPU-6050: SDA 18, SCL 19

  Logger hardware:
    MAX4466 microphone: A2 / pin 16
    ACS72981-050B3 current sensor: A3 / pin 17
    Record button: pin 2 to GND
    Storage: Teensy 4.1 built-in microSD slot

  Safety behavior:
    - All ESCs start at 1000 us.
    - R1 must be held and throttle must be <= 5% to arm.
    - X/Cross, R1 release, a 500 ms command timeout, IMU failure, logger
      failure, or a control-loop overrun disarms all motors.
    - Recording starts before arming and stops after disarming.
    - A logger buffer overflow drops samples; it never blocks the ADC ISR.

  Required Teensy/Arduino libraries:
    ADC, Servo, SD/SdFat, IntervalTimer, Wire (included with Teensyduino)
*/

// -------------------- User configuration --------------------
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t RADIO_UART_BAUD = 115200;

constexpr uint8_t MOTOR_PINS[4] = {3, 4, 5, 6};
constexpr char MOTOR_DIRECTIONS[4] = {'+', '-', '-', '+'};
constexpr uint8_t RECORD_BUTTON_PIN = 2;
constexpr uint8_t RECORD_LED_PIN = LED_BUILTIN;
constexpr uint8_t MICROPHONE_PIN = A2;
constexpr uint8_t CURRENT_SENSOR_PIN = A3;
constexpr uint8_t MPU_ADDRESS = 0x68;

constexpr int THROTTLE_MIN_US = 1000;
constexpr int THROTTLE_MAX_US = 2000;
constexpr int ARM_THROTTLE_MAX_PERCENT = 5;
constexpr int CONTROL_CORRECTION_PERCENT = 15;
constexpr bool LOGGING_REQUIRED_TO_ARM = true;

constexpr uint32_t FAILSAFE_MS = 500;
constexpr uint32_t CONTROL_PERIOD_US = 4000;     // 250 Hz target
constexpr uint32_t MAX_CONTROL_GAP_US = 20000;  // Disarm at >20 ms while armed
constexpr uint32_t TELEMETRY_PERIOD_MS = 40;    // 25 Hz

constexpr uint32_t SAMPLE_RATE_HZ = 48000;
constexpr float SAMPLE_PERIOD_US = 1000000.0f / SAMPLE_RATE_HZ;
constexpr uint8_t ADC_BITS = 12;
constexpr uint16_t ADC_MAX_VALUE = (1u << ADC_BITS) - 1u;
constexpr uint16_t ADC_MIDPOINT = 1u << (ADC_BITS - 1u);

// ACS72981LLRATR-050B3: bidirectional +/-50 A, 26.4 mV/A at 3.3 V.
constexpr float CURRENT_SENSOR_MV_PER_AMP = 26.4f;
constexpr float CURRENT_SENSOR_SUPPLY_V = 3.3f;
constexpr float CURRENT_COUNTS_PER_AMP =
    ADC_MAX_VALUE * (CURRENT_SENSOR_MV_PER_AMP / 1000.0f) /
    CURRENT_SENSOR_SUPPLY_V;

constexpr uint16_t CURRENT_ZERO_MIN_RAW = 1200;
constexpr uint16_t CURRENT_ZERO_MAX_RAW = 2900;
constexpr uint32_t ZERO_CALIBRATION_SAMPLES = 4096;
constexpr uint16_t CLIP_LOW_RAW = 8;
constexpr uint16_t CLIP_HIGH_RAW = ADC_MAX_VALUE - 8;

constexpr uint32_t MAX_RUN_SECONDS = 120;
constexpr uint64_t MAX_DATA_BYTES =
    static_cast<uint64_t>(SAMPLE_RATE_HZ) * 2u * sizeof(int16_t) *
    MAX_RUN_SECONDS;
constexpr uint64_t PREALLOCATE_BYTES = 44u + MAX_DATA_BYTES;

constexpr uint32_t RING_CAPACITY = 16384;
constexpr uint32_t RING_MASK = RING_CAPACITY - 1u;
constexpr uint32_t WRITE_BLOCK_FRAMES = 1024;
static_assert((RING_CAPACITY & (RING_CAPACITY - 1u)) == 0,
              "RING_CAPACITY must be a power of two");

constexpr size_t MAX_COMMAND_LINE = 180;

// -------------------- Types --------------------
struct SerialLineState {
  char buffer[MAX_COMMAND_LINE];
  size_t index;
  bool overflow;
};

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

struct __attribute__((packed)) StereoFrame {
  int16_t microphone;
  int16_t current;
};
static_assert(sizeof(StereoFrame) == 4, "Stereo WAV frames must be 4 bytes");

// -------------------- Global state --------------------
Servo motorEsc[4];
bool motorOutputReady[4] = {false, false, false, false};
bool motorOutputsHealthy = false;

FlightData data = {};
SerialLineState usbRx = {};
SerialLineState radioRx = {};

bool haveValidPacket = false;
bool failsafeActive = true;
bool motorArmed = false;
bool throttleNotLow = false;
bool imuConfigured = false;
bool imuHealthy = false;
bool loopOverrun = false;
uint32_t lastPacketMs = 0;
uint32_t lastControlUs = 0;
uint32_t lastTelemetryMs = 0;

ADC adc;
IntervalTimer sampleTimer;
FsFile wavFile;
StereoFrame ringBuffer[RING_CAPACITY];
volatile uint32_t ringHead = 0;
volatile uint32_t ringTail = 0;
volatile bool recording = false;
volatile uint32_t capturedFrames = 0;
volatile uint32_t droppedFrames = 0;
volatile uint32_t adcErrorCount = 0;
volatile uint32_t microphoneClipCount = 0;
volatile uint32_t currentClipCount = 0;
volatile uint16_t microphoneMinRaw = ADC_MAX_VALUE;
volatile uint16_t microphoneMaxRaw = 0;
volatile uint16_t currentMinRaw = ADC_MAX_VALUE;
volatile uint16_t currentMaxRaw = 0;

uint64_t writtenFrames = 0;
uint16_t currentZeroRaw = ADC_MIDPOINT;
bool currentZeroValid = false;
bool sdReady = false;
bool writeFailed = false;
bool preallocationSucceeded = false;
bool stopLoggingRequested = false;
const char* stopLoggingReason = "disarmed";

char wavFilename[20] = "";
char metadataFilename[20] = "";
bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
uint32_t buttonChangedMs = 0;
uint32_t lastLoggerStatusMs = 0;

// -------------------- Forward declarations --------------------
bool startRecording();
void stopRecording(const char* reason);
void stopAllMotors();
bool calibrateCurrentZero();

// -------------------- ESC output --------------------
void writeMotorMicroseconds(size_t motorIndex, int pulseUs) {
  if (motorIndex >= 4 || !motorOutputReady[motorIndex]) {
    return;
  }
  motorEsc[motorIndex].writeMicroseconds(
      constrain(pulseUs, THROTTLE_MIN_US, THROTTLE_MAX_US));
}

void stopAllMotors() {
  data.throttle = THROTTLE_MIN_US;
  data.throttlePercent = 0;
  data.correctionRange = 0;
  data.pitchOffset = 0;
  data.rollOffset = 0;
  data.yawOffset = 0;
  for (size_t i = 0; i < 4; ++i) {
    data.motorPwm[i] = THROTTLE_MIN_US;
    writeMotorMicroseconds(i, THROTTLE_MIN_US);
  }
}

bool initializeMotorOutputs() {
  bool allReady = true;
  for (size_t i = 0; i < 4; ++i) {
    motorEsc[i].attach(MOTOR_PINS[i], THROTTLE_MIN_US, THROTTLE_MAX_US);
    motorOutputReady[i] = motorEsc[i].attached();
    allReady = allReady && motorOutputReady[i];
    if (motorOutputReady[i]) {
      motorEsc[i].writeMicroseconds(THROTTLE_MIN_US);
    }
  }
  return allReady;
}

void requestDisarm(const char* loggingReason) {
  const bool wasArmed = motorArmed;
  motorArmed = false;
  stopAllMotors();
  if ((wasArmed || recording) && recording) {
    stopLoggingRequested = true;
    stopLoggingReason = loggingReason;
  }
}

// -------------------- Command parser --------------------
bool readIntField(const char* source, const char* key, int& output) {
  char token[32];
  snprintf(token, sizeof(token), "\"%s\":", key);
  const char* start = strstr(source, token);
  if (start == nullptr) {
    return false;
  }
  start += strlen(token);
  while (*start == ' ' || *start == '\t') {
    ++start;
  }
  char* end = nullptr;
  const long value = strtol(start, &end, 10);
  if (end == start) {
    return false;
  }
  while (*end == ' ' || *end == '\t') {
    ++end;
  }
  if (*end != ',' && *end != '}') {
    return false;
  }
  output = static_cast<int>(value);
  return true;
}

bool parseControllerLine(const char* line) {
  const size_t length = strlen(line);
  if (length < 2 || line[0] != '{' || line[length - 1] != '}') {
    return false;
  }

  int lx, ly, rx, ry, cross, circle, square, triangle, l1, r1;
  if (!readIntField(line, "lx", lx) ||
      !readIntField(line, "ly", ly) ||
      !readIntField(line, "rx", rx) ||
      !readIntField(line, "ry", ry) ||
      !readIntField(line, "cross", cross) ||
      !readIntField(line, "circle", circle) ||
      !readIntField(line, "square", square) ||
      !readIntField(line, "triangle", triangle) ||
      !readIntField(line, "l1", l1) ||
      !readIntField(line, "r1", r1)) {
    return false;
  }

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

void handleUsbTextCommand(const char* line) {
  if (strcmp(line, "status") == 0) {
    Serial.print("STATUS armed=");
    Serial.print(motorArmed ? 1 : 0);
    Serial.print(" recording=");
    Serial.print(recording ? 1 : 0);
    Serial.print(" sd_ready=");
    Serial.print(sdReady ? 1 : 0);
    Serial.print(" zero_valid=");
    Serial.print(currentZeroValid ? 1 : 0);
    Serial.print(" dropped=");
    Serial.print(droppedFrames);
    Serial.print(" loop_overrun=");
    Serial.println(loopOverrun ? 1 : 0);
  } else if (strcmp(line, "zero") == 0) {
    if (motorArmed || recording) {
      Serial.println("ERROR Disarm and stop recording before zero calibration.");
    } else {
      calibrateCurrentZero();
    }
  } else if (strcmp(line, "start") == 0) {
    if (motorArmed) {
      Serial.println("ERROR Manual recording control is disabled while armed.");
    } else {
      startRecording();
    }
  } else if (strcmp(line, "stop") == 0) {
    if (motorArmed) {
      Serial.println("ERROR Release R1 before stopping the logger.");
    } else {
      stopRecording("serial");
    }
  } else if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
    Serial.println("Commands: status, zero, start, stop, help");
  } else if (line[0] != '\0') {
    Serial.print("ERROR Unknown line: ");
    Serial.println(line);
  }
}

void processSerialInput(Stream& input, SerialLineState& state, bool usbSource) {
  while (input.available() > 0) {
    const int incoming = input.read();
    if (incoming < 0) {
      break;
    }
    const char character = static_cast<char>(incoming);
    if (character == '\n') {
      if (!state.overflow && state.index > 0) {
        state.buffer[state.index] = '\0';
        if (!parseControllerLine(state.buffer) && usbSource) {
          handleUsbTextCommand(state.buffer);
        }
      }
      state.index = 0;
      state.overflow = false;
    } else if (character != '\r' && !state.overflow) {
      if (state.index < MAX_COMMAND_LINE - 1u) {
        state.buffer[state.index++] = character;
      } else {
        state.index = 0;
        state.overflow = true;
      }
    }
  }
}

// -------------------- MPU-6050 --------------------
bool writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool initializeImu() {
  return writeMpuRegister(0x6B, 0x00) &&
         writeMpuRegister(0x1A, 0x05) &&
         writeMpuRegister(0x1C, 0x10) &&
         writeMpuRegister(0x1B, 0x08);
}

bool readImuSample(float deltaSeconds, bool applyCalibration) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(static_cast<uint8_t>(0x3B));
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  const uint8_t received =
      Wire.requestFrom(MPU_ADDRESS, static_cast<uint8_t>(14),
                       static_cast<uint8_t>(true));
  if (received != 14 || Wire.available() < 14) {
    return false;
  }

  data.accelerometerX = (static_cast<int16_t>(Wire.read()) << 8) | Wire.read();
  data.accelerometerY = (static_cast<int16_t>(Wire.read()) << 8) | Wire.read();
  data.accelerometerZ = (static_cast<int16_t>(Wire.read()) << 8) | Wire.read();
  Wire.read();
  Wire.read();
  data.gyroX = (static_cast<int16_t>(Wire.read()) << 8) | Wire.read();
  data.gyroY = (static_cast<int16_t>(Wire.read()) << 8) | Wire.read();
  data.gyroZ = (static_cast<int16_t>(Wire.read()) << 8) | Wire.read();

  data.accX = static_cast<float>(data.accelerometerX) / 4096.0f;
  data.accY = static_cast<float>(data.accelerometerY) / 4096.0f;
  data.accZ = static_cast<float>(data.accelerometerZ) / 4096.0f;
  data.rateRoll = static_cast<float>(data.gyroX) / 65.5f;
  data.ratePitch = static_cast<float>(data.gyroY) / 65.5f;
  data.rateYaw = static_cast<float>(data.gyroZ) / 65.5f;

  if (applyCalibration) {
    data.rateRoll -= data.rateCalibrationRoll;
    data.ratePitch -= data.rateCalibrationPitch;
    data.rateYaw -= data.rateCalibrationYaw;
  }

  data.angleRoll = atan2(
      data.accY, sqrt(data.accX * data.accX + data.accZ * data.accZ)) *
      RAD_TO_DEG;
  data.anglePitch = -atan2(
      data.accX, sqrt(data.accY * data.accY + data.accZ * data.accZ)) *
      RAD_TO_DEG;

  if (applyCalibration) {
    data.angleYaw += data.rateYaw * deltaSeconds;
    if (data.angleYaw > 180.0f) {
      data.angleYaw -= 360.0f;
    } else if (data.angleYaw < -180.0f) {
      data.angleYaw += 360.0f;
    }
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

  for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
    if (readImuSample(0.0f, false)) {
      rollSum += data.rateRoll;
      pitchSum += data.ratePitch;
      yawSum += data.rateYaw;
      ++validSamples;
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

void kalmanUpdate(float& state, float& uncertainty, float rate,
                  float measurement, float deltaSeconds) {
  state += deltaSeconds * rate;
  uncertainty += deltaSeconds * deltaSeconds * 4.0f * 4.0f;
  const float gain = uncertainty / (uncertainty + 3.0f * 3.0f);
  state += gain * (measurement - state);
  uncertainty *= (1.0f - gain);
  state = constrain(state, -60.0f, 60.0f);
}

// -------------------- Logger helpers --------------------
int16_t rawAdcToPcm(int32_t raw) {
  int32_t centered = raw - static_cast<int32_t>(ADC_MIDPOINT);
  centered <<= (16 - ADC_BITS);
  centered = constrain(centered, static_cast<int32_t>(INT16_MIN),
                       static_cast<int32_t>(INT16_MAX));
  return static_cast<int16_t>(centered);
}

void writeLittleEndian16(FsFile& file, uint16_t value) {
  const uint8_t bytes[2] = {
      static_cast<uint8_t>(value & 0xFFu),
      static_cast<uint8_t>((value >> 8u) & 0xFFu)};
  file.write(bytes, sizeof(bytes));
}

void writeLittleEndian32(FsFile& file, uint32_t value) {
  const uint8_t bytes[4] = {
      static_cast<uint8_t>(value & 0xFFu),
      static_cast<uint8_t>((value >> 8u) & 0xFFu),
      static_cast<uint8_t>((value >> 16u) & 0xFFu),
      static_cast<uint8_t>((value >> 24u) & 0xFFu)};
  file.write(bytes, sizeof(bytes));
}

bool writeWavHeader(FsFile& file, uint32_t dataBytes) {
  constexpr uint16_t channelCount = 2;
  constexpr uint16_t bitsPerSample = 16;
  constexpr uint16_t blockAlign = channelCount * (bitsPerSample / 8u);
  constexpr uint32_t byteRate = SAMPLE_RATE_HZ * blockAlign;
  if (!file.seekSet(0)) {
    return false;
  }
  file.write("RIFF", 4);
  writeLittleEndian32(file, 36u + dataBytes);
  file.write("WAVE", 4);
  file.write("fmt ", 4);
  writeLittleEndian32(file, 16);
  writeLittleEndian16(file, 1);
  writeLittleEndian16(file, channelCount);
  writeLittleEndian32(file, SAMPLE_RATE_HZ);
  writeLittleEndian32(file, byteRate);
  writeLittleEndian16(file, blockAlign);
  writeLittleEndian16(file, bitsPerSample);
  file.write("data", 4);
  writeLittleEndian32(file, dataBytes);
  return file.getWriteError() == 0;
}

bool chooseRunFilenames() {
  for (uint16_t run = 1; run <= 9999; ++run) {
    snprintf(wavFilename, sizeof(wavFilename), "RUN%04u.WAV", run);
    if (!SD.exists(wavFilename)) {
      snprintf(metadataFilename, sizeof(metadataFilename), "RUN%04u.JSON", run);
      return true;
    }
  }
  return false;
}

void resetRunState() {
  noInterrupts();
  ringHead = 0;
  ringTail = 0;
  capturedFrames = 0;
  droppedFrames = 0;
  adcErrorCount = 0;
  microphoneClipCount = 0;
  currentClipCount = 0;
  microphoneMinRaw = ADC_MAX_VALUE;
  microphoneMaxRaw = 0;
  currentMinRaw = ADC_MAX_VALUE;
  currentMaxRaw = 0;
  interrupts();
  writtenFrames = 0;
  writeFailed = false;
  preallocationSucceeded = false;
}

void sampleIsr() {
  if (!recording) {
    return;
  }
  const ADC::Sync_result result =
      adc.analogSynchronizedRead(MICROPHONE_PIN, CURRENT_SENSOR_PIN);
  if (result.result_adc0 < 0 || result.result_adc0 > ADC_MAX_VALUE ||
      result.result_adc1 < 0 || result.result_adc1 > ADC_MAX_VALUE) {
    ++adcErrorCount;
    return;
  }

  const uint16_t microphoneRaw = static_cast<uint16_t>(result.result_adc0);
  const uint16_t currentRaw = static_cast<uint16_t>(result.result_adc1);
  microphoneMinRaw = min(microphoneMinRaw, microphoneRaw);
  microphoneMaxRaw = max(microphoneMaxRaw, microphoneRaw);
  currentMinRaw = min(currentMinRaw, currentRaw);
  currentMaxRaw = max(currentMaxRaw, currentRaw);
  if (microphoneRaw <= CLIP_LOW_RAW || microphoneRaw >= CLIP_HIGH_RAW) {
    ++microphoneClipCount;
  }
  if (currentRaw <= CLIP_LOW_RAW || currentRaw >= CLIP_HIGH_RAW) {
    ++currentClipCount;
  }

  const uint32_t nextHead = (ringHead + 1u) & RING_MASK;
  if (nextHead == ringTail) {
    ++droppedFrames;
    return;
  }
  ringBuffer[ringHead].microphone = rawAdcToPcm(microphoneRaw);
  ringBuffer[ringHead].current = rawAdcToPcm(currentRaw);
  ringHead = nextHead;
  ++capturedFrames;
}

bool drainRingBuffer() {
  if (!wavFile || writeFailed) {
    return false;
  }
  uint32_t tailSnapshot;
  uint32_t headSnapshot;
  noInterrupts();
  tailSnapshot = ringTail;
  headSnapshot = ringHead;
  interrupts();
  if (tailSnapshot == headSnapshot) {
    return true;
  }

  const uint32_t availableFrames =
      headSnapshot > tailSnapshot ? headSnapshot - tailSnapshot
                                  : RING_CAPACITY - tailSnapshot;
  const uint32_t framesToWrite = min(availableFrames, WRITE_BLOCK_FRAMES);
  const size_t bytesToWrite = framesToWrite * sizeof(StereoFrame);
  const size_t bytesWritten = wavFile.write(
      reinterpret_cast<const uint8_t*>(&ringBuffer[tailSnapshot]),
      bytesToWrite);
  if (bytesWritten != bytesToWrite || wavFile.getWriteError() != 0) {
    writeFailed = true;
    return false;
  }
  noInterrupts();
  ringTail = (tailSnapshot + framesToWrite) & RING_MASK;
  interrupts();
  writtenFrames += framesToWrite;
  return true;
}

bool calibrateCurrentZero() {
  if (motorArmed || recording) {
    Serial.println("ERROR Motors and recording must be off for zero calibration.");
    return false;
  }
  Serial.println("ZERO Keep all motors off...");
  delay(250);
  uint64_t sum = 0;
  uint32_t validSamples = 0;
  for (uint32_t i = 0; i < ZERO_CALIBRATION_SAMPLES; ++i) {
    const ADC::Sync_result result =
        adc.analogSynchronizedRead(MICROPHONE_PIN, CURRENT_SENSOR_PIN);
    if (result.result_adc1 >= 0 && result.result_adc1 <= ADC_MAX_VALUE) {
      sum += static_cast<uint16_t>(result.result_adc1);
      ++validSamples;
    }
    delayMicroseconds(20);
  }
  if (validSamples < ZERO_CALIBRATION_SAMPLES / 2u) {
    currentZeroValid = false;
    Serial.println("ERROR Current ADC returned too few valid samples.");
    return false;
  }
  currentZeroRaw =
      static_cast<uint16_t>((sum + validSamples / 2u) / validSamples);
  currentZeroValid = currentZeroRaw >= CURRENT_ZERO_MIN_RAW &&
                     currentZeroRaw <= CURRENT_ZERO_MAX_RAW;
  Serial.print("ZERO current_zero_adc=");
  Serial.print(currentZeroRaw);
  Serial.print(" estimated_zero_amps=");
  Serial.println((static_cast<float>(currentZeroRaw) - ADC_MIDPOINT) /
                     CURRENT_COUNTS_PER_AMP,
                 3);
  if (!currentZeroValid) {
    Serial.println("ERROR Check ACS72981 3.3 V, GND, OUT, and A3 wiring.");
  }
  return currentZeroValid;
}

bool writeMetadataFile(uint32_t dataBytes) {
  FsFile metadata = SD.sdfs.open(metadataFilename, O_WRITE | O_CREAT | O_TRUNC);
  if (!metadata) {
    return false;
  }
  metadata.println("{");
  metadata.print("  \"wav_file\": \"");
  metadata.print(wavFilename);
  metadata.println("\",");
  metadata.println("  \"format\": \"stereo_pcm_s16le\",");
  metadata.print("  \"sample_rate_hz\": ");
  metadata.print(SAMPLE_RATE_HZ);
  metadata.println(",");
  metadata.println("  \"channel_1\": \"microphone\",");
  metadata.println("  \"channel_2\": \"current_sensor\",");
  metadata.print("  \"adc_bits\": ");
  metadata.print(ADC_BITS);
  metadata.println(",");
  metadata.print("  \"adc_midpoint\": ");
  metadata.print(ADC_MIDPOINT);
  metadata.println(",");
  metadata.println("  \"current_sensor\": \"ACS72981LLRATR-050B3\",");
  metadata.print("  \"current_zero_adc\": ");
  metadata.print(currentZeroRaw);
  metadata.println(",");
  metadata.print("  \"current_counts_per_amp\": ");
  metadata.print(CURRENT_COUNTS_PER_AMP, 6);
  metadata.println(",");
  metadata.print("  \"captured_frames\": ");
  metadata.print(capturedFrames);
  metadata.println(",");
  metadata.print("  \"written_frames\": ");
  metadata.print(static_cast<uint32_t>(writtenFrames));
  metadata.println(",");
  metadata.print("  \"data_bytes\": ");
  metadata.print(dataBytes);
  metadata.println(",");
  metadata.print("  \"duration_seconds\": ");
  metadata.print(static_cast<double>(writtenFrames) / SAMPLE_RATE_HZ, 6);
  metadata.println(",");
  metadata.print("  \"dropped_frames\": ");
  metadata.print(droppedFrames);
  metadata.println(",");
  metadata.print("  \"adc_error_count\": ");
  metadata.print(adcErrorCount);
  metadata.println(",");
  metadata.print("  \"microphone_clip_count\": ");
  metadata.print(microphoneClipCount);
  metadata.println(",");
  metadata.print("  \"current_clip_count\": ");
  metadata.print(currentClipCount);
  metadata.println(",");
  metadata.print("  \"microphone_min_adc\": ");
  metadata.print(microphoneMinRaw);
  metadata.println(",");
  metadata.print("  \"microphone_max_adc\": ");
  metadata.print(microphoneMaxRaw);
  metadata.println(",");
  metadata.print("  \"current_min_adc\": ");
  metadata.print(currentMinRaw);
  metadata.println(",");
  metadata.print("  \"current_max_adc\": ");
  metadata.print(currentMaxRaw);
  metadata.println(",");
  metadata.print("  \"preallocation_succeeded\": ");
  metadata.print(preallocationSucceeded ? "true" : "false");
  metadata.println(",");
  metadata.print("  \"control_loop_overrun\": ");
  metadata.print(loopOverrun ? "true" : "false");
  metadata.println(",");
  metadata.print("  \"write_failed\": ");
  metadata.println(writeFailed ? "true" : "false");
  metadata.println("}");
  const bool ok = metadata.getWriteError() == 0;
  metadata.close();
  return ok;
}

bool startRecording() {
  if (recording) {
    return true;
  }
  if (!sdReady || !currentZeroValid || !chooseRunFilenames()) {
    Serial.println("ERROR Logger is not ready; arming remains blocked.");
    return false;
  }
  resetRunState();
  wavFile = SD.sdfs.open(wavFilename, O_RDWR | O_CREAT | O_TRUNC);
  if (!wavFile) {
    Serial.println("ERROR Could not create logger WAV file.");
    return false;
  }
  preallocationSucceeded = wavFile.preAllocate(PREALLOCATE_BYTES);
  if (!writeWavHeader(wavFile, 0) || !wavFile.seekSet(44)) {
    wavFile.close();
    Serial.println("ERROR Could not prepare logger WAV file.");
    return false;
  }
  digitalWriteFast(RECORD_LED_PIN, HIGH);
  recording = true;
  sampleTimer.priority(64);
  if (!sampleTimer.begin(sampleIsr, SAMPLE_PERIOD_US)) {
    recording = false;
    digitalWriteFast(RECORD_LED_PIN, LOW);
    wavFile.close();
    Serial.println("ERROR No IntervalTimer resource is available.");
    return false;
  }
  Serial.print("RECORDING file=");
  Serial.println(wavFilename);
  return true;
}

void stopRecording(const char* reason) {
  if (!recording && !wavFile) {
    return;
  }
  recording = false;
  sampleTimer.end();
  digitalWriteFast(RECORD_LED_PIN, LOW);
  while (ringTail != ringHead && !writeFailed) {
    drainRingBuffer();
  }
  const uint64_t dataBytes64 =
      writtenFrames * static_cast<uint64_t>(sizeof(StereoFrame));
  const uint32_t dataBytes = dataBytes64 > UINT32_MAX
                                 ? UINT32_MAX
                                 : static_cast<uint32_t>(dataBytes64);
  bool finalized = !writeFailed;
  if (wavFile) {
    finalized = wavFile.truncate(44u + dataBytes64) && finalized;
    finalized = writeWavHeader(wavFile, dataBytes) && finalized;
    finalized = wavFile.sync() && finalized;
    wavFile.close();
  }
  const bool metadataOk = writeMetadataFile(dataBytes);
  Serial.print("STOPPED reason=");
  Serial.print(reason);
  Serial.print(" file=");
  Serial.print(wavFilename);
  Serial.print(" dropped=");
  Serial.println(droppedFrames);
  if (!finalized || !metadataOk) {
    Serial.println("ERROR Logger file finalization was not clean.");
  }
}

void pollRecordButton() {
  const bool reading = digitalReadFast(RECORD_BUTTON_PIN);
  if (reading != lastButtonReading) {
    lastButtonReading = reading;
    buttonChangedMs = millis();
  }
  if (millis() - buttonChangedMs >= 30u && reading != stableButtonState) {
    stableButtonState = reading;
    if (stableButtonState == LOW) {
      if (motorArmed) {
        Serial.println("INFO Record button ignored while armed.");
      } else if (recording) {
        stopRecording("button");
      } else {
        startRecording();
      }
    }
  }
}

// -------------------- Motor mixer --------------------
void updateMotors() {
  throttleNotLow = false;
  if (data.cross || !data.r1) {
    requestDisarm(data.cross ? "kill" : "r1_released");
    return;
  }
  if (!imuHealthy || !motorOutputsHealthy ||
      (LOGGING_REQUIRED_TO_ARM && (!sdReady || !currentZeroValid))) {
    requestDisarm("hardware_not_ready");
    return;
  }

  data.throttlePercent = constrain(data.ly, 0, 100);
  if (!motorArmed) {
    if (data.throttlePercent > ARM_THROTTLE_MAX_PERCENT) {
      throttleNotLow = true;
      stopAllMotors();
      return;
    }
    if (LOGGING_REQUIRED_TO_ARM && !recording && !startRecording()) {
      stopAllMotors();
      return;
    }
    loopOverrun = false;
    lastControlUs = micros();
    motorArmed = true;
  }

  data.pitch = constrain(data.ry, -100, 100);
  data.roll = constrain(data.rx, -100, 100);
  data.yaw = constrain(data.lx, -100, 100);
  data.throttle = map(data.throttlePercent, 0, 100,
                      THROTTLE_MIN_US, THROTTLE_MAX_US);
  data.correctionRange =
      (static_cast<long>(data.throttle - THROTTLE_MIN_US) *
       CONTROL_CORRECTION_PERCENT) /
      100L;
  data.pitchOffset = map(data.pitch, -100, 100,
                         -data.correctionRange, data.correctionRange);
  data.rollOffset = map(data.roll, -100, 100,
                        -data.correctionRange, data.correctionRange);
  data.yawOffset = map(data.yaw, -100, 100,
                       -data.correctionRange, data.correctionRange);

  const long limitedPitchAngle =
      static_cast<long>(constrain(data.kalmanAnglePitch, -80.0f, 80.0f));
  const long limitedRollAngle =
      static_cast<long>(constrain(data.kalmanAngleRoll, -80.0f, 80.0f));
  const int pitchGyro = map(limitedPitchAngle, -80, 80,
                            -data.correctionRange, data.correctionRange);
  const int rollGyro = map(limitedRollAngle, -80, 80,
                           -data.correctionRange, data.correctionRange);

  data.motorPwm[0] = data.throttle + pitchGyro + rollGyro -
                     data.pitchOffset + data.rollOffset + data.yawOffset;
  data.motorPwm[1] = data.throttle - pitchGyro + rollGyro +
                     data.pitchOffset + data.rollOffset - data.yawOffset;
  data.motorPwm[2] = data.throttle + pitchGyro - rollGyro -
                     data.pitchOffset - data.rollOffset - data.yawOffset;
  data.motorPwm[3] = data.throttle - pitchGyro - rollGyro +
                     data.pitchOffset - data.rollOffset + data.yawOffset;

  if (data.lx == 0) {
    const long limitedYawRate =
        static_cast<long>(constrain(data.yawRate, -125.0f, 125.0f));
    const int yawGyro = map(limitedYawRate, -125, 125,
                            -data.correctionRange, data.correctionRange);
    data.motorPwm[0] += yawGyro;
    data.motorPwm[1] -= yawGyro;
    data.motorPwm[2] -= yawGyro;
    data.motorPwm[3] += yawGyro;
  }

  for (size_t i = 0; i < 4; ++i) {
    data.motorPwm[i] =
        constrain(data.motorPwm[i], THROTTLE_MIN_US, THROTTLE_MAX_US);
    writeMotorMicroseconds(i, data.motorPwm[i]);
  }
}

// -------------------- Telemetry --------------------
void sendTelemetry(Print& output) {
  output.print("ACK motor_pwm=");
  for (size_t i = 0; i < 4; ++i) {
    if (i) output.print(" ");
    output.print(data.motorPwm[i]);
  }
  output.print(" dir=");
  for (size_t i = 0; i < 4; ++i) {
    if (i) output.print(" ");
    output.print(MOTOR_DIRECTIONS[i]);
  }
  output.print(" lx="); output.print(data.lx);
  output.print(" ly="); output.print(data.ly);
  output.print(" rx="); output.print(data.rx);
  output.print(" ry="); output.print(data.ry);
  output.print(" Roll [deg]: "); output.print(data.kalmanAngleRoll, 2);
  output.print(" Pitch [deg]: "); output.print(data.kalmanAnglePitch, 2);
  output.print(" Yaw [deg]: "); output.print(data.angleYaw, 2);
  output.print(" Yaw Rate [deg/s]: "); output.print(data.yawRate, 2);
  if (data.cross) output.print(" KILL");
  if (!data.r1 || !motorArmed) output.print(" MOTOR_LOCKED");
  if (throttleNotLow) output.print(" THROTTLE_NOT_LOW");
  if (failsafeActive) output.print(" FAILSAFE");
  if (!imuHealthy) output.print(" IMU_ERROR");
  if (!motorOutputsHealthy) output.print(" PWM_ERROR");
  if (loopOverrun) output.print(" OVERRUN");
  output.print(" SERVOS_DORMANT");
  output.println();
}

// -------------------- Setup and main loop --------------------
void initializeFlightData() {
  data.kalmanUncertaintyRoll = 4.0f;
  data.kalmanUncertaintyPitch = 4.0f;
  stopAllMotors();
}

void setup() {
  pinMode(RECORD_LED_PIN, OUTPUT);
  digitalWriteFast(RECORD_LED_PIN, LOW);
  pinMode(RECORD_BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(USB_BAUD);
  Serial1.begin(RADIO_UART_BAUD);
  const uint32_t serialWaitStarted = millis();
  while (!Serial && millis() - serialWaitStarted < 3000u) {
  }
  Serial.println("AmbatuDrone Teensy flight/logger starting. REMOVE PROPELLERS.");

  initializeFlightData();
  motorOutputsHealthy = initializeMotorOutputs();
  stopAllMotors();

  adc.adc0->setAveraging(0);
  adc.adc0->setResolution(ADC_BITS);
  adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
  adc.adc1->setAveraging(0);
  adc.adc1->setResolution(ADC_BITS);
  adc.adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc.adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);

  sdReady = SD.sdfs.begin(SdioConfig(FIFO_SDIO));
  if (!sdReady) {
    Serial.println("ERROR microSD initialization failed.");
  }
  calibrateCurrentZero();

  Wire.begin();
  Wire.setClock(400000);
  Serial.println("Keep the MPU-6050 still: calibrating...");
  imuConfigured = initializeImu();
  imuHealthy = imuConfigured && calibrateImu();

  stopAllMotors();
  delay(3000);
  haveValidPacket = false;
  failsafeActive = true;
  motorArmed = false;
  lastControlUs = micros();
  lastTelemetryMs = millis();
  Serial.println("AmbatuDrone Teensy flight/logger ready.");
}

void loop() {
  processSerialInput(Serial, usbRx, true);
  processSerialInput(Serial1, radioRx, false);
  pollRecordButton();

  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastControlUs;
  if (elapsedUs >= CONTROL_PERIOD_US) {
    if (motorArmed && elapsedUs > MAX_CONTROL_GAP_US) {
      loopOverrun = true;
      requestDisarm("control_loop_overrun");
    }

    const float deltaSeconds =
        constrain(elapsedUs * 0.000001f, 0.001f, 0.020f);
    lastControlUs = nowUs;
    imuHealthy = imuConfigured && readImuSample(deltaSeconds, true);
    if (imuHealthy) {
      kalmanUpdate(data.kalmanAngleRoll, data.kalmanUncertaintyRoll,
                   data.rateRoll, data.angleRoll, deltaSeconds);
      kalmanUpdate(data.kalmanAnglePitch, data.kalmanUncertaintyPitch,
                   data.ratePitch, data.anglePitch, deltaSeconds);
    }

    failsafeActive =
        !haveValidPacket || (millis() - lastPacketMs > FAILSAFE_MS);
    if (failsafeActive) {
      data.r1 = 0;
      throttleNotLow = false;
      requestDisarm("link_failsafe");
    } else if (!loopOverrun) {
      updateMotors();
    } else {
      requestDisarm("control_loop_overrun");
    }
  }

  if (stopLoggingRequested && !motorArmed) {
    stopLoggingRequested = false;
    stopRecording(stopLoggingReason);
  }

  if (recording) {
    if (!drainRingBuffer()) {
      requestDisarm("sd_write_error");
    } else if (capturedFrames >= SAMPLE_RATE_HZ * MAX_RUN_SECONDS) {
      requestDisarm("maximum_recording_duration");
    }
    if (millis() - lastLoggerStatusMs >= 1000u) {
      lastLoggerStatusMs = millis();
      Serial.print("LOGGER file=");
      Serial.print(wavFilename);
      Serial.print(" captured=");
      Serial.print(capturedFrames);
      Serial.print(" dropped=");
      Serial.println(droppedFrames);
    }
  }

  if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = millis();
    sendTelemetry(Serial);
    sendTelemetry(Serial1);
  }
}
