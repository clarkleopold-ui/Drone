#include <Arduino.h>
#include <ADC.h>
#include <IntervalTimer.h>
#include <Servo.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <cstdio>
#include <cstring>
#include "SparkFun_BNO080_Arduino_Library.h"

/*
  AmbatuDrone Teensy 4.1 combined flight controller and motor-health logger
  Version: 2026-08-18

  BENCH-TEST BUILD. REMOVE ALL PROPELLERS WHEN TESTING.


  Command paths:
    Computer app -> Teensy USB Serial
    Radiomaster Pocket ELRS Remote Controller -> RadioMaster 2.4GHz RP1 ELRS -> Teensy Serial1 ->
    Radiomaster Pocket ELRS Remote Controller -> computer app -> drone app

  Flight hardware:
    ESC 4 in 1: pins 3, 4, 5, 6
    IMU/BNO080: SDA 18, SCL 19

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
//Baud rates for usb and for radio communication
constexpr uint32_t USB_BAUD = 115200; 
constexpr uint32_t RADIO_UART_BAUD = 115200;
// ---------- Pin assignment ----------
constexpr uint8_t MOTOR_PINS[4] = {3, 4, 5, 6};
constexpr char MOTOR_DIRECTIONS[4] = {'+', '-', '-', '+'}; //+ = CW and - = CCW
constexpr uint8_t RECORD_BUTTON_PIN = 2;
constexpr uint8_t RECORD_LED_PIN = LED_BUILTIN;
constexpr uint8_t MICROPHONE_PIN = A2;
constexpr uint8_t CURRENT_SENSOR_PIN = A3;
// ---------- ESC output ----------
constexpr int THROTTLE_MIN_US = 1000;
constexpr int THROTTLE_MAX_US = 2000;
constexpr int ARM_THROTTLE_MAX_PERCENT = 5; //variable to limit motor PWM values to a maximum 5% when calibrating the drone
constexpr int CONTROL_CORRECTION_PERCENT = 20; //variable to calculate the the PWM values for each motor
constexpr bool LOGGING_REQUIRED_TO_ARM = false;

constexpr uint32_t FAILSAFE_MS = 500; //A variable to store a cap to trigger a failsafe if the cap has been reached or exceeded the cap
constexpr uint32_t CONTROL_PERIOD_US = 4000; //Variable to store the frequency of PWM values being sent to motors 250 Hz
constexpr uint32_t MAX_CONTROL_GAP_US = 20000;  // Disarm at >20 ms while armed
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;    //Variable to call Telemetry function every 100 milisec for data transfer to custom app. 
//Was at 40 which is 25 Hz but updated it to send data faster

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

constexpr size_t MAX_COMMAND_LINE = 180; //Maximum number of indexes for data to be transfered into teensy from python files
// -------------------- IMU --------------------
constexpr uint8_t IMU_ADDRESS = 0x4B;
// pin declaration for the IMU on the teensy 4.1
constexpr uint8_t SDA_PIN = 18; 
constexpr uint8_t SCL_PIN = 19; 
BNO080 myIMU

// -------------------- Types --------------------
//Struct to hold variables for data transfer between teensy 4.1, python files, and ps4 controller
struct SerialLineState {
  char buffer[MAX_COMMAND_LINE];
  size_t index;
  bool overflow;
};
//Struct to hold all variables regarding ps4 controller inputs, motor PWM values,
//MPU values: gyro, accelerometer, magnotometer, and angle values.

//Plan to remove a lot of angle variables assosiated with kalman filter since
//a different IMu is now being used.
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
//Struct to hold variables regarding current sensor and microphones
struct __attribute__((packed)) StereoFrame {
  int16_t microphone;
  int16_t current;
};
static_assert(sizeof(StereoFrame) == 4, "Stereo WAV frames must be 4 bytes");

// -------------------- Global state --------------------
Servo motorEsc[4]; //Declare each motor through servo library
bool motorOutputReady[4] = {false, false, false, false}; //bool to check if each motor is ready to receive PWM values 
bool motorOutputsHealthy = false; //bool for checking if the outputs to motors are stable

FlightData data = {};
SerialLineState usbRx = {};
SerialLineState radioRx = {};

bool haveValidPacket = false; //bool to check if data is being sent to teensy
bool failsafeActive = true; //bool to check the failsafe in place for the drone
bool motorArmed = false; //bool to check if motors were configured
bool throttleNotLow = false; //bool to check if motors are spinning
bool imuConfigured = false; //bool to check if IMU was configured correctly
bool imuHealthy = true; //bool to check IMU health
bool loopOverrun = false; //bool to check if the entire program is looping too fast
uint32_t lastPacketMs = 0; //variable to check the last pieces of data to was recieved by teensy
uint32_t lastControlUs = 0; //variable to check the last time the Telemetry function was ran
uint32_t lastTelemetryMs = 0; //timer to track the intervals when the Telemetry function should be called

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
//Function to output PWM values to motors for them to spin
void writeMotorMicroseconds(size_t motorIndex, int pulseUs) {
  //If the index for any motor array is 4 or more or any motorOutputReady is false
  //don't update the motors and return early
  if (motorIndex >= 4 || !motorOutputReady[motorIndex]) {
    return;
  }
  //Send values to motors through esc
  motorEsc[motorIndex].writeMicroseconds(
      constrain(pulseUs, THROTTLE_MIN_US, THROTTLE_MAX_US));
}
//Function to force all motors off regardless of any user input from ps4 controller
//as a safety feature for the drone
void stopAllMotors() {
  //Set all variables that can manipulate PWM values to zero and 
  //the throttle valriable to the lowest value of 1000
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
//Function to initialize each motor to check if they are ready and return their ready states
bool initializeMotorOutputs() {
  bool allReady = true;
  //Run a for loop to initialize each motor's pin, frequency and resolution
  for (size_t i = 0; i < 4; ++i) {
    motorEsc[i].attach(MOTOR_PINS[i], THROTTLE_MIN_US, THROTTLE_MAX_US);
    motorOutputReady[i] = motorEsc[i].attached();
    allReady = allReady && motorOutputReady[i];
    if (motorOutputReady[i]) {
      motorEsc[i].writeMicroseconds(THROTTLE_MIN_US);
    }
  }
  //return bool when done running this function
  return allReady;
}
//Function to stop all hardware if there are any errors or manual input was sent
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
//This function reads the parsed data from the function processSerialInput
//and converts that data from a char to a number
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
//From the readIntField function parse some of that data into local ps4 controller variables 
//in this code for the esp32 to understand inputs from the ps4
bool parseControllerLine(const char* line) {
  const size_t length = strlen(line);
  //Checks if the data incoming starts with a '{' if not then the data
  //is not sufficent and return early
  if (length < 2 || line[0] != '{' || line[length - 1] != '}') {
    return false;
  }

  int lx, ly, rx, ry, cross, circle, square, triangle, l1, r1;
  //If there is no data for any of the inputs on the ps4 then return early
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
  //Commit only after the complete packet is valid. A malformed packet never
  //refreshes the communication failsafe.
  //Stores each input button/joystick value from ps4 controller into their corresponding 
  //variables for the teensy to recognize while also contraining them.
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
  //Set haveValidPacket to true and updates the last time a packet of data has been
  //transfer through lastPacketMs since the function has reached the bottom
  //and there is sufficent data transfer
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
//Function to parse any incoming raw data from python files into 
//here and turns it into data is able to be read from the teensy.
//Raw data are in forms of chars
void processSerialInput(Stream& input, SerialLineState& state, bool usbSource) {
  //using a while loop check if there is any data available
  while (input.available() > 0) {
    const int incoming = input.read();
    //If there is no incoming data end the while loop early
    if (incoming < 0) {
      break;
    }
    //Checks if there is a new line to the data
    const char character = static_cast<char>(incoming);
    if (character == '\n') {
      //Checks if there is any overflow in the data and if there isn't,
      //set a null terminator to validate the data and send data to the
      //function parseControllerLine
      if (!state.overflow && state.index > 0) {
        state.buffer[state.index] = '\0';
        if (!parseControllerLine(state.buffer) && usbSource) {
          handleUsbTextCommand(state.buffer);
        }
      }
      state.index = 0;
      state.overflow = false;
      //If the data doesn't have carriage returns then append
      //data to the array called buffer if there is room in the array
    } else if (character != '\r' && !state.overflow) {
      if (state.index < MAX_COMMAND_LINE - 1u) {
        state.buffer[state.index++] = character;
        //For protection against overflow of data, if there is no
        //room in the arry set overflow to true and reset the index back to 0
      } else {
        state.index = 0;
        state.overflow = true;
      }
    }
  }
}

// -------------------- MPU-6050 --------------------
//These function are commented out for now since MPU-6050 is not being used since
//BNO080 is now being used. 
//Function to parse addresses to MPU to to set the MPU to output
//correct values through the gyroscope and accelerometer
/*bool writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}
//Function to send addresses to MPU-6050 to start the communcation proccess 
//for gyroscope and accelerometer measurements
bool initializeImu() {
  return writeMpuRegister(0x6B, 0x00) &&
         writeMpuRegister(0x1A, 0x05) &&
         writeMpuRegister(0x1C, 0x10) &&
         writeMpuRegister(0x1B, 0x08);
}*/
//Communicate with the IMU to gather gyroscope and accelerometer values.
//Then convert those values into g forces for accelerometer and degrees/sec for gyroscope.
//Lastly, with those values calculate the angle of the pitch, roll, and yaw at real time
bool readImuSample(float deltaSeconds, bool applyCalibration) {
  /*Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(static_cast<uint8_t>(0x3B));
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  //Store a Wire.requestFrom into the variable received with 14 bytes of memory
  const uint8_t received =
      Wire.requestFrom(MPU_ADDRESS, static_cast<uint8_t>(14),
                       static_cast<uint8_t>(true));
  //If received is not 14 bytes or there are atleast 14 bytes
  //in the teensy's I2C then return a false early for safety
  if (received != 14 || Wire.available() < 14) {
    return false;
  }
  //Gather data for each axis of the accelerometer
  data.accelerometerX = (static_cast<int16_t>(Wire.read()) << 8) | Wire.read();
  data.accelerometerY = (static_cast<int16_t>(Wire.read()) << 8) | Wire.read();
  data.accelerometerZ = (static_cast<int16_t>(Wire.read()) << 8) | Wire.read();
  // MPU temperature is not used by the flight loop.
  Wire.read();
  Wire.read();
  //Gather data for each axis of the gyroscope
  data.gyroX = (static_cast<int16_t>(Wire.read()) << 8) | Wire.read();
  data.gyroY = (static_cast<int16_t>(Wire.read()) << 8) | Wire.read();
  data.gyroZ = (static_cast<int16_t>(Wire.read()) << 8) | Wire.read();
  //Convert the raw data from each acceleroemter axis
  //into g forces
  data.accX = static_cast<float>(data.accelerometerX) / 4096.0f;
  data.accY = static_cast<float>(data.accelerometerY) / 4096.0f;
  data.accZ = static_cast<float>(data.accelerometerZ) / 4096.0f;
  //Convert the raw data from each gyroscope axis
  //into degrees/sec
  data.rateRoll = static_cast<float>(data.gyroX) / 65.5f;
  data.ratePitch = static_cast<float>(data.gyroY) / 65.5f;
  data.rateYaw = static_cast<float>(data.gyroZ) / 65.5f;
  //If applyCalibration is true update
  if (applyCalibration) {
    data.rateRoll -= data.rateCalibrationRoll;
    data.ratePitch -= data.rateCalibrationPitch;
    data.rateYaw -= data.rateCalibrationYaw;
  }
  //Calculate the roll and pitch angle and converts them to degrees
  data.angleRoll = atan2(
      data.accY, sqrt(data.accX * data.accX + data.accZ * data.accZ)) *
      RAD_TO_DEG;
  data.anglePitch = -atan2(
      data.accX, sqrt(data.accY * data.accY + data.accZ * data.accZ)) *
      RAD_TO_DEG;
  //If applyCalibration is true update the angles for roll and pitch based on their offsets declared
  //as a constant and calculate the angle of the yaw motion.
  //(Magnetometer is not being used so the drone will not know where relative north is meaning
  //the yaw angle will only update and be based off when the MPU starts calculating)
  if (applyCalibration) {
    data.angleYaw += data.rateYaw * deltaSeconds;
    if (data.angleYaw > 180.0f) {
      data.angleYaw -= 360.0f;
    } else if (data.angleYaw < -180.0f) {
      data.angleYaw += 360.0f;
    }
    data.yawRate = data.rateYaw;
  }*/
    
  //if there is data available gather the roll, pitch, and yaw angles from the SparkFun_BNO080_Arduino_Library
  if(myIMU.dataAvailable()) {
    data.kalmanAngleRoll = myIMU.getRoll() * RAD_TO_DEG;
    data.kalmanAnglePitch = myIMU.getPitch() * RAD_TO_DEG;
    data.angleYaw = myIMU.getYaw() * RAD_TO_DEG;
  } else 
    return false;
  return true;
}
//Function to calibrate the Imu/MPU to ultimatly determine
//if the IMU is healthy and working correctly
bool calibrateImu() {
  /*constexpr int CALIBRATION_SAMPLES = 2000;
  int validSamples = 0;
  float rollSum = 0.0f;
  float pitchSum = 0.0f;
  float yawSum = 0.0f;
  //Loop 200 times with 1ms delay to call the function readImuSample. Then
  //checks to see if data from the MPU is valid and adds the roll, pitch, and yaw
  //values to a sum.
  for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
    if (readImuSample(0.0f, false)) {
      rollSum += data.rateRoll;
      pitchSum += data.ratePitch;
      yawSum += data.rateYaw;
      //Counts how many valid pieces of data there where through the whole loop
      ++validSamples;
    }
    delay(1);
  }
  //If the number of valid data is less than 90% correct then return
  //false saying the MPU was not calibrated correctly and is not healthy
  if (validSamples < CALIBRATION_SAMPLES * 9 / 10) {
    return false;
  }*/
  //if myIMU.begin returns false return early and return a false indicating the IMU wasn't calibrated correctly
  if(!myIMU.begin(IMU_ADDRESS, Wire)) {
    Serial.println("IMU not detected at default I2C address. Check your jumpers and the hookup guide. Freezing");
    return false;
  }
  //The sum values are then used to calculate the calibration values for each degree of movement
  /*data.rateCalibrationRoll = rollSum / validSamples;
  data.rateCalibrationPitch = pitchSum / validSamples;
  data.rateCalibrationYaw = yawSum / validSamples;
  //Reset roll, pitch, and yaw rate values to prepare MPU to 
  //run and track drone orientation
  data.rateRoll = 0.0f;
  data.ratePitch = 0.0f;
  data.rateYaw = 0.0f;
  data.yawRate = 0.0f;
  data.angleYaw = 0.0f;*/
  return true;
}
//Function for the Kalman filter; comparing real angle values to predicted angle values
//for roll and pitch then updating the predicted values based on the real values. 
//Creating a controlled feedback loop (Currently not being used)
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
//Function to update PWM values for the motors from ps4 inputs and IMU angle values
void updateMotors() {
  throttleNotLow = false;
  //If the cross button on ps4 is pressed turn off all motors
  if (data.cross || !data.r1) {
    requestDisarm(data.cross ? "kill" : "r1_released");
    return;
  }
  //If there is any issues with the motor health and IMU health turn off the motors
  if (!imuHealthy || !motorOutputsHealthy ||
      (LOGGING_REQUIRED_TO_ARM && (!sdReady || !currentZeroValid))) {
    requestDisarm("hardware_not_ready");
    return;
  }
  //Constrain throttlePercent values from 0% to 100%
  data.throttlePercent = constrain(data.ly, 0, 100);

  // Every transition from disarmed to armed must happen at 5% or lower.
  if (!motorArmed) {
    //If the throttle Percent is greater than ARM_THROTTLE_MAX_PERCENT (5%) then stop all the motors
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

  //Map the throttle precents to speeds/pwm values for each motor then store
  //it in the variable data.throttle
  data.throttle = map(data.throttlePercent, 0, 100,
                      THROTTLE_MIN_US, THROTTLE_MAX_US);
  data.correctionRange =
      (static_cast<long>(data.throttle - THROTTLE_MIN_US) *
       CONTROL_CORRECTION_PERCENT) /
      100L;
  //Set pitch, yaw, and roll offsets via mapping controller
  //joystick values to correction values
  data.pitchOffset = map(data.pitch, -100, 100,
                         -data.correctionRange, data.correctionRange);
  data.rollOffset = map(data.roll, -100, 100,
                        -data.correctionRange, data.correctionRange);
  data.yawOffset = map(data.yaw, -100, 100,
                       -data.correctionRange, data.correctionRange);
  //Gather angles from IMU and contrain their angles
  const long limitedPitchAngle =
      static_cast<long>(constrain(data.kalmanAnglePitch, -80.0f, 80.0f));
  const long limitedRollAngle =
      static_cast<long>(constrain(data.kalmanAngleRoll, -80.0f, 80.0f));
  const int pitchGyro = map(limitedPitchAngle, -80, 80,
                            -data.correctionRange, data.correctionRange);
  const int rollGyro = map(limitedRollAngle, -80, 80,
                           -data.correctionRange, data.correctionRange);
  const int yawGyro = map(data.angleYaw, -360, 360,
                           -data. , data.correctionRange);                
  //Add all possible variables that contribute to PWM values for each motor.
  //Motor order and signs match the corrected motor maintaing a balanced orientation of the drone.
  data.motorPwm[0] = data.throttle + pitchGyro + rollGyro - yawGyro -
                     data.pitchOffset + data.rollOffset + data.yawOffset;
  data.motorPwm[1] = data.throttle - pitchGyro + rollGyro + yawGyro +
                     data.pitchOffset + data.rollOffset - data.yawOffset;
  data.motorPwm[2] = data.throttle + pitchGyro - rollGyro + yawGyro -
                     data.pitchOffset - data.rollOffset - data.yawOffset;
  data.motorPwm[3] = data.throttle - pitchGyro - rollGyro - yawGyro +
                     data.pitchOffset - data.rollOffset + data.yawOffset;
  
  //Constrain the arry data.motorPwm storing the PWM values once more before writing to each motor to
  //avoid too much current draw through the ESC and to stay with in the ranges of 1000 to 2000.
  for (size_t i = 0; i < 4; ++i) {
    data.motorPwm[i] =
        constrain(data.motorPwm[i], THROTTLE_MIN_US, THROTTLE_MAX_US);
    writeMotorMicroseconds(i, data.motorPwm[i]); //write PWM value to each motor
  }
}

// -------------------- Telemetry --------------------
//Function to print and send data to python files to be shown on drone app (plan to use radio frequency)
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
  //When cross on ps4 controller is pressed send KILL to python files
  if (data.cross) output.print(" KILL");
  //When r1 on ps4 controller isn't pressed and motorArmed is false send MOTOR_LOCKED 
  if (!data.r1 || !motorArmed) output.print(" MOTOR_LOCKED");
  //When throttleNotLow is true send THROTTLE_NOT_LOW 
  if (throttleNotLow) output.print(" THROTTLE_NOT_LOW");
  //When failsafeActive is true send FAILSAFE 
  if (failsafeActive) output.print(" FAILSAFE");
  //When imuHealthy is false send IMU_ERROR 
  if (!imuHealthy) output.print(" IMU_ERROR");
  //When motorOutputsHealthy is false send PWM_ERROR 
  if (!motorOutputsHealthy) output.print(" PWM_ERROR");
  //If loopOverrun is true then send "OVERRUN"
  if (loopOverrun) output.print(" OVERRUN");
  //output.print(" SERVOS_DORMANT"); not needed
  output.println();
}

// -------------------- Setup and main loop --------------------
//Function to initialize kalman uncertainly variables 
//(not needed since kalman filter is not being used)
void initializeFlightData() {
  data.kalmanUncertaintyRoll = 4.0f;
  data.kalmanUncertaintyPitch = 4.0f;
}

void setup() {
  pinMode(RECORD_LED_PIN, OUTPUT);
  digitalWriteFast(RECORD_LED_PIN, LOW);
  pinMode(RECORD_BUTTON_PIN, INPUT_PULLUP);
  //Set the baud rate to 115200
  Serial.begin(USB_BAUD);
  Serial1.begin(RADIO_UART_BAUD);
  const uint32_t serialWaitStarted = millis();
  while (!Serial && millis() - serialWaitStarted < 3000u) {
  }
  Serial.println("AmbatuDrone Teensy flight/logger starting. REMOVE PROPELLERS WHEN TESTING.");

  //Check if the motors are healthy and initialize
  //kalman filter values through the function initializeFlightData();
  motorOutputsHealthy = initializeMotorOutputs();
  //initializeFlightData();
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
  //Send min value to ESC for calibration
  stopAllMotors();
  delay(3000);
  //Begin communication with MPU by sending the pin numbers and communication speed
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  //Serial.println("Keep the MPU-6050 still: calibrating...");

  //Check if the IMU/MPU is calibrated and healthy and if the
  //bluetooth connection is healthy  
  //imuConfigured = initializeImu();
  imuConfigured = calibrateImu();
  myIMU.enableRotationVector(10); //Send data every 10ms

  //Set the safety bool variables to their respected values to turn
  //on the safety features on start up to ensure the drone
  //is calibrated correctly and ready to be ran at maximum capacity
  haveValidPacket = false;
  failsafeActive = true;
  motorArmed = false;
  //initialize the variables lastControlUs and lastTelemetryMs
  lastControlUs = micros();
  lastTelemetryMs = millis();
  Serial.println("AmbatuDrone Teensy flight/logger ready.");
}

// ---------- Main loop ----------
void loop() {
  processSerialInput(Serial, usbRx, true);
  processSerialInput(Serial1, radioRx, false);
  pollRecordButton();
  //Store the time in microseconds and calculate how long each loop took
  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastControlUs;
  //Check if it took to loop ran slower than 
  //expected loop time of 4000microseconds
  if (elapsedUs >= CONTROL_PERIOD_US) {
    //set loopOverrun to true and call requestDisarm to disarm hardware of the drone
    //if elapsedUs is greater than MAX_CONTROL_GAP_US and motorAmred is true
    if (motorArmed && elapsedUs > MAX_CONTROL_GAP_US) {
      loopOverrun = true;
      requestDisarm("control_loop_overrun");
    }
    //Constrain the deltaSeconds functions which will be used
    //for the Kalman filter and MPU
    /*const float deltaSeconds =
        constrain(elapsedUs * 0.000001f, 0.001f, 0.020f);*/
    lastControlUs = nowUs;
    //If the IMU is healthy run the Kalman filter
    imuHealthy = imuConfigured && readImuSample(deltaSeconds, true);
    /*if (imuHealthy) {
      //Call the Kalman filter function twice for the roll and pitch orientation
      kalmanUpdate(data.kalmanAngleRoll, data.kalmanUncertaintyRoll,
                   data.rateRoll, data.angleRoll, deltaSeconds);
      kalmanUpdate(data.kalmanAnglePitch, data.kalmanUncertaintyPitch,
                   data.ratePitch, data.anglePitch, deltaSeconds);
    }*/
    //If the Teensy doesn't receive any packets of data and the last time a packet was
    //received was over 500ms then set failsafeActive to true
    failsafeActive =
        !haveValidPacket || (millis() - lastPacketMs > FAILSAFE_MS);
    //if failsafeActive is true lock the motors and stop all the motors since there
    //isn't sufficent data being sent to the teensy
    if (failsafeActive) {
      data.r1 = 0;
      throttleNotLow = false;
      requestDisarm("link_failsafe");
    //If there is sufficent data being sent to the teensy keep updating 
    //the motors based on ps4 controller inputs and MPU data.
    } else if (!loopOverrun) {
      updateMotors();
    //If neither then call requestDisarm
    } else {
      requestDisarm("control_loop_overrun");
    }
  }
  //if stopLoggingRequested is true and motorArmed is false stop the recording of the microphones
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
  //Call sendTelemetry function every 100ms for consistant data transfer
  if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = millis();
    sendTelemetry(Serial);
    sendTelemetry(Serial1);
  }
}