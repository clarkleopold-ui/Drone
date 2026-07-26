#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

// ---------- Serial bridge ----------
const int MAX_LINE_LENGTH = 180;
char rxBuffer[MAX_LINE_LENGTH];
int bufferIndex = 0;
bool rxOverflow = false;

// ---------- Servo and ESC outputs ----------
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

// ---------- Pin setup ----------
const int MOTOR1_SPEED = 10, MOTOR2_SPEED = 11, MOTOR3_SPEED = 12, MOTOR4_SPEED = 13;
const int SERVO1_PIN = 3, SERVO2_PIN = 4, SERVO3_PIN = 5, SERVO4_PIN = 6;

// ---------- ESC settings ----------
// 1000 us = off/minimum command, 2000 us = maximum command.
const int THROTTLE_MIN = 1000;
const int THROTTLE_MAX = 2000;
const int HOVER_THROTTLE = 1400;
const int CRUISE_SPEED = 1750;
const int ARM_THROTTLE_MAX_PERCENT = 5;
const int CONTROL_CORRECTION_PERCENT = 15;

// ---------- Servo settings (dormant; retained for later) ----------
const int SERVO_MIN_DEG_C = 60;
const int SERVO_MAX_DEG_C = 120;
const int SERVO_MIN_DEG_T = 70;
const int SERVO_MAX_DEG_T = 110;
const int SERVO_CENTER_DEG = 90;

// ---------- Timing and safety ----------
const unsigned long FAILSAFE_MS = 500;
const unsigned long CONTROL_PERIOD_US = 4000;  // 250 Hz
const unsigned long TELEMETRY_PERIOD_MS = 40;  // 25 Hz

unsigned long lastPacketMs = 0;
unsigned long lastControlUs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long currTime = 0;
unsigned long prevServoTime = 0;

bool haveValidPacket = false;
bool failsafeActive = true;
bool motorArmed = false;
bool throttleNotLow = false;
bool imuConfigured = false;
bool imuHealthy = false;

// ---------- Serial field helper ----------
// Read one integer while rejecting a missing or nonnumeric field.
bool readIntField(const char* src, const char* key, int& output) {
  char token[32];
  snprintf(token, sizeof(token), "\"%s\":", key);
  const char* start = strstr(src, token);
  if (start == NULL) {
    return false;
  }

  start += strlen(token);
  while (*start == ' ' || *start == '\t') {
    start++;
  }

  char* end = NULL;
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
// Struct holding controller, actuator, and MPU-6050 state.
struct RawData {
  int lx, ly, rx, ry;
  int cross, circle, square, triangle;
  int l1, r1;
  // ---------- Motor / 5010-360KV ----------
  int motorPwm[4]; //variables to store pwm values for each motor
  int throttlePercent; //variables to map the rpm of the motor to a percentage.
  int throttle; //variable to store values from the controller to real motor values in circle state
  int roll, pitch, yaw;
  int pitchOffSet, rollOffSet, yawOffSet; 
  int delta; //calculates how much each motor can increase or decrease in value based on each degree of movement (pitch, roll, yaw)
  int front_throttle, back_throttle; //variable to store values from the controller to real motor values in triangle state.
  //----------- servo -------- (will use later)
  int servoAngle[4];
  bool triangle_lock, square_lock, circle_lock;
  int servoPosition;
  bool free_move;
  //----------- drone states --------
  int drone_state, prevState; //Variables to store the different states of the drone
  //---------- gyroscope and accelerometer ----------
  char tmp_str[7]; // temporary array used to store int16 variable values 
  int MPU_ADDR; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
  int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
  int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
  float AccX, AccY, AccZ; // stores accelerometer raw data into g-forces
  float RateRoll, RatePitch, RateYaw; // stores raw angular velocity for each axis of movement in degrees/sec 
  float AngleRoll, AnglePitch, AngleYaw; // stores final values of the angle for pitch, roll, and yaw axis
  float YawRate;
  float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
  //---------- kalmanfilter variables ----------
  float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
  float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
};
RawData data;

void stopAllMotors() {
  data.throttle = THROTTLE_MIN;
  data.front_throttle = THROTTLE_MIN;
  data.back_throttle = THROTTLE_MIN;
  data.throttlePercent = 0;
  data.delta = 0;

  data.pitchOffSet = 0;
  data.rollOffSet = 0;
  data.yawOffSet = 0;

  for (int i = 0; i < 4; i++) {
    data.motorPwm[i] = THROTTLE_MIN;
  }

  motor1.writeMicroseconds(THROTTLE_MIN);
  motor2.writeMicroseconds(THROTTLE_MIN);
  motor3.writeMicroseconds(THROTTLE_MIN);
  motor4.writeMicroseconds(THROTTLE_MIN);
}

// ---------- Drone classes ----------
class ControllerClass {
  public:
    ControllerClass();
    int axisToServoAngle(int axisValue);
    bool parseControllerLine(const char* line);
    void displayValues_and_buttonPress() const;
    void DroneStates_input();
};
class MotorClass {
  public: 
    void state_circle_motor();
    void state_triangle_motor();
};
//Class responsible for manipulating the servos (will use later)
/*class ServoClass {
  public:
    void state_circle_servo();
    void state_triangle_servo();
};*/
class gyro_accelerometerClass {
  public:
    bool begin();
    bool calibrateGyro();
    bool readSample(float deltaSeconds, bool applyCalibration);
    void kalmanUpdate(
      float& state,
      float& uncertainty,
      float rate,
      float measurement,
      float deltaSeconds
    );

  private:
    bool writeRegister(uint8_t reg, uint8_t value);
};
ControllerClass controller;
MotorClass motor;
//ServoClass servo;
gyro_accelerometerClass gyro_accel;

ControllerClass::ControllerClass() {  //Constructor
  data.lx = data.ly = data.rx = data.ry = 0;
  data.cross = data.circle = data.square = data.triangle = 0;
  data.l1 = data.r1 = 0;
  data.throttlePercent = data.pitch = data.roll = data.yaw = 0;
  data.triangle_lock = data.square_lock = false;
  data.circle_lock = true;
  data.servoPosition = 0;
  data.pitchOffSet = data.rollOffSet = data.yawOffSet = 0;
  prevServoTime = 0;
  data.drone_state = 0;
  data.prevState = 0;
  data.free_move = true;
  data.MPU_ADDR = 0x68;
  data.AccX = data.AccY = data.AccZ = 0.0f;
  data.RateRoll = data.RatePitch = data.RateYaw = 0.0f;
  data.AngleRoll = data.AnglePitch = data.AngleYaw = 0.0f;
  data.YawRate = 0.0f;
  data.RateCalibrationRoll = 0.0f;
  data.RateCalibrationPitch = 0.0f;
  data.RateCalibrationYaw = 0.0f;
  data.KalmanAngleRoll = 0.0f;
  data.KalmanAnglePitch = 0.0f;
  data.KalmanUncertaintyAngleRoll = 4.0f;
  data.KalmanUncertaintyAnglePitch = 4.0f;
  data.throttle = THROTTLE_MIN;
  data.front_throttle = THROTTLE_MIN;
  data.back_throttle = THROTTLE_MIN;
  data.delta = 0;

  for (int i = 0; i < 4; i++) {
    data.motorPwm[i] = THROTTLE_MIN;
  }
}
//returns mapped values from the controller joystick to the python files then to values within the servo range so the servo can read and rotate to those angles
int ControllerClass::axisToServoAngle(int axisValue) {
  // axisValue is -100 to +100 from Python.
  //if statement to return values depending on which state the drone is in.
  if (data.drone_state == 3)
    return map(axisValue, -100, 100, SERVO_MIN_DEG_T, SERVO_MAX_DEG_T);
  else
    return map(axisValue, -100, 100, SERVO_MIN_DEG_C, SERVO_MAX_DEG_C);
}
// Accept only a complete controller packet. A malformed line must not refresh
// the failsafe timer.
bool ControllerClass::parseControllerLine(const char* line) {
  const size_t length = strlen(line);
  if (length < 2 || line[0] != '{' || line[length - 1] != '}') {
    return false;
  }

  int lx, ly, rx, ry;
  int cross, circle, square, triangle;
  int l1, r1;

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

  // Update the live state only after the complete packet has been validated.
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
/*displays all the values from the controller onto the drone app while 
also turing the RGB LED on to specific colors when specific buttons are pressed.*/
void ControllerClass::displayValues_and_buttonPress() const{
  // ---------- Debug output ----------
  Serial.print("ACK motor_pwm=");
  Serial.print(data.motorPwm[0]);
  Serial.print(" ");
  Serial.print(data.motorPwm[1]);
  Serial.print(" ");
  Serial.print(data.motorPwm[2]);
  Serial.print(" ");
  Serial.print(data.motorPwm[3]);
  /*Serial.print(" servo1=");
  Serial.print(data.servoAngle[0]);
  Serial.print(" servo2=");
  Serial.print(data.servoAngle[1]);
  Serial.print(" servo3=");
  Serial.print(data.servoAngle[2]);
  Serial.print(" servo4=");
  Serial.print(data.servoAngle[3]);*/
  Serial.print(" lx=");
  Serial.print(data.lx);
  Serial.print(" ly=");
  Serial.print(data.ly);
  Serial.print(" rx=");
  Serial.print(data.rx);
  Serial.print(" ry=");
  Serial.print(data.ry);
  Serial.print(" Roll [°]: ");
  Serial.print(data.KalmanAngleRoll);
  Serial.print(" Pitch [°]: ");
  Serial.print(data.KalmanAnglePitch);
  Serial.print(" Yaw [°]: ");
  Serial.print(data.AngleYaw);
  Serial.print(" Yaw Rate [°/s]: ");
  Serial.print(data.YawRate, 2);
  //checks if the cross button has been pressed and on the serial monitor " Kill" will be displayed
  if (data.cross) Serial.print(" KILL");
  //Prints " MOTOR_LOCKED" if r1 is not pressed
  if (!data.r1 || !motorArmed) Serial.print(" MOTOR_LOCKED");
  if (throttleNotLow) Serial.print(" THROTTLE_NOT_LOW");
  if (failsafeActive) Serial.print(" FAILSAFE");
  if (!imuHealthy) Serial.print(" IMU_ERROR");
  Serial.println();
}
// Circle mode is the only enabled motor mode in this safety baseline.
// Triangle-mode data and functions remain in the file for later development.
void ControllerClass::DroneStates_input() {
  data.prevState = data.drone_state;
  data.drone_state = 0;
  motor.state_circle_motor();
}

// Default four-motor control. R1 is a dead-man enable and X is kill.
void MotorClass::state_circle_motor() {
  throttleNotLow = false;

  if (data.cross || !data.r1) {
    motorArmed = false;
    stopAllMotors();
    return;
  }

  // Do not arm without valid attitude data.
  if (!imuHealthy) {
    motorArmed = false;
    stopAllMotors();
    return;
  }

  data.throttlePercent = constrain(data.ly, 0, 100);

  // Require a low throttle once after R1 is pressed or after any disarm.
  // This prevents an immediate jump to high power if R1 is pressed while
  // the stick is already raised.
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

  // The Python app sends upward stick motion as positive ly.
  data.throttle = map(
    data.throttlePercent,
    0, 100,
    THROTTLE_MIN, THROTTLE_MAX
  );

  // Control authority rises gradually from zero at minimum throttle.
  data.delta =
    ((long)(data.throttle - THROTTLE_MIN) * CONTROL_CORRECTION_PERCENT) / 100L;

  // Recalculate every loop so offsets return to zero with centered sticks.
  data.pitchOffSet = map(data.pitch, -100, 100, -data.delta, data.delta);
  data.rollOffSet = map(data.roll, -100, 100, -data.delta, data.delta);
  data.yawOffSet = map(data.yaw, -100, 100, -data.delta, data.delta);

  const long limitedPitchAngle =
    (long)constrain(data.KalmanAnglePitch, -80.0f, 80.0f);
  const long limitedRollAngle =
    (long)constrain(data.KalmanAngleRoll, -80.0f, 80.0f);

  const int pitchGyro =
    map(limitedPitchAngle, -80, 80, -data.delta, data.delta);
  const int rollGyro =
    map(limitedRollAngle, -80, 80, -data.delta, data.delta);

  // Preserve the original motor order and mixer signs.
  data.motorPwm[0] =
    data.throttle + pitchGyro + rollGyro
    - data.pitchOffSet + data.rollOffSet + data.yawOffSet;

  data.motorPwm[1] =
    data.throttle - pitchGyro + rollGyro
    + data.pitchOffSet + data.rollOffSet - data.yawOffSet;

  data.motorPwm[2] =
    data.throttle + pitchGyro - rollGyro
    - data.pitchOffSet - data.rollOffSet - data.yawOffSet;

  data.motorPwm[3] =
    data.throttle - pitchGyro - rollGyro
    + data.pitchOffSet - data.rollOffSet + data.yawOffSet;

  // With no manual yaw command, oppose measured yaw rate.
  if (data.lx == 0) {
    const long limitedYawRate =
      (long)constrain(data.YawRate, -125.0f, 125.0f);
    const int yawGyro =
      map(limitedYawRate, -125, 125, -data.delta, data.delta);

    data.motorPwm[0] += yawGyro;
    data.motorPwm[1] -= yawGyro;
    data.motorPwm[2] -= yawGyro;
    data.motorPwm[3] += yawGyro;
  }

  for (int i = 0; i < 4; i++) {
    data.motorPwm[i] =
      constrain(data.motorPwm[i], THROTTLE_MIN, THROTTLE_MAX);
  }

  motor1.writeMicroseconds(data.motorPwm[0]);
  motor2.writeMicroseconds(data.motorPwm[1]);
  motor3.writeMicroseconds(data.motorPwm[2]);
  motor4.writeMicroseconds(data.motorPwm[3]);
}
// Triangle mode is intentionally disabled until its motor/servo transition
// behavior is redesigned and bench-tested.
void MotorClass::state_triangle_motor() {
  motorArmed = false;
  stopAllMotors();
}
//This function corresponds with the default drone state allowing the drone to fly like a normal drone giving full access to all servos
//(Will be used later)
/*void ServoClass::state_circle_servo() {
  //Checks if the bool circle_lock is false (not locked) and if it is false, allow the servos to move back into the 90 degree position from of the circle state
  //for the drone. This also locks all servos from being controlled by the ps4 controller untill unlocked later in the code below
  if(!data.circle_lock) {
    data.servoPosition = 180;
    data.free_move = false;
  }
  //This if statement checks if the previous state was from the triangle state and checks if the servoPositions are not yet locked at 90
  if(data.prevState == 3 || (data.servoPosition >= 90)) {
    //If either conditions are met then the front two servos will slowly rotate to 90 degrees
    if(currTime - prevServoTime >= 40) {
      prevServoTime = millis();
      data.servoPosition--;
      data.servoAngle[0] = data.servoAngle[2] = data.servoPosition;
      servo1.write(data.servoPosition);
      servo3.write(data.servoPosition);
    }
  } else {
    data.free_move = true; //unlocks all servos
    data.triangle_lock = false;  //unlocks the triangle button to allowing for that button to be pressed again
  }
  //If the all servos are unlocked they will be able to recieve input from the controller
  if (data.free_move) {
    //Sends the mapped values from the left joystick and servo values into each individual servo variable.
    data.servoAngle[0] = data.servoAngle[1] = data.servoAngle[2] = data.servoAngle[3] = controller.axisToServoAngle(data.ry);
    servo1.write(data.servoAngle[0]);
    servo2.write(data.servoAngle[1]);
    servo3.write(data.servoAngle[2]);
    servo4.write(data.servoAngle[3]);
  }
  locks the triangle button to not allow the code to repeat the beginning stage for the triangle state which is the motion
  of moving the front two servo motors into fixed 180 degree positions.
  data.circle_lock = true;
}*/
//This function corresponds to Drone state 3 which is where the two front servos get locked 
//to 180 degrees while the two back servos are still able to freely rotate. (Similar to a vtol aircraft)
//(Will be used later)
/*void ServoClass::state_triangle_servo() {
  //Checks if the bool triangle_lock is false (not locked) and if it is false, gives permission to the front two servos to rotate 
  //to the 180 degree position. This also locks all servos from being controlled by the ps4 controller untill unlocked later 
  //in the code below.
  if (!data.triangle_lock) {
    data.servoPosition = controller.axisToServoAngle(data.ry);
    data.free_move = false;
  }
  //This if statement checks if the previous state was from the circle state and checks if the servoPositions are not yet locked at 180
  if(data.prevState == 0 || (data.servoPosition < 180)) {
    //If those conditions are true the two front servos will slowly rotate to the 180 degree position 
    if(currTime - prevServoTime >= 40) {
      prevServoTime = millis();
      data.servoPosition++;
      data.servoAngle[0] = data.servoAngle[2] = data.servoPosition;
      servo1.write(data.servoPosition);
      servo3.write(data.servoPosition);
    }
  } else {
    data.free_move = true; //unlocks the two back servos
    data.circle_lock = false; //unlocks the circle button to allowing for that button to be pressed again
  }
  //If the two back servos are unlocked they will be able to recieve input from the controller
  if (data.free_move) {
    //Sends the mapped values from the right joystick and servo values into each individual 
    //servo variable.
    data.servoAngle[1] = data.servoAngle[3] = controller.axisToServoAngle(data.ry);
    servo2.write(data.servoAngle[1]);
    servo4.write(data.servoAngle[3]);
  }
  locks the triangle button to not allow the code to repeat the beginning stage for the triangle state which is the motion
  of moving the front two servo motors into fixed 180 degree positions.
  data.triangle_lock = true;
}*/

// ---------- MPU-6050 ----------
bool gyro_accelerometerClass::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission((uint8_t)data.MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool gyro_accelerometerClass::begin() {
  // Wake the MPU-6050, configure the low-pass filter, then select
  // accelerometer +/-8 g and gyroscope +/-500 deg/s ranges.
  return
    writeRegister(0x6B, 0x00) &&
    writeRegister(0x1A, 0x05) &&
    writeRegister(0x1C, 0x10) &&
    writeRegister(0x1B, 0x08);
}

bool gyro_accelerometerClass::readSample(
  float deltaSeconds,
  bool applyCalibration
) {
  // Read accelerometer, temperature, and gyro registers in one burst.
  Wire.beginTransmission((uint8_t)data.MPU_ADDR);
  Wire.write((uint8_t)0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t received =
    Wire.requestFrom((uint8_t)data.MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (received != 14 || Wire.available() < 14) {
    return false;
  }

  data.accelerometer_x = ((int16_t)Wire.read() << 8) | Wire.read();
  data.accelerometer_y = ((int16_t)Wire.read() << 8) | Wire.read();
  data.accelerometer_z = ((int16_t)Wire.read() << 8) | Wire.read();

  // Temperature is not used yet.
  Wire.read();
  Wire.read();

  data.gyro_x = ((int16_t)Wire.read() << 8) | Wire.read();
  data.gyro_y = ((int16_t)Wire.read() << 8) | Wire.read();
  data.gyro_z = ((int16_t)Wire.read() << 8) | Wire.read();

  data.AccX = (float)data.accelerometer_x / 4096.0f;
  data.AccY = (float)data.accelerometer_y / 4096.0f;
  data.AccZ = (float)data.accelerometer_z / 4096.0f;

  data.RateRoll = (float)data.gyro_x / 65.5f;
  data.RatePitch = (float)data.gyro_y / 65.5f;
  data.RateYaw = (float)data.gyro_z / 65.5f;

  if (applyCalibration) {
    data.RateRoll -= data.RateCalibrationRoll;
    data.RatePitch -= data.RateCalibrationPitch;
    data.RateYaw -= data.RateCalibrationYaw;
  }

  data.AngleRoll = atan2(
    data.AccY,
    sqrt(data.AccX * data.AccX + data.AccZ * data.AccZ)
  ) * RAD_TO_DEG;

  data.AnglePitch = -atan2(
    data.AccX,
    sqrt(data.AccY * data.AccY + data.AccZ * data.AccZ)
  ) * RAD_TO_DEG;

  if (applyCalibration) {
    data.AngleYaw += data.RateYaw * deltaSeconds;
    data.YawRate = data.RateYaw;
  }

  return true;
}

bool gyro_accelerometerClass::calibrateGyro() {
  const int calibrationSamples = 2000;
  int validSamples = 0;
  float rollSum = 0.0f;
  float pitchSum = 0.0f;
  float yawSum = 0.0f;

  for (int i = 0; i < calibrationSamples; i++) {
    if (readSample(0.0f, false)) {
      rollSum += data.RateRoll;
      pitchSum += data.RatePitch;
      yawSum += data.RateYaw;
      validSamples++;
    }
    delay(1);
  }

  if (validSamples < calibrationSamples * 9 / 10) {
    return false;
  }

  data.RateCalibrationRoll = rollSum / validSamples;
  data.RateCalibrationPitch = pitchSum / validSamples;
  data.RateCalibrationYaw = yawSum / validSamples;
  data.RateRoll = data.RatePitch = data.RateYaw = 0.0f;
  data.YawRate = 0.0f;
  data.AngleYaw = 0.0f;
  return true;
}

// One-dimensional Kalman filter for roll and pitch.
void gyro_accelerometerClass::kalmanUpdate(
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
// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  // Servo setup is retained for later use. No runtime servo-control function
  // is called by this baseline.
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);

  motor1.attach(MOTOR1_SPEED);
  motor2.attach(MOTOR2_SPEED);
  motor3.attach(MOTOR3_SPEED);
  motor4.attach(MOTOR4_SPEED);

  servo1.write(SERVO_CENTER_DEG);
  servo2.write(SERVO_CENTER_DEG);
  servo3.write(SERVO_CENTER_DEG);
  servo4.write(SERVO_CENTER_DEG);

  // Normal ESC arming sends only the minimum pulse. Maximum-throttle ESC
  // calibration must be done with a separate, deliberate calibration sketch.
  stopAllMotors();

  Wire.begin();
  Wire.setClock(400000);

  imuConfigured = gyro_accel.begin();
  imuHealthy = imuConfigured && gyro_accel.calibrateGyro();

  stopAllMotors();
  delay(3000);

  haveValidPacket = false;
  failsafeActive = true;
  motorArmed = false;
  lastControlUs = micros();
  lastTelemetryMs = millis();

  Serial.println("Arduino bridge ready");
}

// ---------- Main loop ----------
void loop() {
  // Read all currently available bytes. Each completed line is parsed
  // immediately, even when another packet has already started arriving.
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();

    if (c == '\n') {
      if (!rxOverflow && bufferIndex > 0) {
        rxBuffer[bufferIndex] = '\0';
        controller.parseControllerLine(rxBuffer);
      }

      bufferIndex = 0;
      rxOverflow = false;
    } else if (c != '\r') {
      if (!rxOverflow) {
        if (bufferIndex < MAX_LINE_LENGTH - 1) {
          rxBuffer[bufferIndex++] = c;
        } else {
          // Discard the entire oversized line until its newline arrives.
          bufferIndex = 0;
          rxOverflow = true;
        }
      }
    }
  }

  const unsigned long nowUs = micros();
  const unsigned long elapsedUs = nowUs - lastControlUs;
  if (elapsedUs < CONTROL_PERIOD_US) {
    return;
  }

  // Use measured timing, with a cap to keep the estimator stable after a
  // debugger pause or other unusually long delay.
  const float deltaSeconds =
    constrain(elapsedUs * 0.000001f, 0.001f, 0.020f);
  lastControlUs = nowUs;
  currTime = millis();

  if (imuConfigured) {
    imuHealthy = gyro_accel.readSample(deltaSeconds, true);
  } else {
    imuHealthy = false;
  }

  if (imuHealthy) {
    gyro_accel.kalmanUpdate(
      data.KalmanAngleRoll,
      data.KalmanUncertaintyAngleRoll,
      data.RateRoll,
      data.AngleRoll,
      deltaSeconds
    );

    gyro_accel.kalmanUpdate(
      data.KalmanAnglePitch,
      data.KalmanUncertaintyAnglePitch,
      data.RatePitch,
      data.AnglePitch,
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
    controller.DroneStates_input();
  }

  // Rate-limit telemetry so the USB serial link and Tkinter app are not
  // flooded by the 250 Hz control loop.
  if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = millis();
    controller.displayValues_and_buttonPress();
  }
}
