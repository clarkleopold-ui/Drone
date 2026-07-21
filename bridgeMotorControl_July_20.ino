#include <Arduino.h>
#include <Servo.h>
#include "Wire.h" //This library allows you to communicate with the gyroscope and accelerometer
//array to store data from python files into the arduino
const int MAX_LINE_LENGTH = 180;
char rxBuffer[MAX_LINE_LENGTH];
int bufferIndex = 0;
//Declared servo and motor variable names
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
//---------- Pin setup ----------
const int MOTOR1_SPEED = 10, MOTOR2_SPEED = 11, MOTOR3_SPEED = 12, MOTOR4_SPEED = 13;
const int SERVO1_PIN = 3, SERVO2_PIN = 4, SERVO3_PIN = 5, SERVO4_PIN = 6;
//---------- Global Variables ----------
//ESC pulse range. ESCs use 1000 us = off, 2000 us = full throttle.
//const variables for ESC values
const int THROTTLE_MIN = 1000;
const int THROTTLE_MAX = 2000;
const int HOVER_THROTTLE = 1400;
const int CRUISE_SPEED = 1750;
//Servo angle limits
const int SERVO_MIN_DEG_C = 60;
const int SERVO_MAX_DEG_C = 120;
const int SERVO_MIN_DEG_T = 70;
const int SERVO_MAX_DEG_T = 110;
const int SERVO_CENTER_DEG = 90;
//Safety timeout: two global variable timers, checks if data is being sent and sets the drone to hover if no data is being sent
const unsigned long FAILSAFE_MS = 500;
unsigned long lastPacketMs = 0;
//Global variable for checking the internal clock in the arduino
long int currTime;
long int LoopTimer;
long int LimitTime;
long int prevServoTime;

//---------- Functions ----------
//Function to communicate to the python files
int readIntField(const char* src, const char *key, int fallback) {
  // Create the key token, e.g., "lx":
  char token[32];
  sprintf(token, "\"%s\":", key);
  // Search for the token in the raw buffer
  const char *start = strstr(src, token);
  if (start == NULL) return fallback; // Not found
  // Move the pointer past the token to get to the number
  start += strlen(token);
  // atoi converts the characters directly into an integer
  return atoi(start);
}
// (c) Michael Schoeffler 2017, http://www.mschoeffler.de for gyroscope and accelerometer starter code
// https://github.com/CarbonAeronautics/Part-XV-1DKalmanFilter/blob/main/ArduinoCode for kalman filter algorithm

//Struct to hold all variables that stores all raw data for hardware and software components like motors, servos, ps4 controller, etc
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
  float prevAngleYaw, YawDelta, YawRate; // variables to help calculate the derivative of the AngleYaw variable
  float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw; //
  //---------- kalmanfilter variables ----------
  float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
  float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
  float Kalman1DOutput[2]= {0,0};
};
RawData data;
//Drone classes
//class responsible for taking data from python files to read ps4 inputs 
class ControllerClass {
  public:
    ControllerClass(); //constructor
    int axisToServoAngle(int axisValue);
    void parseControllerLine(const char* line);
    void displayValues_and_buttonPress() const;
    void DroneStates_input();
};
//Class responsible for manipulating the motors
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
    char* convert_int16_to_str (int16_t i) { //converts the int16_t from the accelerometer and gyro variables to strings by storing them into a array called tmp_str and returning those values
      sprintf(data.tmp_str, "%6d", i);
      return data.tmp_str;
    } // this function is part of the gyro and accelerometer starter code
    //void start_MPU_data();
    void kalman_calc (float, float, float, float);
    void Gyro_Accle_calc();
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
  data.AngleYaw = 0; // set angleYaw to zero when starting to avoid any unbalances in the Yaw plane for the drone
  data.motorPwm[0] = data.motorPwm[1] = data.motorPwm[2] = data.motorPwm[3] = 0;
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
//This function is responsible for storing all the python read values from the controller to then store them into variables in here
//to allow for the arduino to read and understand inputs from the ps4 controller. Values for the joysticks are stored from -100 to 100.
void ControllerClass::parseControllerLine(const char* line) {
  data.lx = constrain(readIntField(line, "lx", data.lx), -100, 100);
  data.ly = constrain(readIntField(line, "ly", data.ly), -100, 100);
  data.rx = constrain(readIntField(line, "rx", data.rx), -100, 100);
  data.ry = constrain(readIntField(line, "ry", data.ry), -100, 100);
  data.cross = readIntField(line, "cross", data.cross);
  data.circle = readIntField(line, "circle", data.circle);
  data.square = readIntField(line, "square", data.square);
  data.triangle = readIntField(line, "triangle", data.triangle);
  data.l1 = readIntField(line, "l1", data.l1);
  data.r1 = readIntField(line, "r1", data.r1);
  lastPacketMs = millis();
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
  Serial.print(" Yaw Rate[°/s^6]: ");
  Serial.print(data.YawRate, 6);
  //checks if the cross button has been pressed and on the serial monitor " Kill" will be displayed
  if (data.cross) Serial.print(" KILL");
  //Prints " MOTOR_LOCKED" if r1 is not pressed
  if (!data.r1) Serial.print(" MOTOR_LOCKED");
  Serial.println();
}
//This function will activate the two states of the drone
void ControllerClass::DroneStates_input() {
  //Checks if the triangle or circle button on the ps4 controller is pressed
  //Also has capability of remembering the last drone state just is case if more states will be added
  if (data.triangle) {
    data.prevState = data.drone_state;
    data.drone_state = 3;
  } else if (data.circle) {
    data.prevState = data.drone_state;
    data.drone_state = 0;
  }
  if (data.drone_state == 3) {
    //state.state_triangle_servo();
    motor.state_triangle_motor();
  } else if (data.drone_state == 0) {
    //state.state_circle_servo();
    motor.state_circle_motor();
  }
  controller.displayValues_and_buttonPress();
}
//This function corresponds to the default drone state and updates variables corressponding to the motors from the controller 
void MotorClass::state_circle_motor() {
  //When the r1 button on the controller is held down the left joystick can be used to drive power into each motor.
  //This allows the drone to fly higher into the sky.
  //When r1 isn't held down the motors will not respond in any circumstance.
  //If the cross button is also pressed then the motors cannot rotate in any circumstance.
  //When the right joystick is moving in the x-axis the motors will not respond in any circumstance.
  if (data.cross || !data.r1) { 
    //all motors set to hover
    data.throttle = THROTTLE_MIN;
    data.throttlePercent = (THROTTLE_MIN/THROTTLE_MAX)*100;
    data.pitchOffSet = data.rollOffSet = data.yawOffSet = 0;
  } else if (data.free_move) {
    // map and constrain the ps4 left and right joystick movements to values of -100 to 100 for easier motor rpm manipulation
    data.throttlePercent = constrain(data.ly, -100, 100);
    data.pitch = constrain(data.ry, -100, 100);
    data.roll = constrain(data.rx, -100, 100);
    data.yaw = constrain(data.lx, -100, 100);
    //When moveing the left joystick in the y axis it will controll the throttle of all motors
    if (data.ly >= 0) {
      //maps the left joystick's y axis into a percent value then maps those percentages to PWM values for each motor through the ESC
      data.throttle = map(data.ly, -100, 0, THROTTLE_MIN, HOVER_THROTTLE);
    } else if (data.ly <= 0) {
      data.throttle = map(data.ly, 0, 100, HOVER_THROTTLE, THROTTLE_MAX);
    }
    data.delta = data.throttle * 0.15;
    //When moving the right joystick in the y-axis it will tilt the drone forward and back (pitch) 
    if (data.ry) data.pitchOffSet = map(data.pitch, -100, 100, -data.delta, data.delta);
    //When moving the right joystick in the x-axis it will tilt the drone left and right (roll)
    if (data.rx) data.rollOffSet = map(data.roll, -100, 100, -data.delta, data.delta);
    //This if statement controls the yawn of the drone, allowing the drone to rotate/spin in place.
    //This spins the drone CW and CCW
    if(data.lx) data.yawOffSet = map(data.yaw, -100, 100, -data.delta, data.delta);
  }
  //set a maximum and minimum limit for each directional input from the ps4 controller 
  data.throttle = constrain(data.throttle, THROTTLE_MIN, THROTTLE_MAX);
  data.roll = constrain(data.roll, THROTTLE_MIN, THROTTLE_MAX);
  data.pitch = constrain(data.pitch, THROTTLE_MIN, THROTTLE_MAX);
  data.yaw = constrain(data.yaw, THROTTLE_MIN, THROTTLE_MAX);
  //Calculate the pitch, roll, and yaw, pwm values based on the gyroscope and accelerometer
  int pitch_gyro_accel = map(data.KalmanAnglePitch, -80, 80, -data.delta, data.delta);
  int roll_gyro_accel = map(data.KalmanAngleRoll, -80, 80, -data.delta, data.delta);

  data.motorPwm[0] = data.throttle + pitch_gyro_accel + roll_gyro_accel - data.pitchOffSet + data.rollOffSet + data.yawOffSet;
  data.motorPwm[1] = data.throttle - pitch_gyro_accel + roll_gyro_accel + data.pitchOffSet + data.rollOffSet - data.yawOffSet;
  data.motorPwm[2] = data.throttle + pitch_gyro_accel - roll_gyro_accel - data.pitchOffSet - data.rollOffSet - data.yawOffSet;
  data.motorPwm[3] = data.throttle - pitch_gyro_accel - roll_gyro_accel + data.pitchOffSet - data.rollOffSet + data.yawOffSet;

  if(data.lx == 0) {
    //if the rate of change value from data.YawRate is - it spins CW, if + CCW
    int yaw_gyro_accel = map(data.YawRate, -125, 125, -data.delta, data.delta);
    data.motorPwm[0] +=  yaw_gyro_accel;
    data.motorPwm[1] -=  yaw_gyro_accel;
    data.motorPwm[2] -=  yaw_gyro_accel;
    data.motorPwm[3] +=  yaw_gyro_accel;
  }
  for (int y=0; y<4; y++)
    data.motorPwm[y] = constrain(data.motorPwm[y], THROTTLE_MIN, THROTTLE_MAX);

  motor1.writeMicroseconds(data.motorPwm[0]);
  motor2.writeMicroseconds(data.motorPwm[1]);
  motor3.writeMicroseconds(data.motorPwm[2]);
  motor4.writeMicroseconds(data.motorPwm[3]);
}
//This function corresponds with the a drone state 3 where the 2 front motors get set to a
//crusing speed or can increase to a higher speed controlled by the left joystick in the y-axis,
//and the back 2 motors will get adjusted based on inputs from the right joystick in the y-axis
void MotorClass::state_triangle_motor() {
  //when cross is pressed or r1 isn't pressed have the front 2 motors rotate at crusing speed
  //and have the back 2 motors rotate at hovering speeds
  data.delta = data.back_throttle * 0.1;
  if ((data.cross || !data.r1) && data.free_move) {
    data.throttlePercent = (HOVER_THROTTLE/THROTTLE_MAX)*100;
    data.front_throttle = CRUISE_SPEED;
    data.back_throttle = HOVER_THROTTLE;
  } else {
    //When the 2 front motors are free to move and the left joystick in the y-axis greater than 0
    //increase the motor rpm starting from the cruise speed rpm to the max rpm
    if ((data.ly >= 0) && data.free_move) {
      data.throttlePercent = constrain(data.ly, 0, 100);
      data.front_throttle = map(data.throttlePercent, 0, 100, CRUISE_SPEED, THROTTLE_MAX);
    //Changes the rmp of the back 2 motors by a +- 10%
    } else if (data.ry && data.free_move) {
      data.throttlePercent = constrain(data.ry, -100, 100);
      data.back_throttle = map(data.throttlePercent, -100, 100, -data.delta, data.delta);
    }
  }
  //Add all values from controller inputs, gyroscope and accelerometer values into each motor
  data.motorPwm[0] = data.front_throttle;
  data.motorPwm[1] = data.back_throttle;
  data.motorPwm[2] = data.front_throttle;
  data.motorPwm[3] = data.back_throttle;
  //Set limits to each motor's PWM value to not go under or exceed the range of 1000-2000
  for (int y=0; y<4; y++)
    data.motorPwm[y] = constrain(data.motorPwm[y], THROTTLE_MIN, THROTTLE_MAX);
  //Send output signal to each motor
  motor1.writeMicroseconds(data.motorPwm[0]);
  motor2.writeMicroseconds(data.motorPwm[1]);
  motor3.writeMicroseconds(data.motorPwm[2]);
  motor4.writeMicroseconds(data.motorPwm[3]);
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

//Function to store the data from the gyroscope and accelerometer and
//converts that raw data to readable data in units like degrees/sec.
void gyro_accelerometerClass::Gyro_Accle_calc() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  data.accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  data.accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  data.accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)

  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);

  data.gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  data.gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  data.gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  //converts raw gyroscope data into degrees/sec
  data.RateRoll=(float)data.gyro_x/65.5;
  data.RatePitch=(float)data.gyro_y/65.5;
  data.RateYaw=(float)data.gyro_z/65.5;
  //converts raw accelerometer data into g forces
  data.AccX=(float)data.accelerometer_x/4096;
  data.AccY=(float)data.accelerometer_y/4096;
  data.AccZ=(float)data.accelerometer_z/4096;
  //Using trigometry to calculate the angles for the roll and pitch movements
  data.AngleRoll=atan(data.AccY/sqrt(data.AccX*data.AccX+data.AccZ*data.AccZ))*1/(3.142/180);
  data.AnglePitch=-atan(data.AccX/sqrt(data.AccY*data.AccY+data.AccZ*data.AccZ))*1/(3.142/180);

  data.RateRoll-=data.RateCalibrationRoll;
  data.RatePitch-=data.RateCalibrationPitch;
  data.RateYaw-=data.RateCalibrationYaw;
  //Calculate Yaw by integrating the gyroscope rate
  data.AngleYaw += data.RateYaw * 0.004;
  //Calculates the derivate of the AngleYaw variable.
  //If YawRate is -, drone spins CCW and if it's +, drone spins CW
  float delta_time = (static_cast<float>(currTime) - static_cast<float>(LimitTime)) *1000.0;
  if(delta_time > 0) {
    data.YawDelta = data.AngleYaw - data.prevAngleYaw;
    data.YawRate = (data.YawDelta / delta_time) * 1000000; //mutiply by 1 million to get numbers larger than 0
    data.prevAngleYaw = data.AngleYaw;
    LimitTime = currTime;
  }
}
//This function runs the kalman filter comparing the data from the gyroscope and accelerometer.
//Kalman gain is the vairbale to check which piece of data to trust more at that moment
void gyro_accelerometerClass::kalman_calc(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  KalmanState = constrain(KalmanState, -60, 60);
  data.Kalman1DOutput[0]=KalmanState;
  data.Kalman1DOutput[1]=KalmanUncertainty;
}
//Setting up all input pins, output pins, and hardware
void setup() {
  Serial.begin(115200);
  //arduinoSerial.begin(115200);
  //Servo setup
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  //Motor setup
  motor1.attach(MOTOR1_SPEED);
  motor2.attach(MOTOR2_SPEED);
  motor3.attach(MOTOR3_SPEED);
  motor4.attach(MOTOR4_SPEED);
  servo1.write(SERVO_CENTER_DEG);
  servo2.write(SERVO_CENTER_DEG);
  servo3.write(SERVO_CENTER_DEG);
  servo4.write(SERVO_CENTER_DEG);
  //Gyro and accelerometer board setup
  Wire.setClock(400000);
  Wire.begin();
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (int RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_accel.Gyro_Accle_calc();
    data.RateCalibrationRoll += data.RateRoll;
    data.RateCalibrationPitch += data.RatePitch;
    data.RateCalibrationYaw += data.RateYaw;
  }
  data.RateCalibrationRoll/=2000;
  data.RateCalibrationPitch/=2000;
  data.RateCalibrationYaw/=2000;
  Serial.println("Arduino bridge ready: ");
  //Set the min throttle speed to each motor
  motor1.writeMicroseconds(THROTTLE_MAX);
  motor2.writeMicroseconds(THROTTLE_MAX);
  motor3.writeMicroseconds(THROTTLE_MAX);
  motor4.writeMicroseconds(THROTTLE_MAX);
  //Using small delay to give the ESC time to see maximum throttle on startup.
  delay(5000);
  motor1.writeMicroseconds(THROTTLE_MIN);
  motor2.writeMicroseconds(THROTTLE_MIN);
  motor3.writeMicroseconds(THROTTLE_MIN);
  motor4.writeMicroseconds(THROTTLE_MIN);
  //Using small delay to give the ESC time to see minimum throttle on startup.
  delay(3000);
  lastPacketMs = millis();
}

void loop() {
  //checks the interal clock in the arduino
  currTime = millis();
  //Checks to see if the data from the python file are being sent to the arduino (code for the bridge between pythong files and arduino)
  while (Serial.available() > 0) {
    char c = (char)Serial.read(); //variable to store data from python files
    //checks if there is incoming data from the python files
    if (c == '\n') {
      //checks to see if rxLine has any data for the arduino to use to then call the rest of the class in this program
      if (bufferIndex > 0) {
        rxBuffer[bufferIndex] = '\0';
        //call all functions that manipulate drone outputs or display debugging information
        if (Serial.available() == 0) {
          controller.parseControllerLine(rxBuffer);
        }
        bufferIndex = 0;
      }
      //
    } else if (c != '\r') {
      // Prevent runaway String growth if serial data gets corrupted.
      if (bufferIndex < MAX_LINE_LENGTH - 1) {
        rxBuffer[bufferIndex++] = c; // Add character to array
      } else {
        bufferIndex = 0; // Prevent overflow by clearing if too long
      }
    }
  }
  //Calls the function that converts the raw data from the gyroscope and accelerometer into degrees and g force
  gyro_accel.Gyro_Accle_calc();
  //Apply Kalman filter for Roll
  gyro_accel.kalman_calc(data.KalmanAngleRoll, data.KalmanUncertaintyAngleRoll, data.RateRoll, data.AngleRoll);
  data.KalmanAngleRoll = data.Kalman1DOutput[0]; 
  data.KalmanUncertaintyAngleRoll= data.Kalman1DOutput[1];
  //Kalman filter for Pitch 
  gyro_accel.kalman_calc(data.KalmanAnglePitch, data.KalmanUncertaintyAnglePitch, data.RatePitch, data.AnglePitch);
  data.KalmanAnglePitch = data.Kalman1DOutput[0]; 
  data.KalmanUncertaintyAnglePitch = data.Kalman1DOutput[1];
  controller.DroneStates_input();
  // Failsafe: if computer/controller bridge to arduino disconnects, set the motors to hover and turn on RGB LED to an orange yellow color.
  if (millis() - lastPacketMs > FAILSAFE_MS) {
    motor1.writeMicroseconds(HOVER_THROTTLE);
    motor2.writeMicroseconds(HOVER_THROTTLE);
    motor3.writeMicroseconds(HOVER_THROTTLE);
    motor4.writeMicroseconds(HOVER_THROTTLE);
  }
}