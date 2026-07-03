#include <Arduino.h>
#include <Servo.h>
#include "Wire.h" // This library allows you to communicate with the gyroscope, accelorometer, and altimeter devices.
//ps4 controller used to control outputs
String rxLine;
//Declared servo variable names
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
const int SERVO1_PIN = 3;
const int SERVO2_PIN = 4;
const int SERVO3_PIN = 5;
const int SERVO4_PIN = 6;
//---------- Global Variables ----------
//ESC pulse range. Many ESCs use 1000 us = off, 2000 us = full throttle.
//Calibrate your ESC separately if needed.
const int THROTTLE_MIN = 1000;
const int THROTTLE_MAX = 2000;
const int HOVER_THROTTLE = 1400;
const int CRUISE_SPEED = 1600;
//Servo angle limits. Narrow these if your linkage binds mechanically.
const int SERVO_MIN_DEG = 60;
const int SERVO_MAX_DEG = 120;
const int SERVO_CENTER_DEG = 90;
//Safety timeout: if Python stops sending packets, set the motors to hover.
const unsigned long FAILSAFE_MS = 500;
unsigned long lastPacketMs = 0;
//Global variable for checking the internal clock in the arduino
static long int currTime;
static long int LoopTimer; //timer to keep track of how often the code runs to ensure it runs every 4ms
//---------- Functions ----------
//Function to communicate to the python files
int readIntField(const String &src, const char *key, int fallback) {
  String token = String("\"") + key + "\":";
  int start = src.indexOf(token);
  if (start < 0) return fallback;

  start += token.length();
  int end = start;

  if (end < src.length() && src.charAt(end) == '-') end++;
  while (end < src.length() && isDigit(src.charAt(end))) {
    end++;
  }

  if (end <= start) return fallback;
  return src.substring(start, end).toInt();
}
// (c) Michael Schoeffler 2017, http://www.mschoeffler.de for gyroscope and accelerometer starter code
// https://github.com/CarbonAeronautics/Part-XV-1DKalmanFilter/blob/main/ArduinoCode for kalman filter algorithm
//Drone class
class DroneState {
  private:
    //---------- controller ----------
    int lx, ly, rx, ry;
    int cross, circle, square, triangle;
    int l1, r1;
    // ---------- Motor / 5010-360KV ----------
    int motorPwm[4]; //variables to store pwm values for each motor
    int throttlePercent; //variables to map the rpm of the motor to a percentage.
    int throttle; //variable to store values from the controller to real motor values in circle state
    int pitchOffSet, rollOffSet, yawOffSet; 
    int delta; //calculates how much each motor can increase or decrease in value based on each degree of movement (pitch, roll, yaw)
    int front_throttle, back_throttle; //variable to store values from the controller to real motor values in triangle state.
    //----------- servo --------
    int servoAngle[4];
    bool triangle_lock, square_lock, circle_lock;
    int servoPosition;
    long int prevServoTime;
    bool free_move;
    //----------- drone states --------
    int drone_state, prevState; //Variables to store the different states of the drone
    //---------- gyroscope and accelerometer ----------
    char tmp_str[7]; // temporary variable used in convert function
    int MPU_ADDR; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
    int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
    int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
    float AccX, AccY, AccZ; // stores accelerometer raw data into g-forces
  public:
    DroneState(); //constructor
    int get_MPU_ADDR() { return MPU_ADDR; } //gets the address of the MPU-6050 which is the chip storing the
    char* convert_int16_to_str (int16_t i) { //converts the int16_t to strings by storing them into a array called tmp_str and returning those values
      sprintf(tmp_str, "%6d", i);
      return tmp_str;
    } // this function is part of the gyro and accelerometer starter code
    int axisToServoAngle(int axisValue);
    void parseControllerLine(const String &line);
    void state_circle_motor();
    void state_circle_servo();
    void state_triangle_motor();
    void state_triangle_servo();
    void displayValues_and_buttonPress() const;
    void DroneStates_input();
    //---------- gyroscope and accelerometer function----------
    void start_MPU_data();
    void kalman_calc (float, float, float, float);
    void Gyro_Accle_calc();
    float RateRoll, RatePitch, RateYaw; // stores raw angular velocity for each axis of movement in degrees/sec 
    float AngleRoll, AnglePitch, AngleYaw; // stores final values of the angle for pitch, roll, and yaw axis
    float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw; // 
    int RateCalibrationNumber; //
    float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
    float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
    float Kalman1DOutput[2]= {0,0};
};
DroneState state;
DroneState::DroneState() {  //Constructor
  lx = 0;
  ly = 0;
  rx = 0;
  ry = 0;
  cross = circle = square = triangle = 0;
  l1 = 0;
  r1 = 0;
  triangle_lock = square_lock = false;
  circle_lock = true;
  servoPosition = 0;
  pitchOffSet = rollOffSet = yawOffSet = 0;
  prevServoTime = 0;
  drone_state = 0;
  prevState = 0;
  free_move = true;
  MPU_ADDR = 0x68;
  AngleYaw = 0;
}
//returns mapped values from the controller joystick to the python files then to values within the servo range so the servo can read and rotate to those angles
int DroneState::axisToServoAngle(int axisValue) {
  // axisValue is -100 to +100 from Python.
  //if statement to return values depending on which state the drone is in.
  if (drone_state == 3)
    return map(axisValue, -100, 100, 70, 110);
  else
    return map(axisValue, -100, 100, SERVO_MIN_DEG, SERVO_MAX_DEG);
}
//This function is responsible for storing all the python read values from the controller to then store them into variables in here
//to allow for the arduino to read and understand inputs from the ps4 controller. Values are stored from -100 to 100.
void DroneState::parseControllerLine(const String &line) {
  lx = constrain(readIntField(line, "lx", lx), -100, 100);
  ly = constrain(readIntField(line, "ly", ly), -100, 100);
  rx = constrain(readIntField(line, "rx", rx), -100, 100);
  ry = constrain(readIntField(line, "ry", ry), -100, 100);
  cross = readIntField(line, "cross", cross);
  circle = readIntField(line, "circle", circle);
  square = readIntField(line, "square", square);
  triangle = readIntField(line, "triangle", triangle);
  l1 = readIntField(line, "l1", l1);
  r1 = readIntField(line, "r1", r1);
  lastPacketMs = millis();
}
//This function corresponds to the default drone state and updates variables corressponding to the motors from the controller 
void DroneState::state_circle_motor() {
  //When the r1 button on the controller is held down the left joystick can be used to drive power into each motor.
  //This allows the drone to fly higher into the sky.
  //When r1 isn't held down the motors will not respond in any circumstance.
  //If the cross button is also pressed then the motors cannot rotate in any circumstance.
  //When the right joystick is moving in the x-axis the motors will not respond in any circumstance.
  if (cross || !r1) { 
    //all motors set to hover
    throttle = HOVER_THROTTLE;
    throttlePercent = (HOVER_THROTTLE/THROTTLE_MAX)*100;;
  } else {
    //When moveing the left joystick in the y axis it will controll the throttle of all motors
    if (ly && free_move) {
      //maps the left joystick's y axis into a percent value then maps those percentages to PWM values for each motor through the ESC
      throttlePercent = constrain(ly, -100, 100);
      throttle = map(throttlePercent, -100, 100, THROTTLE_MIN, THROTTLE_MAX);
    }
  }
  delta = throttle * 0.15;
  //When moving the right joystick in the y-axis it will tilt the drone forward and back (pitch) 
  if (ry && free_move) {
    throttlePercent = constrain(ry, -100, 100);
    pitchOffSet = map(throttlePercent, -100, 100, -delta, delta);
  }
  //When moving the right joystick in the x-axis it will tilt the drone left and right (roll)
  if (rx && free_move) {
    throttlePercent = constrain(rx, -100, 100);
    rollOffSet = map(throttlePercent, -100, 100, -delta, delta);
  }
  //This if statement controls the yawn of the drone, allowing the drone to rotate/spin in place.
  //This spins the drone CW and CCW
  if(lx && free_move) {
    throttlePercent = constrain(lx, -100, 100);
    yawOffSet = map(throttlePercent, -100, 100, -delta, delta);
  }
  throttle = constrain(throttle, THROTTLE_MIN, THROTTLE_MAX);
  //Calculate the pitch, roll, and yaw, pwm values based on the gyroscope and accelerometer
  static int pitch = map(KalmanAnglePitch, -60, 60, -delta, delta);
  static int roll = map(KalmanAngleRoll, -60, 60, -delta, delta);
  /*motorPwm[0] = throttle + pitch + roll - pitchOffSet + rollOffSet + yawOffSet;
  motorPwm[1] = throttle - pitch + roll + pitchOffSet + rollOffSet - yawOffSet;
  motorPwm[2] = throttle + pitch - roll - pitchOffSet - rollOffSet - yawOffSet;
  motorPwm[3] = throttle - pitch - roll + pitchOffSet - rollOffSet + yawOffSet;*/
  motorPwm[0] = throttle - pitchOffSet + rollOffSet + yawOffSet;
  motorPwm[1] = throttle + pitchOffSet + rollOffSet - yawOffSet;
  motorPwm[2] = throttle - pitchOffSet - rollOffSet - yawOffSet;
  motorPwm[3] = throttle + pitchOffSet - rollOffSet + yawOffSet;
  for (int y=0; y<4; y++)
    motorPwm[y] = constrain(motorPwm[y], THROTTLE_MIN, THROTTLE_MAX);

  motor1.writeMicroseconds(motorPwm[0]);
  motor2.writeMicroseconds(motorPwm[1]);
  motor3.writeMicroseconds(motorPwm[2]);
  motor4.writeMicroseconds(motorPwm[3]);
}
//This function corresponds with the default drone state allowing the drone to fly like a normal drone
//giving full access to all servos
void DroneState::state_circle_servo() {
  //Checks if the bool circle_lock is false (not locked) and if it is false, allow the servos to move back into the 90 degree position from of the circle state
  //for the drone. This also locks all servos from being controlled by the ps4 controller untill unlocked later in the code below
  if(!circle_lock) {
    servoPosition = 180;
    free_move = false;
  }
  //This if statement checks if the previous state was from the triangle state and checks if the servoPositions are not yet locked at 90
  if(prevState == 3 || (servoPosition >= 90)) {
    //If either conditions are met then the front two servos will slowly rotate to 90 degrees
    if(currTime - prevServoTime >= 40) {
      prevServoTime = millis();
      servoPosition--;
      servoAngle[0] = servoAngle[2] = servoPosition;
      servo1.write(servoPosition);
      servo3.write(servoPosition);
    }
  } else {
    free_move = true; //unlocks all servos
    triangle_lock = false;  //unlocks the triangle button to allowing for that button to be pressed again
  }
  //If the all servos are unlocked they will be able to recieve input from the controller
  if (free_move) {
    //Sends the mapped values from the left joystick and servo values into each individual servo variable.
    servoAngle[0] = servoAngle[1] = servoAngle[2] = servoAngle[3] = state.axisToServoAngle(ry);
    servo1.write(servoAngle[0]);
    servo2.write(servoAngle[1]);
    servo3.write(servoAngle[2]);
    servo4.write(servoAngle[3]);
  }
  /*locks the triangle button to not allow the code to repeat the beginning stage for the triangle state which is the motion
  of moving the front two servo motors into fixed 180 degree positions.*/
  circle_lock = true;
}
//This function corresponds with the a drone state 3 where the 2 front motors get set to a
//crusing speed or can increase to a higher speed controlled by the left joystick in the y-axis,
//and the back 2 motors will get adjusted based on inputs from the right joystick in the y-axis
void DroneState::state_triangle_motor() {
  //when cross is pressed or r1 isn't pressed have the front 2 motors rotate at crusing speed
  //and have the back 2 motors rotate at hovering speeds
  delta = back_throttle * 0.1;
  if ((cross || !r1) && free_move) {
    throttlePercent = (HOVER_THROTTLE/THROTTLE_MAX)*100;
    front_throttle = CRUISE_SPEED;
    back_throttle = HOVER_THROTTLE - 100;
  } else {
    //When the 2 front motors are free to move and the left joystick in the y-axis greater than 0
    //increase the motor rpm starting from the cruise speed rpm to the max rpm
    if ((ly >= 0) && free_move) {
      throttlePercent = constrain(ly, 0, 100);
      front_throttle = map(throttlePercent, 0, 100, CRUISE_SPEED, THROTTLE_MAX);
    //Changes the rmp of the back 2 motors by a +- 10%
    } else if (ry && free_move) {
      throttlePercent = constrain(ry, -100, 100);
      back_throttle = map(throttlePercent, -100, 100, -delta, delta);
    }
  }
  //Add all values from controller inputs, gyroscope and accelerometer values into each motor
  motorPwm[0] = front_throttle;
  motorPwm[1] = back_throttle;
  motorPwm[2] = front_throttle;
  motorPwm[3] = back_throttle;
  //Set limits to each motor's PWM value to not go under or exceed the range of 1000-2000
  for (int y=0; y<4; y++)
    motorPwm[y] = constrain(motorPwm[y], THROTTLE_MIN, THROTTLE_MAX);
  //Send output signal to each motor
  motor1.writeMicroseconds(motorPwm[0]);
  motor2.writeMicroseconds(motorPwm[1]);
  motor3.writeMicroseconds(motorPwm[2]);
  motor4.writeMicroseconds(motorPwm[3]);
}
//This function corresponds to Drone state 3 which is where the two front servos get locked 
//to 180 degrees while the two back servos are still able to freely rotate.
//(Similar to a vtol aircraft)
void DroneState::state_triangle_servo() {
  //Checks if the bool triangle_lock is false (not locked) and if it is false, gives permission to the front two servos to rotate 
  //to the 180 degree position. This also locks all servos from being controlled by the ps4 controller untill unlocked later 
  //in the code below.
  if (!triangle_lock) {
    servoPosition = state.axisToServoAngle(ry);
    free_move = false;
  }
  //This if statement checks if the previous state was from the circle state and checks if the servoPositions are not yet locked at 180
  if(prevState == 0 || (servoPosition < 180)) {
    //If those conditions are true the two front servos will slowly rotate to the 180 degree position 
    if(currTime - prevServoTime >= 40) {
      prevServoTime = millis();
      servoPosition++;
      servoAngle[0] = servoAngle[2] = servoPosition;
      servo1.write(servoPosition);
      servo3.write(servoPosition);
    }
  } else {
    free_move = true; //unlocks the two back servos
    circle_lock = false; //unlocks the circle button to allowing for that button to be pressed again
  }
  //If the two back servos are unlocked they will be able to recieve input from the controller
  if (free_move) {
    //Sends the mapped values from the right joystick and servo values into each individual 
    //servo variable.
    servoAngle[1] = servoAngle[3] = state.axisToServoAngle(ry);
    servo2.write(servoAngle[1]);
    servo4.write(servoAngle[3]);
  }
  /*locks the triangle button to not allow the code to repeat the beginning stage for the triangle state which is the motion
  of moving the front two servo motors into fixed 180 degree positions.*/
  triangle_lock = true;
}
/*displays all the values from the controller onto the drone app while 
also turing the RGB LED on to specific colors when specific buttons are pressed.*/
void DroneState::displayValues_and_buttonPress() const{
  // ---------- Debug output ----------
  //if (millis() - LoopTimer >= 50) {}
    //LoopTimer= millis();
  Serial.print("ACK motor_pwm=");
  Serial.print(motorPwm[0]);
  Serial.print(" ");
  Serial.print(motorPwm[1]);
  Serial.print(" ");
  Serial.print(motorPwm[2]);
  Serial.print(" ");
  Serial.print(motorPwm[3]);
  Serial.print(" servo1=");
  Serial.print(servoAngle[0]);
  Serial.print(" servo2=");
  Serial.print(servoAngle[1]);
  Serial.print(" servo3=");
  Serial.print(servoAngle[2]);
  Serial.print(" servo4=");
  Serial.print(servoAngle[3]);
  Serial.print(" lx=");
  Serial.print(lx);
  Serial.print(" ly=");
  Serial.print(ly);
  Serial.print(" rx=");
  Serial.print(rx);
  Serial.print(" ry=");
  Serial.print(ry);
  Serial.print(" Roll [°]: ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" Pitch [°]: ");
  Serial.print(KalmanAnglePitch);
  Serial.print(" Yaw [°]: ");
  Serial.print(AngleYaw);
  //checks if the cross button has been pressed and on the serial monitor " Kill" will be displayed
  if (cross) Serial.print(" KILL");
  //Prints " MOTOR_LOCKED" if r1 is not pressed
  if (!r1) Serial.print(" MOTOR_LOCKED");
  Serial.println();
}
//Function to store the data from the gyroscope and accelerometer and
//converts that raw data to readable data in units like degrees/sec.
void DroneState::Gyro_Accle_calc() {
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
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  //converts raw gyroscope data into degrees/sec
  RateRoll=(float)gyro_x/65.5;
  RatePitch=(float)gyro_y/65.5;
  RateYaw=(float)gyro_z/65.5;
  //converts raw accelerometer data into g forces
  AccX=(float)accelerometer_x/4096;
  AccY=(float)accelerometer_y/4096;
  AccZ=(float)accelerometer_z/4096;
  //Using trigometry to calculate the angles for the roll and pitch movements
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);

  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  // Calculate Yaw by integrating the gyroscope rate over the 4ms loop time
  AngleYaw += RateYaw * 0.004;
}
//This function will activate the two states of the drone
void DroneState::DroneStates_input() {
  //Checks if the triangle or circle button on the ps4 controller is pressed
  //Also has capability of remembering the last drone state just is case if more states will be added
  if (triangle) {
    prevState = drone_state;
    drone_state = 3;
  } else if (circle) {
    prevState = drone_state;
    drone_state = 0;
  }
  if (drone_state == 3) {
    state.state_triangle_servo();
    state.state_triangle_motor();
  } else if (drone_state == 0) {
    state.state_circle_servo();
    state.state_circle_motor();
  }
}
//This function runs the kalman filter comparing the data from the gyroscope and accelerometer.
//Kalman gain is the vairbale to check which piece of data to trust more at that moment
void DroneState::kalman_calc(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  KalmanState = constrain(KalmanState, -60, 60);
  Kalman1DOutput[0]=KalmanState;
  Kalman1DOutput[1]=KalmanUncertainty;
}
//Setting up all input pins, output pins, and hardware
void setup() {
  Serial.begin(115200);
  rxLine.reserve(180); //reserves 180 bytes of memory for this string
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
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (state.RateCalibrationNumber=0; state.RateCalibrationNumber<2000; state.RateCalibrationNumber ++) {
    state.Gyro_Accle_calc();
    state.RateCalibrationRoll += state.RateRoll;
    state.RateCalibrationPitch += state.RatePitch;
    state.RateCalibrationYaw += state.RateYaw;
  }
  state.RateCalibrationRoll/=2000;
  state.RateCalibrationPitch/=2000;
  state.RateCalibrationYaw/=2000;
  int LoopTimer=micros();
  Serial.println("Arduino bridge ready: ");
  //Set the min throttle speed to each motor
  motor1.writeMicroseconds(THROTTLE_MIN);
  motor1.writeMicroseconds(THROTTLE_MIN);
  motor1.writeMicroseconds(THROTTLE_MIN);
  motor1.writeMicroseconds(THROTTLE_MIN);
  //Using small delay to give the ESC time to see minimum throttle on startup.
  delay(3000);
  lastPacketMs = millis();
}

void loop() {
  //checks the interal clock in the arduino
  currTime = millis();

  state.Gyro_Accle_calc();
  //Apply Kalman filter for Roll
  state.kalman_calc(state.KalmanAngleRoll, state.KalmanUncertaintyAngleRoll, state.RateRoll, state.AngleRoll);
  state.KalmanAngleRoll = state.Kalman1DOutput[0]; 
  state.KalmanUncertaintyAngleRoll= state.Kalman1DOutput[1];
  //Kalman filter for Pitch 
  state.kalman_calc(state.KalmanAnglePitch, state.KalmanUncertaintyAnglePitch, state.RatePitch, state.AnglePitch);
  state.KalmanAnglePitch = state.Kalman1DOutput[0]; 
  state.KalmanUncertaintyAnglePitch = state.Kalman1DOutput[1];
  //
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n') {
      if (rxLine.length() > 0) {
        //call all functions that manipulate drone outputs or display debugging information
        state.parseControllerLine(rxLine);
        state.DroneStates_input();
        state.displayValues_and_buttonPress();
      }
      rxLine = "";
    } else if (c != '\r') {
      rxLine += c;
      // Prevent runaway String growth if serial data gets corrupted.
      if (rxLine.length() > 170) {
        rxLine = "";
      }
    }
  }
  // Failsafe: if computer/controller bridge to arduino disconnects, set the motors to hover and turn on RGB LED to an orange yellow color.
  if (millis() - lastPacketMs > FAILSAFE_MS) {
    motor1.writeMicroseconds(HOVER_THROTTLE);
    motor2.writeMicroseconds(HOVER_THROTTLE);
    motor3.writeMicroseconds(HOVER_THROTTLE);
    motor4.writeMicroseconds(HOVER_THROTTLE);
  }
}