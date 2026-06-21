#include <Arduino.h>
#include <Servo.h>
#include "Wire.h" // This library allows you to communicate with the gyroscope, accelorometer, and altimeter devices.
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BMP280.h>
//ps4 controller used to control outputs
String rxLine;
//Declared servo variable names
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
//---------- Pin setup ----------
const int MOTOR1_SPEED = 10, MOTOR2_SPEED = 11, MOTOR3_SPEED = 12, MOTOR4_SPEED = 13;
const int SERVO1_PIN = 3;
const int SERVO2_PIN = 4;
const int SERVO3_PIN = 5;
const int SERVO4_PIN = 6;
const int BLUE = 7;
const int GREEN = 8; 
const int RED = 9;
//---------- Global Variables ----------
//ESC pulse range. Many ESCs use 1000 us = off, 2000 us = full throttle.
//Calibrate your ESC separately if needed.
const int ESC_MIN_US = 0;
const int ESC_MAX_US = 255;
const int HOVER_THROTTLE = 127;
const int CRUISE_SPEED = 153;
//Servo angle limits. Narrow these if your linkage binds mechanically.
const int SERVO_MIN_DEG = 60;
const int SERVO_MAX_DEG = 120;
const int SERVO_CENTER_DEG = 90;
//Safety timeout: if Python stops sending packets, set the motors to hover.
const unsigned long FAILSAFE_MS = 500;
unsigned long lastPacketMs = 0;
//Global variable for checking the internal clock in the arduino
static long int currTime;
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
//Drone class
class DroneState {
  private:
    //---------- controller ----------
    int lx, ly, rx, ry;
    int cross, circle, square, triangle;
    int l1, r1;
    // ---------- Motor / L298 ----------
    int motor1Pwm, motor2Pwm, motor3Pwm, motor4Pwm; //variables to store pwm values for each motor
    char motor1Dir, motor2Dir ,motor3Dir ,motor4Dir; //variables to store if the throttle of each motor is increasing, decreasing, or constant for debugging purposes
    int throttlePercent1, throttlePercent2, throttlePercent3, throttlePercent4; //variables to map the rpm of the motor to a percentage.
    //----------- servo --------
    int servo1Angle, servo2Angle, servo3Angle, servo4Angle;
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
    void Gyro_Accle_setInfo();
    float RateRoll, RatePitch, RateYaw; // stores raw angular velocity for each axis of movement in degrees/sec 
    float AngleRoll, AnglePitch, AngleYaw; // stores final values of the angle for pitch and roll axis
    uint32_t LoopTimer; //timer to keep track of how often the code runs to ensure it runs every 4ms
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
  cross = 0;
  circle = 0;
  square = 0;
  triangle = 0;
  l1 = 0;
  r1 = 0;
  motor1Pwm = HOVER_THROTTLE;
  motor2Pwm = HOVER_THROTTLE;
  motor3Pwm = HOVER_THROTTLE;
  motor4Pwm = HOVER_THROTTLE;
  triangle_lock = false;
  square_lock = false;
  circle_lock = true;
  servoPosition = 0;
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
    motor1Dir = motor2Dir =  motor3Dir =  motor4Dir = 'C';
    //all motors set to hover
    motor1Pwm = motor2Pwm = motor3Pwm = motor4Pwm = HOVER_THROTTLE;
    throttlePercent1 = throttlePercent2 = throttlePercent3 = throttlePercent4 = (HOVER_THROTTLE/ESC_MAX_US)*100;;
  }
  if ((100 > ly > -100) && r1 && free_move) {
    //maps the left joystick's y axis into a percent value then maps those percentages to PWM values for each motor through the ESC
    throttlePercent1 = throttlePercent2 = throttlePercent3 = throttlePercent4 = constrain(ly, -100, 100);
    motor1Pwm = map(throttlePercent1, -100, 100, ESC_MIN_US, ESC_MAX_US);
    motor2Pwm = map(throttlePercent2, -100, 100, ESC_MIN_US, ESC_MAX_US);
    motor3Pwm = map(throttlePercent3, -100, 100, ESC_MIN_US, ESC_MAX_US);
    motor4Pwm = map(throttlePercent4, -100, 100, ESC_MIN_US, ESC_MAX_US);
    //Check each motors spin direction
    if(ly > 0)
      motor1Dir = motor2Dir = motor3Dir = motor4Dir = '+';
    else if (ly < 0)
      motor1Dir = motor2Dir = motor3Dir = motor4Dir = '-';
    else 
      motor1Dir = motor2Dir = motor3Dir = motor4Dir = 'C';
  }
  //When moving the right joystick in the x-axis it will tilt the drone left and right only
  //if the left loystick is not moving in the y-axis
  if (rx > 0 && free_move) {
    throttlePercent1 = throttlePercent2 = throttlePercent3 = throttlePercent4 = constrain(rx, 0, 100);
    //decrease throttle
    motor2Pwm = map(throttlePercent2, 0, 100, HOVER_THROTTLE, HOVER_THROTTLE - (HOVER_THROTTLE*0.15));
    motor4Pwm = map(throttlePercent4, 0, 100, HOVER_THROTTLE, HOVER_THROTTLE - (HOVER_THROTTLE*0.15));
    //increase throttle
    motor1Pwm = map(throttlePercent1, 0, 100, HOVER_THROTTLE, HOVER_THROTTLE + (HOVER_THROTTLE*0.15));
    motor3Pwm = map(throttlePercent3, 0, 100, HOVER_THROTTLE, HOVER_THROTTLE + (HOVER_THROTTLE*0.15));
    motor1Dir = '+';
    motor2Dir = '-';
    motor3Dir = '+';
    motor4Dir = '-';
  } else if (rx < 0 && free_move) {
    throttlePercent1 = throttlePercent2 = throttlePercent3 = throttlePercent4 = constrain(rx, -100, 0);
    //decrease throttle
    motor1Pwm = map(throttlePercent1, -100, 0, HOVER_THROTTLE - (HOVER_THROTTLE*0.15), HOVER_THROTTLE);
    motor3Pwm = map(throttlePercent3, -100, 0, HOVER_THROTTLE - (HOVER_THROTTLE*0.15), HOVER_THROTTLE);
    //increase throttle
    motor2Pwm = map(throttlePercent2, -100, 0, HOVER_THROTTLE + (HOVER_THROTTLE*0.15), HOVER_THROTTLE);
    motor4Pwm = map(throttlePercent4, -100, 0, HOVER_THROTTLE + (HOVER_THROTTLE*0.15), HOVER_THROTTLE);
    motor1Dir = motor3Dir = '-';
    motor2Dir = motor4Dir = '+';
  }
  //This if statement controls the yawn of the drone, allowing the drone
  //to rotate/spin in place.
  //This spins the drone CW
  if(lx > 0 && !r1 && free_move) {
    throttlePercent1 = throttlePercent2 = throttlePercent3 = throttlePercent4 = constrain(lx, 0, 100);
    //decrease throttle
    motor2Pwm = map(throttlePercent1, 0, 100, HOVER_THROTTLE, HOVER_THROTTLE - (HOVER_THROTTLE*0.3));
    motor3Pwm = map(throttlePercent3, 0, 100, HOVER_THROTTLE, HOVER_THROTTLE - (HOVER_THROTTLE*0.3));
    //increase throttle
    motor1Pwm = map(throttlePercent2, 0, 100, HOVER_THROTTLE, HOVER_THROTTLE + (HOVER_THROTTLE*0.3));
    motor4Pwm = map(throttlePercent4, 0, 100, HOVER_THROTTLE, HOVER_THROTTLE + (HOVER_THROTTLE*0.3));
    motor1Dir = motor4Dir = '+';
    motor2Dir = motor3Dir = '-';
    //This spins the drone CCW
  } else if (lx < 0 && !r1 && free_move) {
    throttlePercent1 = throttlePercent2 = throttlePercent3 = throttlePercent4 = constrain(lx, -100, 0);
    //decrease throttle
    motor1Pwm = map(throttlePercent1, -100, 0, HOVER_THROTTLE - (HOVER_THROTTLE*0.3), HOVER_THROTTLE);
    motor4Pwm = map(throttlePercent3, -100, 0, HOVER_THROTTLE - (HOVER_THROTTLE*0.3), HOVER_THROTTLE);
    //increase throttle
    motor2Pwm = map(throttlePercent2, -100, 0, HOVER_THROTTLE + (HOVER_THROTTLE*0.3), HOVER_THROTTLE);
    motor3Pwm = map(throttlePercent4, -100, 0, HOVER_THROTTLE + (HOVER_THROTTLE*0.3), HOVER_THROTTLE);
    motor1Dir = motor4Dir = '-';
    motor2Dir = motor3Dir = '+';
  }
  analogWrite(MOTOR1_SPEED, motor1Pwm);
  analogWrite(MOTOR2_SPEED, motor2Pwm);
  analogWrite(MOTOR3_SPEED, motor3Pwm);
  analogWrite(MOTOR4_SPEED, motor4Pwm);
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
      servo1Angle = servoPosition;
      servo2Angle = servoPosition;
      servo1.write(servoPosition);
      servo2.write(servoPosition);
    }
  } else {
    free_move = true; //unlocks all servos
    triangle_lock = false;  //unlocks the triangle button to allowing for that button to be pressed again
  }
  //If the all servos are unlocked they will be able to recieve input from the controller
  if (free_move ) {
    //Sends the mapped values from the left joystick and servo values into each individual servo variable.
    servo1Angle = state.axisToServoAngle(ry);
    servo2Angle = state.axisToServoAngle(ry);
    servo3Angle = state.axisToServoAngle(ry);
    servo4Angle = state.axisToServoAngle(ry);
    servo1.write(servo1Angle);
    servo2.write(servo2Angle);
    servo3.write(servo3Angle);
    servo4.write(servo4Angle);
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
  if((cross || !r1) && free_move) {
    throttlePercent1 = throttlePercent2 = (CRUISE_SPEED/ESC_MAX_US)*100;
    throttlePercent3 = throttlePercent4 = (HOVER_THROTTLE/ESC_MAX_US)*100;
    motor1Pwm = motor2Pwm = CRUISE_SPEED;
    motor3Pwm = motor4Pwm = HOVER_THROTTLE;
    motor1Dir = motor2Dir = motor3Dir = motor4Dir = 'C';
  //When the 2 front motors are free to move and the left joystick in the y-axis greater than 0
  //increase the motor rpm starting from the cruise speed rpm to the max rpm
  } else if ((ly >= 0) && free_move) {
    throttlePercent1 = throttlePercent2 = constrain(ly, 0, 100);
    motor1Pwm = map(throttlePercent1, 0, 100, CRUISE_SPEED, ESC_MAX_US);
    motor2Pwm = map(throttlePercent2, 0, 100, CRUISE_SPEED, ESC_MAX_US);
    motor1Dir = motor2Dir = '+';
  //Changes the rmp of the back 2 motors by a +- 10%
  } else if ((100 > ry > -100) && free_move) {
    throttlePercent3 = throttlePercent4 = constrain(ry, -100, 100);
    motor3Pwm = map(throttlePercent3, -100, 100, HOVER_THROTTLE - (HOVER_THROTTLE*0.1), HOVER_THROTTLE + (HOVER_THROTTLE*0.1));
    motor4Pwm = map(throttlePercent4, -100, 100, HOVER_THROTTLE - (HOVER_THROTTLE*0.1), HOVER_THROTTLE + (HOVER_THROTTLE*0.1));
    //Changes the motor4Dir if the 2 back motors are increasing in rpm, decreasing in rpm, or constant
    if (ry > 0)
      motor3Dir = motor4Dir = '+';
    else if (ry < 0)
      motor3Dir = motor4Dir = '-';
    else 
      motor3Dir = motor4Dir = 'C';
  }
  analogWrite(MOTOR1_SPEED, motor1Pwm);
  analogWrite(MOTOR2_SPEED, motor2Pwm);
  analogWrite(MOTOR3_SPEED, motor3Pwm);
  analogWrite(MOTOR4_SPEED, motor4Pwm);
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
      servo1Angle = servoPosition;
      servo2Angle = servoPosition;
      servo1.write(servoPosition);
      servo2.write(servoPosition);
    }
  } else {
    free_move = true; //unlocks the two back servos
    circle_lock = false; //unlocks the circle button to allowing for that button to be pressed again
  }
  //If the two back servos are unlocked they will be able to recieve input from the controller
  if (free_move) {
    //Sends the mapped values from the right joystick and servo values into each individual 
    //servo variable.
    servo3Angle = state.axisToServoAngle(ry);
    servo4Angle = state.axisToServoAngle(ry);
    servo3.write(servo3Angle);
    servo4.write(servo4Angle);
  }
  /*locks the triangle button to not allow the code to repeat the beginning stage for the triangle state which is the motion
  of moving the front two servo motors into fixed 180 degree positions.*/
  triangle_lock = true;
}
/*displays all the values from the controller onto the drone app while 
also turing the RGB LED on to specific colors when specific buttons are pressed.*/
void DroneState::displayValues_and_buttonPress() const{
  static long int last_RGB_time;
  bool buttonPress = false;
  // ---------- Debug output ----------
  Serial.print("ACK motor_pwm=");
  Serial.print(motor1Pwm);
  Serial.print(" ");
  Serial.print(motor2Pwm);
  Serial.print(" ");
  Serial.print(motor3Pwm);
  Serial.print(" ");
  Serial.print(motor4Pwm);
  Serial.print(" servo1=");
  Serial.print(servo1Angle);
  Serial.print(" servo2=");
  Serial.print(servo2Angle);
  Serial.print(" servo3=");
  Serial.print(servo3Angle);
  Serial.print(" servo4=");
  Serial.print(servo4Angle);
  Serial.print(" lx=");
  Serial.print(lx);
  Serial.print(" ly=");
  Serial.print(ly);
  Serial.print(" rx=");
  Serial.print(rx);
  Serial.print(" ry=");
  Serial.print(ry);
  Serial.print(" motor_dir=");
  Serial.print(motor1Dir);
  Serial.print(motor2Dir);
  Serial.print(motor3Dir);
  Serial.print(motor4Dir);
  // print out gyroscope and accelerometer data
  /*Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
  Serial.print("Roll [°]: ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" | Pitch [°]: ");
  Serial.print(KalmanAnglePitch);
  Serial.print(" | Yaw [°]: ");
  Serial.println(AngleYaw);*/
  //checks if the cross button has been pressed and if so the RGB LED will turn on to a light blue color 
  //and on the serial monitor " Kill" will be displayed
  if (cross) {
    buttonPress = true;
    Serial.print(" KILL");
    analogWrite(RED, 4);
    analogWrite(GREEN, 206);
    analogWrite(BLUE, 255);
    //if r1 is pressed turn on the RGB LED to a purple color
  } else if (r1) {
    buttonPress = true;
    analogWrite(RED, 209);
    analogWrite(GREEN, 4);
    analogWrite(BLUE, 247);
  }
  //Prints " MOTOR_LOCKED" if r1 is not pressed
  if (!r1) Serial.print(" MOTOR_LOCKED");
  //turns the RGB LED off after a 1 second time delay
  if (!buttonPress && (currTime - last_RGB_time) > 1000)
  {
    last_RGB_time = millis();
    analogWrite(RED, 0);
    analogWrite(GREEN, 0);
    analogWrite(BLUE, 0);
  }
  Serial.println();
}
/*Function to allow the gyroscope and accelerometer to communicate to the arduino.
//This is temporary and many not be used.
void DroneState::start_MPU_data() { 
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
}*/
//Function to store the data from the gyroscope and accelerometer and
//converts that raw data to readable data in units like degrees/sec.
void DroneState::Gyro_Accle_setInfo() {
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
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}
//Setting up all input pins, output pins, and hardware
void setup() {
  Serial.begin(115200);
  rxLine.reserve(180); //reserves 180 bytes of memory for this string
  //Motor setup
  pinMode(MOTOR1_SPEED, OUTPUT);
  pinMode(MOTOR2_SPEED, OUTPUT);
  pinMode(MOTOR3_SPEED, OUTPUT);
  pinMode(MOTOR4_SPEED, OUTPUT);
  //RGB LED setup
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);
  //Servo setup
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
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
    state.Gyro_Accle_setInfo();
    state.RateCalibrationRoll += state.RateRoll;
    state.RateCalibrationPitch += state.RatePitch;
    state.RateCalibrationYaw += state.RateYaw;
    delay(1);
  }
  state.RateCalibrationRoll/=2000;
  state.RateCalibrationPitch/=2000;
  state.RateCalibrationYaw/=2000;
  state.LoopTimer=micros();
  Serial.println("Arduino bridge ready: ");
  //Using small delay to give the ESC time to see minimum throttle on startup.
  delay(2000);
  lastPacketMs = millis();
}

void loop() {
  //checks the interal clock in the arduino
  currTime = millis();
  state.Gyro_Accle_setInfo();
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
    analogWrite(MOTOR1_SPEED, 150);
    analogWrite(MOTOR2_SPEED, 150);
    analogWrite(MOTOR3_SPEED, 150);
    analogWrite(MOTOR4_SPEED, 150);
    analogWrite(RED, 255);
    analogWrite(GREEN, 70);
    analogWrite(BLUE, 0);
  }
  while (micros() - state.LoopTimer < 4000);
  state.LoopTimer= micros();
}