#include <Arduino.h>
#include <Servo.h>
#include "Wire.h" // This library allows you to communicate with I2C devices.
//ps4 controller used to control outputs
String rxLine;
//Declared servo variable names
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
//---------- Pin setup ----------
//For an RC plane-style brushless motor, the Arduino normally drives the ESC
//with a servo-style signal, not by powering the motor directly.
const int SPEED_CONTROL = 11;
const int IN1 = 12;
const int IN2 = 13;
const int MOTOR1 = 10, MOTOR2 = 11, MOTOR3 = 12, MOTOR4 = 13;
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
//Servo angle limits. Narrow these if your linkage binds mechanically.
const int SERVO_MIN_DEG = 60;
const int SERVO_MAX_DEG = 120;
const int SERVO_CENTER_DEG = 90;
//Safety timeout: if Python stops sending packets, set the motors to hover.
const unsigned long FAILSAFE_MS = 500;
unsigned long lastPacketMs = 0;
//Global variable for checking the internal clock in the arduino
static long int currTime;
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
//returns mapped values from the controller joystick to the python files then to values within the servo range so the servo can read and rotate to those angles
int axisToServoAngle(int axisValue) {
  // axisValue is -100 to +100 from Python.
  return map(axisValue, -100, 100, SERVO_MIN_DEG, SERVO_MAX_DEG);
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
    int motor1Pwm, motor2Pwm, motor3Pwm, motor4Pwm;
    char motor1Dir, motor2Dir ,motor3Dir ,motor4Dir;
    //----------- servo --------
    int servo1Angle, servo2Angle, servo3Angle, servo4Angle;
    bool triangle_lock, square_lock, circle_lock;
    int servoPosition, servoPosition_delay;
    long int prevServoTime;
    bool ServoTimerActive;
    long int MoveServo_StartTime, prev_move_servo_time;
    bool servo_free_move;
    //----------- drone states --------
    int drone_state, prevState; //Variables to store the different states of the drone
    //---------- gyroscope and accelerometer ----------
    char tmp_str[7]; // temporary variable used in convert function
    int MPU_ADDR; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
    int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
    int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
    int16_t temperature; // variables for temperature data
  public:
    DroneState(); //constructor
    int get_MPU_ADDR() { return MPU_ADDR; } //gets the address of the MPU-6050 which is the chip storing the
    char* convert_int16_to_str (int16_t i) { //converts the int16_t to strings by storing them into a array called tmp_str and returning those values
      sprintf(tmp_str, "%6d", i);
      return tmp_str;
    } // this function is part of the gyro and accelerometer starter code
    void parseControllerLine(const String &line);
    void servo_motor_commands();
    void state_circle();
    void state_triangle();
    void displayValues_and_buttonPress() const;
    void DroneStates_input();
    //---------- gyroscope and accelerometer function----------
    void start_MPU_data();
    void Gyro_Accle_setInfo();
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
  motor1Pwm = 0;
  motor2Pwm = 0;
  motor3Pwm = 0;
  motor4Pwm = 0;
  motor1Dir = '0';
  motor2Dir = '0';
  motor3Dir = '0';
  motor4Dir = '0';
  triangle_lock = false;
  square_lock = false;
  circle_lock = true;
  servoPosition = 0;
  servoPosition_delay = 0;
  prevServoTime = 0;
  ServoTimerActive = false;
  MoveServo_StartTime = 0; 
  prev_move_servo_time = 0;
  drone_state = 0;
  prevState = 0;
  servo_free_move = true;
  MPU_ADDR = 0x68;
}
//This function is responsible for storing all the python read values from the controller to then store them into variables in here
//to allow for the arduino to read and understand inputs from the ps4 controller.
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
//Function to update variables corressponding to the motors and servo from the controller
void DroneState::servo_motor_commands() {
  // ---------- Servos ----------
  
  /*servo1Angle = axisToServoAngle(ry);
  servo2Angle = axisToServoAngle(ry);
  servo3Angle = axisToServoAngle(ry);
  servo4Angle = axisToServoAngle(ry);*/
  // ---------- motors ----------
  if (r1 && !cross) {
    //variable to map the rpm of the motor to a percentage.
    static int throttlePercent1;
    static int throttlePercent2;
    static int throttlePercent3;
    static int throttlePercent4;
    //maps the left joystick's y axis into a percent value based on the PWM values from the ESC
    if(ly > 0) {
      throttlePercent1 = constrain(ly, 0, 100);
      throttlePercent2 = constrain(ly, 0, 100);
      throttlePercent3 = constrain(ly, 0, 100);
      throttlePercent4 = constrain(ly, 0, 100);
      motor1Pwm = map(throttlePercent1, 0, 100, ESC_MIN_US, ESC_MAX_US);
      motor2Pwm = map(throttlePercent2, 0, 100, ESC_MIN_US, ESC_MAX_US);
      motor3Pwm = map(throttlePercent3, 0, 100, ESC_MIN_US, ESC_MAX_US);
      motor4Pwm = map(throttlePercent4, 0, 100, ESC_MIN_US, ESC_MAX_US);
    } else if (ly < 0) {
      throttlePercent1 = constrain(ly, -100, 0);
      throttlePercent2 = constrain(ly, -100, 0);
      throttlePercent3 = constrain(ly, -100, 0);
      throttlePercent4 = constrain(ly, -100, 0);
      motor1Pwm = map(throttlePercent1, -100, 0, ESC_MAX_US, ESC_MIN_US);
      motor2Pwm = map(throttlePercent2, -100, 0, ESC_MAX_US, ESC_MIN_US);
      motor3Pwm = map(throttlePercent3, -100, 0, ESC_MAX_US, ESC_MIN_US);
      motor4Pwm = map(throttlePercent4, -100, 0, ESC_MAX_US, ESC_MIN_US);
    }
  }
  //When the r1 button on the controller is held down the left joystick can be used to drive power into each motor.
  //When r1 isn't held down the motors cannot be activated even if the left joystick is being used.
  //If the cross button is also pressed then the motors cannot rotate in any circumstance.
  if (motor1Pwm == 0 || cross || !r1) {
    analogWrite(SPEED_CONTROL, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    motor1Pwm = 0;
    motor1Dir = '0';
  } else if (ly > 0) {
    motor1Dir = '+';
    analogWrite(SPEED_CONTROL, motor1Pwm);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (ly < 0) {
    motor1Dir = '-';
    analogWrite(SPEED_CONTROL, motor1Pwm);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
}

//This function corresponds to Drone_state 3 which is where the two front servos get locked 
//to 180 degrees while the two back servos are still able to freely rotate.
//(Similar to a vtol aircraft)
void DroneState::state_triangle() {
  //Checks if the bool triangle_lock is false (not locked) and if it is false, gives permission to the front two servos to rotate 
  //to the 180 degree position. This also locks all servos from being controlled by the ps4 controller untill unlocked later 
  //in the code below.
  if (!triangle_lock) {
    servoPosition = axisToServoAngle(ry);
    servoPosition_delay = axisToServoAngle(ry);
    ServoTimerActive = false;
    servo_free_move = false;
    MoveServo_StartTime = currTime;
    ServoTimerActive = true;
  }
  //This if statement checks if the previous state was from the circle state and checks if the servoPositions are not yet locked at 180
  if(prevState == 0 || (servoPosition < 180)) {
    //If those conditions are true the two front servos will slowly rotate to the 180 degree position 
    if(currTime - prevServoTime >= 40) {
      prevServoTime = millis();
      servoPosition++;
      servo1Angle = servoPosition;
      servo4Angle = servoPosition;
      servo1.write(servoPosition);
      servo4.write(servoPosition);
    }
  }
  //As long as ServoTimerActive == true it will wait for a specific delay depending on where the servos are positioned right before this
  //line of code and check if servo_free_move == false before the two back servos can recieve inputs from the controller for saftey hazards 
  if(ServoTimerActive) {
    if(((currTime - MoveServo_StartTime) >= (((180 - servoPosition_delay) * 40) + 500)) && (!servo_free_move)) {
      servo_free_move = true; //unlocks the two back servos
      ServoTimerActive = false; //stops tracking the timer
    }
  }
  //If the two back servos are unlocked they will be able to recieve input from the controller
  if (servo_free_move) {
    //Sends the mapped values from the right joystick and servo values into each individual 
    //servo variable.
    servo2Angle = axisToServoAngle(ry);
    servo3Angle = axisToServoAngle(ry);
    servo2.write(servo2Angle);
    servo3.write(servo3Angle);
  }
  /*locks the triangle button to not allow the code to repeat the beginning stage for the triangle state which is the motion
  of moving the front two servo motors into fixed 180 degree positions.*/
  //unlocks the circle button to allow the beginning of the state to be conducted
  triangle_lock = true;
  circle_lock = false;
}

//This function corresponds with the default drone state allowing the drone to fly like a normal drone
//giving full access to all servos
void DroneState::state_circle() {
  //Checks if the bool circle_lock is false (not locked) and if it is false, allow the servos to move back into the 90 degree position from of the circle state
  //for the drone. This also locks all servos from being controlled by the ps4 controller untill unlocked later in the code below
  if(!circle_lock) {
    servoPosition = 180;
    ServoTimerActive = false;
    servo_free_move = false;
    MoveServo_StartTime = currTime;
    ServoTimerActive = true;
  }
  //This if statement checks if the previous state was from the triangle state and checks if the servoPositions are not yet locked at 90
  if(prevState == 3 || (servoPosition >= 90)) {
    //If either conditions are met then the front two servos will slowly rotate to 90 degrees
    if(currTime - prevServoTime >= 40) {
      prevServoTime = millis();
      servoPosition--;
      servo1Angle = servoPosition;
      servo4Angle = servoPosition;
      servo1.write(servoPosition);
      servo4.write(servoPosition);
    }
  }
  //As long as ServoTimerActive == true it will wait for a 3600 delay and check if servo_free_move == false which is when the two front
  //servos are done rotating back to 90 degrees before all sevos can recieve inputs from the controller for saftey hazards.
  //ServoTimerActive is used to track the time from the mills() function stored in the currTime variable to allow for the delay to occur
  if(ServoTimerActive) {
    if(((currTime - MoveServo_StartTime) >= 4500) && (!servo_free_move)) {
      servo_free_move = true; //unlocks all servos
      ServoTimerActive = false; //stops tracking the timer
    }
  }
  //If the all servos are unlocked they will be able to recieve input from the controller
  if (servo_free_move ) {
    //Sends the mapped values from the left joystick and servo values into each individual 
    //servo variable.
    servo1Angle = axisToServoAngle(ry);
    servo2Angle = axisToServoAngle(ry);
    servo3Angle = axisToServoAngle(ry);
    servo4Angle = axisToServoAngle(ry);
    servo1.write(servo1Angle);
    servo2.write(servo2Angle);
    servo3.write(servo3Angle);
    servo4.write(servo4Angle);
  }
  /*locks the triangle button to not allow the code to repeat the beginning stage for the triangle state which is the motion
  of moving the front two servo motors into fixed 180 degree positions.*/
  //unlocks the triangle button to allow the beginning of the state to be conducted
  triangle_lock = false;
  circle_lock = true;
}
/*displays all the values from the controller onto the drone app while 
also turing the RGB LED on to specific colors when specific buttons are pressed.*/
void DroneState::displayValues_and_buttonPress() const{
  static long int last_RGB_time;
  bool buttonPress = false;
  // ---------- Debug output ----------
  Serial.print("ACK motor_pwm=");
  Serial.print(motor1Pwm);
  Serial.print(motor2Pwm);
  Serial.print(motor3Pwm);
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
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));*/
  //checks if the cross button has been pressed and if so the RGB LED will turn on to a light blue color 
  //and on the serial monitor " Kill" will be displayed
  if (cross) {
    buttonPress = true;
    Serial.print(" KILL");
    analogWrite(RED, 4);
    analogWrite(GREEN, 206);
    analogWrite(BLUE, 255);
  }
  //Prints " MOTOR_LOCKED" if r1 is not pressed
  if (!r1) Serial.print(" MOTOR_LOCKED");
  //if r1 is pressed turn on the RGB LED to a purple color
  if (r1) 
  {
    buttonPress = true;
    analogWrite(RED, 209);
    analogWrite(GREEN, 4);
    analogWrite(BLUE, 247);
  }
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

//Function to allow the gyroscope and accelerometer to communicate to the arduino
void DroneState::start_MPU_data() { 
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
}
//Function to store the data from the gyroscope and accelerometer into 7 different variables 
void DroneState::Gyro_Accle_setInfo() {
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
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
  if (drone_state == 3)
    state.state_triangle();
  else if (drone_state == 0)
    state.state_circle();
}
//Setting up all input pins, output pins, and hardware
void setup() {
  Serial.begin(115200);
  rxLine.reserve(180); //reserves 180 bytes of memory for this string
  pinMode(SPEED_CONTROL, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT); 
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo1.write(SERVO_CENTER_DEG);
  servo2.write(SERVO_CENTER_DEG);
  servo3.write(SERVO_CENTER_DEG);
  servo4.write(SERVO_CENTER_DEG);
  Wire.begin();
  Wire.beginTransmission(state.get_MPU_ADDR()); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  byte error = Wire.endTransmission(true);
  Serial.print("Wakeup error = ");
  Serial.println(error);
  Serial.print("Using address 0x");
  Serial.println(state.get_MPU_ADDR(), HEX);
  Serial.println("Arduino bridge ready: ");
  //Using small delay to give the ESC time to see minimum throttle on startup.
  delay(2000);
  lastPacketMs = millis();
}

void loop() {
  //checks the interal clock in the arduino
  currTime = millis();
  //
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n') {
      if (rxLine.length() > 0) {
        //call all functions that manipulate drone outputs or display debugging information
        state.parseControllerLine(rxLine);
        state.DroneStates_input();
        state.servo_motor_commands();
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
    static long int last_Speed_Control;
    static int speedNum = 1;
    /*analogWrite(SPEED_CONTROL, 150);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);*/
    analogWrite(SPEED_CONTROL, (ESC_MAX_US - speedNum));
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(RED, 255);
    analogWrite(GREEN, 70);
    analogWrite(BLUE, 0);
    if(millis() - last_Speed_Control > 75) {
      last_Speed_Control = millis();
      speedNum++;
    }
    if(speedNum >= 255)
    {
      analogWrite(SPEED_CONTROL, 0);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }
  }
  //gyroscope and accelorometer functions called
  state.start_MPU_data();
  state.Gyro_Accle_setInfo();
}