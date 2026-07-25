#include <HardwareSerial.h>
#include "BluetoothSerial.h"
#define RX1_PIN 17
#define TX1_PIN 16

BluetoothSerial SerialBT;

String device_name = "ESP32";

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

//Struct to hold all variables that stores all raw data for hardware and software components like motors, servos, ps4 controller, etc
struct __attribute__((packed)) RawData {
  int16_t lx, ly, rx, ry;
  int16_t cross, circle, square, triangle;
  int16_t l1, r1;
  // ---------- Motor / 5010-360KV ----------
  int16_t motorPwm[4]; //variables to store pwm values for each motor
  int16_t throttlePercent; //variables to map the rpm of the motor to a percentage.
  int16_t throttle; //variable to store values from the controller to real motor values in circle state
  int16_t roll, pitch, yaw;
  int16_t pitchOffSet, rollOffSet, yawOffSet; 
  int16_t delta; //calculates how much each motor can increase or decrease in value based on each degree of movement (pitch, roll, yaw)
  int16_t front_throttle, back_throttle; //variable to store values from the controller to real motor values in triangle state.
  //----------- servo -------- (will use later)
  int16_t servoAngle[4];
  bool triangle_lock, square_lock, circle_lock;
  int16_t servoPosition;
  bool free_move;
  //----------- drone states --------
  int16_t drone_state, prevState; //Variables to store the different states of the drone
  //---------- gyroscope and accelerometer ----------
  char tmp_str[7]; // temporary array used to store int16 variable values 
  int16_t MPU_ADDR; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
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
RawData RecivedData;

uint8_t rxBuffer[sizeof(RawData)]; // Temporary holding pen for incoming bytes
int rxIndex = 0;                   // Tracks how many bytes we've collected
bool isReceiving = false;          // Tracks if we are currently building a packet

void setup() {
  Serial.begin(115200);   // To view on your Computer's Serial Monitor
  SerialBT.begin(device_name);  //Bluetooth device name
  //SerialBT.deleteAllBondedDevices(); // Uncomment this to delete paired devices; Must be called after begin
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  // Initialize Serial2 with custom pins
  Serial2.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);
}

void loop() {
  // Sends data via bluetooth to computer
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }

  // Read data coming from Arduino Mega
  while (Serial2.available() > 0) {
    if (!isReceiving) {
      // 1. We are NOT currently receiving a packet. Look for the '!' header.
      if (Serial2.read() == '!') {
        isReceiving = true; // Header found! Start collecting the payload.
        rxIndex = 0;        // Reset the byte counter
      }
    } 
    else {
      // 2. We ARE currently receiving a packet. Read the next byte into the buffer.
      rxBuffer[rxIndex] = Serial2.read();
      rxIndex++;

      // 3. Check if we have collected enough bytes to fill the struct
      if (rxIndex >= sizeof(RawData)) {
        
        // Copy the raw bytes from the buffer directly into the struct
        memcpy(&RecivedData, rxBuffer, sizeof(RawData));
        
        displayValues_and_buttonPress();
        
        // Reset our state machine to wait for the next '!' header
        isReceiving = false; 
      }
    }
  }
}

void displayValues_and_buttonPress() {
  // ---------- Debug output ----------
  SerialBT.print("ACK motor_pwm=");
  Serial.print(RecivedData.motorPwm[0]);
  Serial.print(" ");
  Serial.print(RecivedData.motorPwm[1]);
  Serial.print(" ");
  Serial.print(RecivedData.motorPwm[2]);
  Serial.print(" ");
  Serial.print(RecivedData.motorPwm[3]);
  Serial.println();
  /*Serial.print(" servo1=");
  Serial.print(RecivedData.servoAngle[0]);
  Serial.print(" servo2=");
  Serial.print(RecivedData.servoAngle[1]);
  Serial.print(" servo3=");
  Serial.print(RecivedData.servoAngle[2]);
  Serial.print(" servo4=");
  Serial.print(RecivedData.servoAngle[3]);*/
  SerialBT.print(" lx=");
  SerialBT.print(RecivedData.lx);
  SerialBT.print(" ly=");
  SerialBT.print(RecivedData.ly);
  SerialBT.print(" rx=");
  SerialBT.print(RecivedData.rx);
  SerialBT.print(" ry=");
  SerialBT.print(RecivedData.ry);
  SerialBT.print(" Roll [°]: ");
  SerialBT.print(RecivedData.KalmanAngleRoll);
  SerialBT.print(" Pitch [°]: ");
  SerialBT.print(RecivedData.KalmanAnglePitch);
  SerialBT.print(" Yaw [°]: ");
  SerialBT.print(RecivedData.AngleYaw);
  SerialBT.print(" Yaw Rate[°/s^6]: ");
  SerialBT.print(RecivedData.YawRate, 6);
  //checks if the cross button has been pressed and on the serial monitor " Kill" will be displayed
  if (RecivedData.cross) SerialBT.print(" KILL");
  //Prints " MOTOR_LOCKED" if r1 is not pressed
  if (!RecivedData.r1) SerialBT.print(" MOTOR_LOCKED");
  SerialBT.println();
}