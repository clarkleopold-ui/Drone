#include <Arduino.h>
#include <Servo.h>

// ---------- Pin setup ----------
// For an RC plane-style brushless motor, the Arduino normally drives the ESC
// with a servo-style signal, not by powering the motor directly.
const int SPEED_CONTROL = 13;
const int IN1 = 12;
const int IN2 = 11;
const int SERVO1_PIN = 4;
const int SERVO2_PIN = 5;

// ESC pulse range. Many ESCs use 1000 us = off, 2000 us = full throttle.
// Calibrate your ESC separately if needed.
const int ESC_MIN_US = 0;
const int ESC_MAX_US = 255;

// Servo angle limits. Narrow these if your linkage binds mechanically.
const int SERVO_MIN_DEG = 20;
const int SERVO_MAX_DEG = 160;
const int SERVO_CENTER_DEG = 90;

// Safety timeout: if Python stops sending packets, kill the motor.
const unsigned long FAILSAFE_MS = 500;

struct ControllerState {
  int lx = 0;
  int ly = 0;
  int rx = 0;
  int ry = 0;
  int cross = 0;
  int circle = 0;
  int square = 0;
  int triangle = 0;
  int l1 = 0;
  int r1 = 0;
};

ControllerState state;
String rxLine;
Servo esc;
Servo servo1;
Servo servo2;
unsigned long lastPacketMs = 0;

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

void parseControllerLine(const String &line) {
  state.lx = constrain(readIntField(line, "lx", state.lx), -100, 100);
  state.ly = constrain(readIntField(line, "ly", state.ly), -100, 100);
  state.rx = constrain(readIntField(line, "rx", state.rx), -100, 100);
  state.ry = constrain(readIntField(line, "ry", state.ry), -100, 100);
  state.cross = readIntField(line, "cross", state.cross);
  state.circle = readIntField(line, "circle", state.circle);
  state.square = readIntField(line, "square", state.square);
  state.triangle = readIntField(line, "triangle", state.triangle);
  state.l1 = readIntField(line, "l1", state.l1);
  state.r1 = readIntField(line, "r1", state.r1);

  lastPacketMs = millis();
}

int axisToServoAngle(int axisValue) {
  // axisValue is -100 to +100 from Python.
  return map(axisValue, -100, 100, SERVO_MIN_DEG, SERVO_MAX_DEG);
}

void motorOff() {
  //esc.writeMicroseconds(ESC_MIN_US);
  analogWrite(SPEED_CONTROL, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void applyCommands() {
  // ---------- Motor / L298 ----------
  int motorPwm = 0;
  char motorDir = '0';

  if (state.r1 && !state.cross) {
    // Only allow forward throttle.
    int throttlePercent;
    if(state.ly > 0) {
      throttlePercent = constrain(state.ly, 0, 100);
      motorPwm = map(throttlePercent, 0, 100, ESC_MIN_US, ESC_MAX_US);
    } else if (state.ly < 0) {
      throttlePercent = constrain(state.ly, -100, 0);
      motorPwm = map(throttlePercent, -100, 0, ESC_MAX_US, ESC_MIN_US);
    }
    //motorPwm = map(throttlePercent, 0, 100, ESC_MIN_US, ESC_MAX_US);
  }

  if (motorPwm <= 0 || state.cross || !state.r1) {
    Serial.println("HI");
    analogWrite(SPEED_CONTROL, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  } else if (state.ly > 0) {
    motorDir = '+';
    Serial.println("CW");
    analogWrite(SPEED_CONTROL, motorPwm);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (state.ly < 0) {
    motorDir = '-';
    Serial.println("Test");
    analogWrite(SPEED_CONTROL, motorPwm);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  // ---------- Servos ----------
  int servo1Angle = axisToServoAngle(state.rx);
  int servo2Angle = axisToServoAngle(state.ry);

  if (state.l1) {
    servo1Angle = SERVO_CENTER_DEG;
    servo2Angle = SERVO_CENTER_DEG;
  }

  servo1.write(servo1Angle);
  servo2.write(servo2Angle);

  // ---------- Debug output ----------
  Serial.print("ACK motor_pwm=");
  Serial.print(motorPwm);
  Serial.print(" servo1=");
  Serial.print(servo1Angle);
  Serial.print(" servo2=");
  Serial.print(servo2Angle);
  Serial.print(" lx=");
  Serial.print(state.lx);
  Serial.print(" ly=");
  Serial.print(state.ly);
  Serial.print(" rx=");
  Serial.print(state.rx);
  Serial.print(" ry=");
  Serial.print(state.ry);
  Serial.print(" motor_dir=");
  Serial.print(motorDir);

  if (state.cross) Serial.print(" KILL");
  if (!state.r1) Serial.print(" MOTOR_LOCKED");
  if (state.l1) Serial.print(" SERVOS_CENTERED");

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  rxLine.reserve(180);

  pinMode(SPEED_CONTROL, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  servo1.write(SERVO_CENTER_DEG);
  servo2.write(SERVO_CENTER_DEG);

  // Give the ESC time to see minimum throttle on startup.
  delay(2000);

  lastPacketMs = millis();
  Serial.println("Arduino bridge ready: ESC pin 9, servo pins 10/11");
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n') {
      if (rxLine.length() > 0) {
        parseControllerLine(rxLine);
        applyCommands();
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

  // Failsafe: if computer/controller bridge disconnects, stop the motor.
  if (millis() - lastPacketMs > FAILSAFE_MS) {
    analogWrite(SPEED_CONTROL, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}
