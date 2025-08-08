#include <Wire.h>
#include <MPU6050_light.h>

// Motor Pins
const int aPwm = 11, Ain1 = 5, Ain2 = 4;
const int bPwm = 6, Bin1 = 8, Bin2 = 7;
const int Stby = 12;

// Encoder Pins
const int Ac1 = 3, Ac2 = 10;
const int Bc1 = 2, Bc2 = 9;

// Button Pins
const int leftBtnPin = A7;
const int forwardBtnPin = A6;

// IR Pins
const int frontLeftIR  = A3;
const int frontRightIR = A1;
const int rightIR      = A2;
const int leftIR       = A0;

// Ticks
volatile long leftTicks = 0, rightTicks = 0;
bool lflag = true, rflag = true;

bool turning = false;

// MPU and PID
MPU6050 mpu(Wire);

// Custom PID variables

float previousError = 0, integral = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  delay(1000);
  mpu.calcOffsets(true, true); // Calibrate

  // Motor setup
  pinMode(aPwm, OUTPUT); pinMode(Ain1, OUTPUT); pinMode(Ain2, OUTPUT);
  pinMode(bPwm, OUTPUT); pinMode(Bin1, OUTPUT); pinMode(Bin2, OUTPUT);
  pinMode(Stby, OUTPUT); digitalWrite(Stby, HIGH);

  // Encoder pins
  pinMode(Ac1, INPUT); pinMode(Ac2, INPUT);
  pinMode(Bc1, INPUT); pinMode(Bc2, INPUT);
  attachInterrupt(digitalPinToInterrupt(Ac1), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Bc1), rightEncoderISR, CHANGE);

  // Buttons
  pinMode(leftBtnPin, INPUT_PULLUP);
  pinMode(forwardBtnPin, INPUT_PULLUP);

  // IR's
  pinMode(frontLeftIR, INPUT);
  pinMode(frontRightIR, INPUT);
  pinMode(rightIR, INPUT);
  pinMode(leftIR, INPUT);
}

void loop() {
  mpu.update();
  
  if (wallFront()) 
  {
    if (wallLeft()) 
      turnRight();
    else 
      turnLeft();
  }
  else
    moveForward();
  delay(3000);
}

// ---------- Walls ----------
bool wallFront(){
  return ((analogRead(frontLeftIR) < 300)|| (analogRead(frontRightIR) < 300));
}
bool wallLeft(){
  return analogRead(leftIR) < 300;
}
bool wallRight(){
  return analogRead(rightIR) < 300;
}


// ---------- Encoder ISRs ----------
void leftEncoderISR() {
  leftTicks++;
  if (leftTicks >= 720 && !turning) {
    lflag = false;
    motorStop('A');
  }
}
void rightEncoderISR() {
  rightTicks++;
  if (rightTicks >= 720 && !turning) {
    rflag = false;
    motorStop('B');
  }
}

// ---------- Movement ----------
void moveForward() {
  lflag = rflag = true;
  leftTicks = rightTicks = 0;
  turning = false;
  while(1){
    move('A', "ACW", 62);
    move('B', "ACW", 60);

    if (leftTicks >= 720 && rightTicks >= 720 && !turning) {
      
      break;
    }
    if((analogRead(frontLeftIR) < 34)|| (analogRead(frontRightIR) < 34)){
      break;
    }
  }
  lflag = false;
  rflag = false;
  motorStop('A');
  motorStop('B');
  leftTicks = rightTicks = 0; // Reset ticks after moving
}

void move(char motor, String dir, int spd) {
  digitalWrite(Stby, HIGH);
  if (motor == 'A') {
    digitalWrite(Ain1, dir == "CW");
    digitalWrite(Ain2, dir != "CW");
    analogWrite(aPwm, spd);
  } else {
    digitalWrite(Bin1, dir == "CW");
    digitalWrite(Bin2, dir != "CW");
    analogWrite(bPwm, spd);
  }
}

void motorStop(char motor) {
  if (motor == 'A') {
    digitalWrite(Ain1, HIGH);
    digitalWrite(Ain2, HIGH);
    analogWrite(aPwm, 0);
  } else {
    digitalWrite(Bin1, HIGH);
    digitalWrite(Bin2, HIGH);
    analogWrite(bPwm, 0);
  }
}

// ---------- Turning ----------
void turnLeft() {
  mpu.update();
  float initialYaw = mpu.getAngleZ();
  float targetYaw = initialYaw + 90;

  previousError = 0;
  integral = 0;
  lastTime = millis();
  turning = true;
  while (true) {
    mpu.update();

    leftTicks = rightTicks = 0; // Reset ticks for turning
    float currentYaw = mpu.getAngleZ();
    float error = targetYaw - currentYaw;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    float Kp = 0.5, Ki = 0.01, Kd = 5;

    integral += error;
    float derivative = (error - previousError);
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    if (error < 20) break;
    if(integral > 10000) integral = 10000;
    // Serial.println(output);

    rotateInPlace(30 + output);
  }

  motorStop('A');
  motorStop('B');
  turning = false;
  leftTicks = rightTicks = 0; // Reset ticks after turning
}

void turnRight() {
  mpu.update();
  
  float initialYaw = mpu.getAngleZ();
  float targetYaw = initialYaw - 90;
  
  previousError = 0;
  integral = 0;
  lastTime = millis();
  
  turning = true;
  while (true) {
    mpu.update();
    leftTicks = rightTicks = 0; // Reset ticks for turning
    float currentYaw = mpu.getAngleZ();
    float error = targetYaw - currentYaw;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    float Kp = 0.5, Ki = 0.01, Kd = 5;

    integral += error;
    if (integral > 10000) integral = 10000;
    float derivative = (error - previousError);
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    if (error > -20) break;
    // Serial.print(error);
    // Serial.print(" ");
    // Serial.println(output);
    rotateInPlace(-(30 - output)); // Negative for clockwise rotation
  }

  motorStop('A');
  motorStop('B');
  turning = false;
  leftTicks = rightTicks = 0; // Reset ticks after turning
}

void rotateInPlace(float speed) {
  speed = constrain(speed, -150, 150);
  if (speed > 0) {
    move('A', "CW", abs(speed));
    move('B', "ACW", abs(speed));
  } else {
    move('A', "ACW", abs(speed));
    move('B', "CW", abs(speed));
  }
}
