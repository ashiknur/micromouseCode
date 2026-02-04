#include <Wire.h>
#include <MPU6050_light.h>
// Motor Pins
const int bPwm = 5, Bin1 = 23, Bin2 = 22;
const int aPwm = 4, Ain1 = 24, Ain2 = 25;
const int Stby = 26;

// Encoder Pins
const int Ac1 = 18, Ac2 = 27;
const int Bc1 = 19, Bc2 = 28;


// Button Pins
const int leftBtnPin = 15;
const int rightBtnPin = 17;

// IR Pins
const int frontLeftIR = A15;
const int frontRightIR = A11;
const int rightIR = A8;
const int leftIR = A12;

// Ticks
volatile long leftTicks = 0, rightTicks = 0;
bool lflag = true, rflag = true;

bool turning = false;

// MPU and PID
MPU6050 mpu(Wire);

// Custom PID variables
float previousError = 0, integral = 0;
unsigned long lastTime = 0;

//////////////////////////////////////////////////////// Algorithm Implementation ////////////////////////////////////////////////////////

enum Heading { NORTH,
               EAST,
               SOUTH,
               WEST };
enum Action { LEFT,
              FORWARD,
              RIGHT,
              IDLE };

bool flag = false;  // Flag to indicate if the goal is reached

struct Node {
  int dist;
  bool wall[4];
  Node()
    : dist(1000){
    for (int i = 0; i < 4; i++) wall[i] = false;
  }
};

int startx = 0, starty = 0;
const int DIM = 16;
const int goalx = (DIM - 1) / 2, goaly = (DIM - 1) / 2;
// const int goalx = 0, goaly = 0;

bool found = false;
Node table[DIM][DIM];
bool visited[DIM][DIM];
int dx[] = { 0, 1, 0, -1 };
int dy[] = { 1, 0, -1, 0 };

const int MAXQ = DIM * DIM * 4;
uint8_t qx[MAXQ], qy[MAXQ];
int qhead, qtail;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  delay(1000);
  mpu.calcOffsets(true, true);

  // Motor setup
  pinMode(aPwm, OUTPUT);
  pinMode(Ain1, OUTPUT);
  pinMode(Ain2, OUTPUT);
  pinMode(bPwm, OUTPUT);
  pinMode(Bin1, OUTPUT);
  pinMode(Bin2, OUTPUT);
  pinMode(Stby, OUTPUT);
  digitalWrite(Stby, HIGH);

  // Encoder pins
  pinMode(Ac1, INPUT);
  pinMode(Ac2, INPUT);
  pinMode(Bc1, INPUT);
  pinMode(Bc2, INPUT);
  attachInterrupt(digitalPinToInterrupt(Ac1), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Bc1), rightEncoderISR, CHANGE);

  // Buttons
  pinMode(leftBtnPin, INPUT_PULLUP);
  pinMode(rightBtnPin, INPUT_PULLUP);

  // IR sensors
  pinMode(frontLeftIR, INPUT);
  pinMode(frontRightIR, INPUT);
  pinMode(rightIR, INPUT);
  pinMode(leftIR, INPUT);
  for(int i = 0; i < DIM; i++){
    for(int j = 0; j < DIM; j++){
      visited[i][j] = true;
    }
  }
}

void initMaze() {
  
  for (int i = 0; i < DIM; i++) {
    for (int j = 0; j < DIM; j++) {
      table[i][j].dist = 1000;
    }
  }
  qhead = qtail = 0;
  if (!flag) {
    qx[qtail] = goalx;
    qy[qtail++] = goaly;
    table[goalx][goaly].dist = 0;
  } else {
    qx[qtail] = startx;
    qy[qtail++] = starty;
    table[startx][starty].dist = 0;
  }
  while (qhead < qtail) {
    int x = qx[qhead], y = qy[qhead++];
    for (int i = 0; i < 4; i++) {
      int nx = x + dx[i], ny = y + dy[i];
      if(!flag) {
        if (nx >= 0 && nx < DIM && ny >= 0 && ny < DIM && !table[x][y].wall[i] && table[nx][ny].dist > table[x][y].dist + 1 && visited[nx][ny]) {
          table[nx][ny].dist = table[x][y].dist + 1;
          qx[qtail] = nx;
          qy[qtail++] = ny;
        }
      }
      else{
        if (nx >= 0 && nx < DIM && ny >= 0 && ny < DIM && !table[x][y].wall[i] && table[nx][ny].dist > table[x][y].dist + 1) {
          table[nx][ny].dist = table[x][y].dist + 1;
          qx[qtail] = nx;
          qy[qtail++] = ny;
        }
      }
    }
  }
}

Action nextMove(int x, int y, Heading heading) {
  int mn = 1000;
  Action nextAction = IDLE;
  visited[x][y] = true;
  if (heading == NORTH) {
    if (wallFront() && y + 1 < DIM) {
      table[x][y].wall[0] = true;
      table[x][y + 1].wall[2] = true;
    }
    if (wallLeft() && x - 1 >= 0) {
      table[x][y].wall[3] = true;
      table[x - 1][y].wall[1] = true;
    }
    if (wallRight() && x + 1 < DIM) {
      table[x][y].wall[1] = true;
      table[x + 1][y].wall[3] = true;
    }
    initMaze();
    mn = table[x][y].dist;
    if (!wallFront() && table[x][y + 1].dist < mn) {
      mn = table[x][y + 1].dist;
      nextAction = FORWARD;
    }
    if (!wallLeft() && table[x - 1][y].dist < mn) {
      mn = table[x - 1][y].dist;
      nextAction = LEFT;
    }
    if (!wallRight() && table[x + 1][y].dist < mn) {
      mn = table[x + 1][y].dist;
      nextAction = RIGHT;
    }
  } else if (heading == EAST) {
    if (wallFront() && x + 1 < DIM) {
      table[x][y].wall[1] = true;
      table[x + 1][y].wall[3] = true;
    }
    if (wallLeft() && y + 1 < DIM) {
      table[x][y].wall[0] = true;
      table[x][y + 1].wall[2] = true;
    }
    if (wallRight() && y - 1 >= 0) {
      table[x][y].wall[2] = true;
      table[x][y - 1].wall[0] = true;
    }
    initMaze();
    mn = table[x][y].dist;
    if (!wallFront() && table[x + 1][y].dist < mn) {
      mn = table[x + 1][y].dist;
      nextAction = FORWARD;
    }
    if (!wallLeft() && table[x][y + 1].dist < mn) {
      mn = table[x][y + 1].dist;
      nextAction = LEFT;
    }
    if (!wallRight() && table[x][y - 1].dist < mn) {
      mn = table[x][y - 1].dist;
      nextAction = RIGHT;
    }
  } else if (heading == SOUTH) {
    if (wallFront() && y - 1 >= 0) {
      table[x][y].wall[2] = true;
      table[x][y - 1].wall[0] = true;
    }
    if (wallLeft() && x + 1 < DIM) {
      table[x][y].wall[1] = true;
      table[x + 1][y].wall[3] = true;
    }
    if (wallRight() && x - 1 >= 0) {
      table[x][y].wall[3] = true;
      table[x - 1][y].wall[1] = true;
    }
    initMaze();
    mn = table[x][y].dist;
    if (!wallFront() && table[x][y - 1].dist < mn) {
      mn = table[x][y - 1].dist;
      nextAction = FORWARD;
    }
    if (!wallLeft() && table[x + 1][y].dist < mn) {
      mn = table[x + 1][y].dist;
      nextAction = LEFT;
    }
    if (!wallRight() && table[x - 1][y].dist < mn) {
      mn = table[x - 1][y].dist;
      nextAction = RIGHT;
    }
  } else if (heading == WEST) {
    if (wallFront() && x - 1 >= 0) {
      table[x][y].wall[3] = true;
      table[x - 1][y].wall[1] = true;
    }
    if (wallLeft() && y - 1 >= 0) {
      table[x][y].wall[2] = true;
      table[x][y - 1].wall[0] = true;
    }
    if (wallRight() && y + 1 < DIM) {
      table[x][y].wall[0] = true;
      table[x][y + 1].wall[2] = true;
    }
    initMaze();
    mn = table[x][y].dist;
    if (!wallFront() && table[x - 1][y].dist < mn) {
      mn = table[x - 1][y].dist;
      nextAction = FORWARD;
    }
    if (!wallLeft() && table[x][y - 1].dist < mn) {
      mn = table[x][y - 1].dist;
      nextAction = LEFT;
    }
    if (!wallRight() && table[x][y + 1].dist < mn) {
      mn = table[x][y + 1].dist;
      nextAction = RIGHT;
    }
  }
  if (nextAction == IDLE) nextAction = LEFT;
  return nextAction;
}

int onWhich = 0;
int calculated = 0;
int lastError = 0;
int baseSpeed = 60;
bool running = false;


void loop() {
  int x = startx, y = starty;
  Heading heading = NORTH;
  // for(int i = 0; i < DIM; i++){
  //   for(int j = 0; j < DIM; j++){
  //     visited[i][j] = false;
  //   }
  // }
  while(1){
    // Serial.println("Press Button");
    int lbtn = false, rbtn = false;
    lbtn = digitalRead(leftBtnPin);
    while(digitalRead(leftBtnPin));
    rbtn = digitalRead(rightBtnPin);
    while(digitalRead(rightBtnPin));
    
    if(!found){
      if(lbtn){
        // Serial.println("Left First");
        startx = 0, starty = 0;
        running = true;
        delay(1000);
      }
      if(rbtn){
        // Serial.println("Right First");
        startx = DIM - 1; starty = 0; // check for correct ness here
        x = startx, y = starty; 
        running = true;
        delay(1000);
      }
    }
    else{
      // Serial.println("ELSE");
      if(lbtn || rbtn){
        running = true;
        // Serial.println("RUN");
        delay(1000);
      }
    }

    while (running) {
      mpu.update();

      // lbtn = digitalRead(leftBtnPin);
      // while(digitalRead(leftBtnPin));
      // rbtn = digitalRead(rightBtnPin);
      // while(digitalRead(rightBtnPin));

      // if(lbtn || rbtn){
      //   running = false;
      //   motorStop('A');
      //   motorStop('B');
      //   break;
      // }
      // visited[x][y] = 1;
      Action nxMove = nextMove(x, y, heading);
      // Action nxMove = IDLE;
      if (flag == false && (x == goalx && y == goaly)) {
        // Serial.println("Goal Reached");
        flag = true;
        found = true;
      }
      if (flag == true && x == startx && y == starty){
        flag = false;
        if(found){
          while(heading != NORTH){
            turnLeft();
            heading = (Heading)((heading + 3) % 4);
          }
          // Serial.println("Here");
          running = false;
          motorStop('A');
          motorStop('B');
          break;
        }
      }

      // for(int i = 0; i < DIM; i++){
      //   for(int j = 0; j < DIM; j++){
      //     Serial.print(table[i][j].dist);
      //     Serial.print(" ");
      //   }
      //   Serial.println();
      // }
      // Serial.println();
      // for(int i = 0; i < DIM; i++){
      //   for(int j = 0; j < DIM; j++){
      //     for(int k = 0; k < 4; k++){
      //       Serial.print(table[i][j].wall[k]);
      //       Serial.print(" ");
      //     }
      //     Serial.print(" | ");
      //   }
      //   Serial.println();
      // }
      // Serial.println();
      switch (nxMove) {
        case FORWARD:
          moveForward();
          // delay(50);
          if (heading == NORTH) y++;
          else if (heading == EAST) x++;
          else if (heading == SOUTH) y--;
          else if (heading == WEST) x--;
          break;
        case LEFT:
          turnLeft();
          // delay(50);
          heading = (Heading)((heading + 3) % 4);
          break;
        case RIGHT:
          turnRight();
          // delay(50);
          heading = (Heading)((heading + 1) % 4);
          break;
        case IDLE: break;
      }
    }
  }
  
}

// ---------- Walls ----------
bool wallFront() {
  return analogRead(frontLeftIR) < 600 || analogRead(frontRightIR) < 600;
}
bool wallLeft() {
  return analogRead(leftIR) < 600;
}
bool wallRight() {
  return analogRead(rightIR) < 600;
}

void bothFollow(){
  int leftIRVal  = analogRead(leftIR);
  int rightIRVal = analogRead(rightIR);

  int frontLeftVal  = analogRead(frontLeftIR);
  int frontRightVal = analogRead(frontRightIR);

  float kp = 0.04;
  float kd = 0.4;
  int error = leftIRVal - rightIRVal;
  int diff = error - lastError;
  lastError = error;
  int correction = error * kp + diff * kd;

  // Serial.println(correction);
  // delay(50);
  move('A', true, constrain(baseSpeed - correction, 40, 80));
  move('B', true, constrain(baseSpeed + correction, 40, 80));

}

void leftFollow(){
  int leftIRVal  = analogRead(leftIR);
  int rightIRVal = analogRead(rightIR);

  int frontLeftVal  = analogRead(frontLeftIR);
  int frontRightVal = analogRead(frontRightIR);

  float kp = 0.04;
  float kd = 0.4;
  int error = leftIRVal - 150;
  int diff = error - lastError;
  lastError = error;
  int correction = error * kp + diff * kd;

  // Serial.println(correction);
  // delay(50);
  move('A', true, constrain(baseSpeed - correction, 50, 80));
  move('B', true, constrain(baseSpeed + correction, 50, 80));

  
}

void rightFollow(){
  int leftIRVal  = analogRead(leftIR);
  int rightIRVal = analogRead(rightIR);

  int frontLeftVal  = analogRead(frontLeftIR);
  int frontRightVal = analogRead(frontRightIR);

  float kp = 0.04;
  float kd = 0.4;
  int error = 220 - rightIRVal;
  int diff = error - lastError;
  lastError = error;
  int correction = error * kp + diff * kd;

  // Serial.println?(correction);
  // delay(50);
  move('A', true, constrain(baseSpeed - correction, 40, 80));
  move('B', true, constrain(baseSpeed + correction, 40, 80));

}
// ---------- Encoder ISRs ----------
void leftEncoderISR() {
  leftTicks++;
  if (leftTicks >= 730 && !turning) {
    lflag = false;
    motorStop('A');
  }
}
void rightEncoderISR() {
  rightTicks++;
  if (rightTicks >= 730 && !turning) {
    rflag = false;
    motorStop('B');
  }
}
// ---------- Movement ----------
void moveForward() {

  lflag = rflag = true;
  leftTicks = rightTicks = 0;
  turning = false;
  float lastErrorForwardMove = 0;
  while (1) {
    int leftIRVal  = analogRead(leftIR);
    int rightIRVal = analogRead(rightIR);

    int frontLeftVal  = analogRead(frontLeftIR);
    int frontRightVal = analogRead(frontRightIR);
    if(wallLeft() && wallRight()){
      if(onWhich != 1){
        lastError = 0;
        onWhich = 1;
        // motorStop('A');
        // motorStop('B');
        // delay(500);
      }
      bothFollow();
    }
    else if(wallLeft()){
      if(onWhich != 2){
        lastError = 0;
        onWhich = 2;
        // motorStop('A');
        // motorStop('B');
        // delay(500);
      }
      leftFollow();
    }
    else if(wallRight()){
      if(onWhich != 3){
        lastError = 0;
        onWhich = 3;
        // motorStop('A');
        // motorStop('B');
        // delay(500);
      }
      rightFollow();
    }
    else{
      move('A', true, constrain(baseSpeed, 40, 80));
      move('B', true, constrain(baseSpeed, 40, 80));
      // motorStop('A');
      // motorStop('B');
    }

    if (leftTicks >= 730 && rightTicks >= 730 && !turning)
      break;

    if ((frontLeftVal < 70 || frontRightVal < 70) && !turning){
      lflag = false;
      rflag = false;
      motorStop('A');
      motorStop('B');
      leftTicks = rightTicks = 0;
      break;
    }
  }

  lflag = false;
  rflag = false;
  motorStop('A');
  motorStop('B');
  leftTicks = rightTicks = 0; // Reset ticks after moving
}

void move(char motor, bool cw, int spd) {
  digitalWrite(Stby, HIGH);
  if (motor == 'A') {
    digitalWrite(Ain1, cw);
    digitalWrite(Ain2, !cw);
    analogWrite(aPwm, spd);
  } else {
    digitalWrite(Bin1, cw);
    digitalWrite(Bin2, !cw);
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

void rotateInPlace(float speed) {
  speed = constrain(speed, -150, 150);
  if (speed > 0) {
    move('A', true, speed);
    move('B', false, speed);
  } else {
    move('A', false, -speed);
    move('B', true, -speed);
  }
}

void turnRight() {
  mpu.update();
  float initMazeial = mpu.getAngleZ(), target = initMazeial + 90;
  previousError = 0;
  integral = 0;
  lastTime = millis();
  turning = true;
  while (true) {
    mpu.update();
    float error = target - mpu.getAngleZ();
    if (error < 9) break;
    integral += error;
    float derivative = error - previousError;
    float output = 0.5 * error + 0.01 * integral + 5 * derivative;
    previousError = error;
    rotateInPlace(-(30 - output));
  }
  motorStop('A');
  motorStop('B');
  turning = false;
}

void turnLeft() {
  mpu.update();
  float initMazeial = mpu.getAngleZ(), target = initMazeial - 90;
  previousError = 0;
  integral = 0;
  lastTime = millis();
  turning = true;
  while (true) {
    mpu.update();
    float error = target - mpu.getAngleZ();
    if (error > -9) break;
    integral += error;
    float derivative = error - previousError;
    float output = 0.5 * error + 0.01 * integral + 5 * derivative;
    previousError = error;
    rotateInPlace((30 + output));
  }
  motorStop('A');
  motorStop('B');
  turning = false;
}
