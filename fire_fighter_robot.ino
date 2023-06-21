#include <Wire.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
Servo headServo; 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 102 // 0 degree;
#define SERVOMAX 512 // 180 degree;
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz

#define headServoPin 9
#define flameSensor 7
#define pumpRelay 8 
#define pirSensor 12
#define trigPin 5
#define echoPin 6

long duration;
int distance;
int headServoPosition = 90;
bool motionState = false;

int servoInitialPositions[16] = {90, 90, 90, 90, 90, 100, 60, 90, 90, 90, 90, 90, 90, 80, 120, 90};
// {90, 90, 90, 90, 90, 100, 60, 90, 90, 90, 90, 90, 90, 80, 120, 90}; initial position

void setup() {
  Serial.begin(9600);
  pinMode(flameSensor, INPUT); // flame sensor DO pin connect as INPUT pin
  pinMode(pirSensor, INPUT); // PIR sensor OUT pin connect as INPUT pin
  pinMode(pumpRelay, OUTPUT); // relay IN pin connect as OUTPUT pin
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  initialPosition();

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  headServo.attach(headServoPin);  // attaches the servo on pin 9 to the servo object
  headServo.write(headServoPosition);

  delay(3000);
}

void setServo(int servo, int angle) {
  int duty;
  duty = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo, 0, duty);
  delay(500);
}

void initialPosition() {
  int positions[16] = {90, 90, 90, 90, 90, 100, 60, 90, 90, 90, 90, 90, 90, 80, 120, 90};
  for(int i = 0; i < 16; i++) {
    setServo(i, positions[i]);
  }
}

void setPosition(int servoNo[], int deg[], int length) {
   for(int i = 0; i < length; i++) {
    setServo(servoNo[i], deg[i]);
  }
}

// ------ Obstical detection start --------------------------

void clearUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
}

void rotateServo(int newPosition) {
  if(newPosition > headServoPosition) {
    for(;headServoPosition <= newPosition; headServoPosition++) {
      headServo.write(headServoPosition);
      delay(10);
    }
  }else {
    for(; headServoPosition >= newPosition; headServoPosition--) {
      headServo.write(headServoPosition);
      delay(10);
    }
  }
  delay(200);
}

int lookRight() {
  rotateServo(10);
  checkObstacle();
  clearUltrasonic();
  delay(50);
  return distance;
}

int lookLeft() {
  rotateServo(170);
  checkObstacle();
  clearUltrasonic();
  delay(50);
  return distance;
}

void checkObstacle() {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration*0.034/2;

  if(distance < 30) {
    Serial.println("Object Detected");
    int rightDistance = lookRight();
    rotateServo(90);
    int leftDistance = lookLeft();
    if(rightDistance > leftDistance) {
      Serial.println("Moving toward right");
    }else if(rightDistance < leftDistance){
      Serial.println("Moving toward left");
    }else {
      Serial.println("Moving toward right");
    }
    rotateServo(90);
  }
}

// --------------- Obstical detection end -----------------------

void setServoPosition(int servo, int newAngle) {
  int servoPositions[16] = {90, 65, 130, 70, 90, 115, 60, 110, 90, 90, 90, 90, 90, 90, 90, 90};
  int newDuty;
  newDuty = map(newAngle, 0, 180, SERVOMIN, SERVOMAX);
  int oldDuty;
  oldDuty = map(servoPositions[servo], 0, 180, SERVOMIN, SERVOMAX);
  if(oldDuty > newDuty) {
    Serial.println(servoPositions[servo]);
    for(int i = oldDuty; i >= newDuty; i--){
      pwm.setPWM(servo, 0, i);
      delay(20);
    }
  }
  
  if(oldDuty < newDuty) {
    for(int i = oldDuty; i <= newDuty; i++){
      pwm.setPWM(servo, 0, i);
      delay(20);
    }
  }

  servoPositions[servo] = newAngle;
}

void runWaterPump() {
  digitalWrite(pumpRelay, HIGH); // run water pump
  delay(5000);
  digitalWrite(pumpRelay, LOW); // off water pump
  searchForFlame();
}

void searchForFlame() {
  if(digitalRead(flameSensor) == 0){
    Serial.println("Flame detected.....");
    runWaterPump();
  }
}

void checkMovement() {
  if(digitalRead(pirSensor) == HIGH) {
      Serial.println("Motion detected");
  }
}

// ------- Hand movement controle start ----------------

// --------- Right Hand -------------

void rightHandUpForward() {
  int servoNo[] = {0, 1, 2};
  int deg[] = {90, 90, 180};
  setPosition(servoNo, deg, 3);
}

void rightHandDown() {
  int servoNo[] = {0, 1, 2};
  int deg[] = {90, 90, 90};
  setPosition(servoNo, deg, 3);
}

void rightHandUpSide() {
  int servoNo[] = {0, 1, 2};
  int deg[] = {90, 180, 90};
  setPosition(servoNo, deg, 3);
}

void rightHandUpMoveLeftRight() {
  rightHandUpForward();
  int startTime = millis();
  int currentTime = millis();
  int duration = 10000;
  int servoNo = 1;
  int maxDeg = map(100, 0, 180, SERVOMIN, SERVOMAX);
  int minDeg = map(60, 0, 180, SERVOMIN, SERVOMAX);
  while(currentTime - startTime < duration) {
    for(int i = minDeg; i <= maxDeg; i++) {
      pwm.setPWM(servoNo, 0, i);
      delay(30);
    }
    for(int j = maxDeg; j >= minDeg; j--) {
      pwm.setPWM(servoNo, 0, j);
      delay(30);
    }
    currentTime = millis();
  }
  rightHandDown();
}


// ----------- Left hand ------------

void leftHandUpForward() {
  int servoNo[] = {8, 9, 10};
  int deg[] = {90, 90, 0};
  setPosition(servoNo, deg, 3);
}

void leftHandDown() {
  int servoNo[] = {8, 9, 10};
  int deg[] = {90, 90, 90};
  setPosition(servoNo, deg, 3);
}

void leftHandUpSide() {
  int servoNo[] = {8, 9, 10};
  int deg[] = {90, 0, 90};
  setPosition(servoNo, deg, 3);
}

// ----------- Both hand -------------

void bothHandUpForward() {
  int servoNo[] = {0, 8, 1, 9, 2, 10};
  int deg[] = {90, 90, 90, 90, 180, 0};
  setPosition(servoNo, deg, 6);
}

void bothHandUpSide() {
  int servoNo[] = {0, 8, 1, 9, 2, 10};
  int deg[] = {90, 90, 180, 0, 90, 90};
  setPosition(servoNo, deg, 6);
}

void bothHandDown() {
  int servoNo[] = {0, 8, 1, 9, 2, 10};
  int deg[] = {90, 90, 90, 90, 90, 90};
  setPosition(servoNo, deg, 6);
}

void moveHandOnWalk() {
  int servoNo[] = {0, 8, 1, 9, 2, 10};
  int deg[] = {80, 100, 90, 90, 90, 90};
  setPosition(servoNo, deg, 6);
  int startTime = millis();
  int currentTime = millis();
  int duration = 20000;
  int rightSholder = 2;
  int leftSholder = 10;
  int maxDeg = map(120, 0, 180, SERVOMIN, SERVOMAX);
  int minDeg = map(60, 0, 180, SERVOMIN, SERVOMAX);
  while(currentTime - startTime < duration) {
      for(int i = minDeg; i < maxDeg; i++) {
        pwm.setPWM(rightSholder, 0, i);
        pwm.setPWM(leftSholder, 0, i);
        delay(30);
      }
      for(int j = maxDeg; j > minDeg; j--) {
        pwm.setPWM(rightSholder, 0, j);
        pwm.setPWM(leftSholder, 0, j);
        delay(30);
      }
      currentTime = millis();
    }
}

void balanceRight() {
  int maxDeg = map(100, 0, 180, SERVOMIN, SERVOMAX);
  int minDeg = map(90, 0, 180, SERVOMIN, SERVOMAX);
  int servoNo = 7;
  for(int i = minDeg; i <= maxDeg; i++) {
    pwm.setPWM(servoNo, 0, i);
    delay(30);
  }
  delay(3000);
  // for(int j = maxDeg; j >= minDeg; j--) {
  //   pwm.setPWM(servoNo, 0, j);
  //   delay(30);
  // }
}

// servoInitialPositions[16] = {90, 90, 90, 90, 90, 100, 60, 90, 90, 90, 90, 90, 90, 80, 120, 90};
int servoPosition[16] = {90, 90, 90, 90, 90, 100, 60, 90, 90, 90, 90, 90, 90, 80, 120, 90};

void walk(int servoList[], int servoPos[], int length, int dly) {
  for(int i = 0; i < length; i++) {
    int prevPos = map(servoPosition[servoList[i]], 0, 180, SERVOMIN, SERVOMAX);
    int newPos = map(servoPos[i], 0, 180, SERVOMIN, SERVOMAX);
    if(prevPos < newPos) {
      for(int j = prevPos; j < newPos; j++) {
        pwm.setPWM(servoList[i], 0, j);
        delay(dly);
      }
      servoPosition[servoList[i]] = servoPos[i];
    }
    if(prevPos > newPos) {
      for(int k = prevPos; k > newPos; k--) {
        pwm.setPWM(servoList[i], 0, k);
        delay(dly);
      }
      servoPosition[servoList[i]] = servoPos[i];
    }
  }
}


// ------- Hand movement controle end ----------------
// {90, 90, 90, 90, 90, 100, 60, 90, 90, 90, 90, 90, 90, 80, 120, 90}
// int servoList[16];
// int servoDegAngle[16];
int len;
int dly = 15;

void step1() {
  // ------- step 1 --------------
  Serial.println("Step 1");
  int servoList[] = {15, 7, 3, 11, 2, 10, 12, 13, 14};
  int servoDegAngle[] = {100, 100, 100, 90, 120, 120, 120, 110, 130};
  len = 6;
  walk(servoList, servoDegAngle, len, dly);
}

void step2() {
  // ------- step 2 --------------
  Serial.println("Step 2");
  int servoList2[] = {7, 4, 6, 14, 13};
  int servoDegAngle2[] = {90, 100, 50, 120, 90};
  walk(servoList2, servoDegAngle2, len, dly);
}

void step3() {
  // ------- step 3 --------------
  Serial.println("Step 3");
  int servoList3[] = {12, 15, 2, 10, 4, 5};
  int servoDegAngle3[] = {90, 75, 70, 70, 70, 70};
  len = 4;
  walk(servoList3, servoDegAngle3, len, dly);
}

void step4() {
  // ------- step 4 --------------
  Serial.println("Step 4");
  int servoList4[] = {4, 5, 6, 15, 12, 13};
  int servoDegAngle4[] = {90, 100, 60, 90, 70, 70};
  len = 6;
  walk(servoList4, servoDegAngle4, len, dly);
}

void loop() {
  step1();
  // step2();
  // step3();
  // step4();

  // int servoList5[] = {12, 15, 4, 5, 6};
  // int servoDegAngle5[] = {90, 80, 70, 70, 50};
  // len = 5;
  // walk(servoList4, servoDegAngle4, len, dly);


  // pwm.setPWM(12, 0, map(70, 0, 180, SERVOMIN, SERVOMAX));
  // pwm.setPWM(13, 0, map(70, 0, 180, SERVOMIN, SERVOMAX));
  // pwm.setPWM(14, 0, map(120, 0, 180, SERVOMIN, SERVOMAX));
}
