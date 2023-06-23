#include <Wire.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
Servo headServo; 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150 // 0 degree;
#define SERVOMAX 600 // 180 degree;
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

const int servoInitialPositions[16] = {90, 90, 90, 88, 90, 90, 90, 90, 90, 90, 90, 92, 90, 90, 90, 90};
int servoPosition[16] = {90, 90, 90, 88, 90, 90, 90, 90, 90, 90, 90, 92, 90, 90, 90, 90};

void setup() {
  Wire.begin();
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
  for(int i = 0; i < 16; i++) {
    setServo(i, servoInitialPositions[i]);
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

// ---------------- Walk function ----------------------
void walk() {

  int servoNo12[] = {2, 10, 13, 12, 5, 4};
  int servoPos12[] = {110, 110, 120, 110, 120, 110};
  increaseSync(servoNo12, servoPos12, 6);

  int servoNo13[] = {2, 10, 13, 12, 5, 4};
  int servoPos13[] = {70, 70, 60, 70, 60, 70};
  increaseSync(servoNo13, servoPos13, 6);

  ///...........................................................
  // int servoNo11[] = {7, 15};
  // int servoPos11[] = {100, 100};
  // increaseSync(servoNo11, servoPos11, 2);

  // int servoNo12[] = {2, 10, 13, 12, 7, 15};
  // int servoPos12[] = {110, 110, 120, 110, 90, 90};
  // increaseSync(servoNo12, servoPos12, 6);

  // int servoNo13[] = {6, 14, 5, 4};
  // int servoPos13[] = {80, 80, 120, 110};
  // increaseSync(servoNo13, servoPos13, 4);

  // int servoNo21[] = {7, 15};
  // int servoPos21[] = {80, 80};
  // increaseSync(servoNo21, servoPos21, 2);

  // int servoNo22[] = {2, 10, 5, 4, 7, 15};
  // int servoPos22[] = {70, 70, 60, 70, 90, 90};
  // increaseSync(servoNo22, servoPos22, 4);

  // int servoNo23[] = {6, 14, 13, 12};
  // int servoPos23[] = {90, 90, 60, 70};
  // increaseSync(servoNo23, servoPos23, 4);
}

void slideRight() {
  int servoNo1[] = {3, 11};
  int servoPos1[] = {80, 100};
  increaseSync(servoNo1, servoPos1, 2);

  int servoNo2[] = {3, 11};
  int servoPos2[] = {90, 90};
  increaseSync(servoNo2, servoPos2, 2);
}

// ------- Hand movement controle end ----------------
int calcPos(int deg) {
  return map(deg, 0, 180, SERVOMIN, SERVOMAX);
}

void increaseSync(int servo[], int deg[], int len) {
  int flag = 0;
  int count = 0;
  while(flag < len) {
    int prevPos = servoPosition[servo[count]];
    int newPos = deg[count];
    if(prevPos < newPos){
        int pos = prevPos + 1;
        pwm.setPWM(servo[count], 0, calcPos(pos));
        servoPosition[servo[count]] = pos;
    }else if(prevPos > newPos){
        int pos = prevPos - 1;
        pwm.setPWM(servo[count], 0, calcPos(pos));
        servoPosition[servo[count]] = pos;
    } else if (prevPos == newPos) {
        flag = flag + 1;
    } 
    delay(10);
    count++;

    if(count == len) {
      count = 0;
    }
  }
}

void loop() {
  walk();
  // slideRight();
}
