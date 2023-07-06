#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL6Pr71HVWo"
#define BLYNK_TEMPLATE_NAME "robo"
#define BLYNK_AUTH_TOKEN "CZTEmHRlwlTJqspdpj1XxAPkDms8WyiA"

#include <Servo.h>
#include<ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include<SoftwareSerial.h>
#include<dht.h>

#define EspSerial Serial1
#define ESP8266_BAUD 9600
ESP8266 wifi(&EspSerial);

char ssid[] = "NO-INTERNET"; // test
char pass[] = "#p@ssword535909#"; // 200312003

Servo RH1;
Servo RH2;
Servo RS;
Servo RH;
Servo RL1;
Servo RL2;
Servo RL3;
Servo RL4;
Servo LH1;
Servo LH2;
Servo LS;
Servo LH;
Servo LL1;
Servo LL2;
Servo LL3;
Servo LL4;
Servo Head;

dht DHT;

Servo servos[] = {
  RH1, RH2, RS, RH, RL1, RL2, RL3, RL4,
  LH1, LH2, LS, LH, LL1, LL2, LL3, LL4
};

// define blynk virtual pin
#define vTemperature V0
#define vHumidity V1
#define vAirQuality V4
#define vFlame V9

#define DHT_PIN  46
#define flameSensorDI  47
#define flameSensorAI  A14
#define gasSensor  A15
#define trigPin  2
#define echoPin  3
#define pir 48
#define pumpRelay 49
#define headServo  50

const int dly5 = 5;
const int dly10 = 10;
const int dly15 = 15;
const int dly20 = 20;

const int servoPin[] = {
  30, 32, 34, 36, 38, 40, 42, 44,
  31, 33, 35, 37, 39, 41, 43, 45
};

const int initialPosition[] = {
  90, 90, 90, 88, 90, 90, 85, 90, 
  90, 90, 90, 92, 90, 90, 95, 90
};

const int initialPosition2[] = {
  90, 90, 90, 88, 90, 65, 65, 90, 
  90, 90, 90, 92, 90, 115, 115, 90
};

long duration;
int distance;
const int headInitialPosition = 82;
int headServoPosition = headInitialPosition;
int state = LOW; 

bool onWalkMode = false, onFireDetectionMode = false, onObstacleDetectionMode = false, onMotionDetectionMode = false;

const int flameMinDistance = 0;
const int flameMaxDistance = 16;

int servoPositions[] = {
  90, 90, 90, 88, 90, 90, 85, 90, 
  90, 90, 90, 92, 90, 90, 95, 90
};

int numServos = sizeof(servos) / sizeof(servos[0]);

void setup() {
  Serial.begin(9600);
  EspSerial.begin(ESP8266_BAUD);
  Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass);

  pinMode(DHT_PIN, INPUT);
  pinMode(flameSensorDI, INPUT);
  pinMode(flameSensorAI, INPUT);
  pinMode(gasSensor, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(headServo, OUTPUT);
  pinMode(pumpRelay, OUTPUT);
  digitalWrite(pumpRelay, HIGH);

  Head.attach(headServo);
  Head.write(headInitialPosition);
  for (int i = 0; i < numServos; i++) {
    servos[i].attach(servoPin[i]);
  }
  setServoToInitialPosition();
  delay(2000);
}

void setServoToInitialPosition(){
  for (int i = 0; i < numServos; i++) {
    servos[i].write(initialPosition[i]);
    delay(5);
  }
}

void updatePosition(int servoNo, int servoPosition, int dly) {
  servos[servoNo].write(servoPosition);
  servoPositions[servoNo] = servoPosition;
  delay(dly);
}

void updateServoPosition(int updateServos[], int positions[], int numServos,  int dly) {
  bool flag = true; // Flag to track if the array has been changed
  int i = 0;        // Loop variable to iterate through servos
  while (numServos > 0) {
    int prevPosition = servoPositions[updateServos[i]];  // Previous position of the servo
    int newPosition = positions[i];                      // New position for the servo
    int position = 0;                                    // Intermediate position for gradual update
    if (prevPosition < newPosition) {
      position = prevPosition + 1;                       // Increment position gradually
      updatePosition(updateServos[i], position, dly);         // Update the servo position
    } else if (prevPosition > newPosition) {
      position = prevPosition - 1;                       // Decrement position gradually
      updatePosition(updateServos[i], position, dly);         // Update the servo position
    } else {
      // If the servo position is already equal to the new position, remove it from the array
      for(int j = i; j < numServos - 1; j++) {
        updateServos[j] = updateServos[j+1];
        positions[j] = positions[j+1];
      }
      numServos--;
      flag = false;  // Array has been changed
    }

    // If the array has not been changed, increment the loop variable
    if(flag){
      i++;
    } else {
      flag = true;  // Reset the flag for the next iteration
    }
    
    // If the loop variable reaches the last servo, reset it to 0 for circular iteration
    if (i == numServos) {
      i = 0;
    } 
  }
}

// ---------- Right Hand -------------------
void rightHandUpForward() {
  int updateServos1[] = {0, 1, 2};
  int newPositions1[] = {90, 90, 180};
  updateServoPosition(updateServos1, newPositions1, 3, dly10);
}

void rightHandDown() {
  int updateServos2[] = {0, 1, 2};
  int newPositions2[] = {90, 90, 90};
  updateServoPosition(updateServos2, newPositions2, 3, dly10);
}

void rightHandUpSide() {
  int updateServos3[] = {0, 1, 2};
  int newPositions3[] = {90, 180, 90};
  updateServoPosition(updateServos3, newPositions3, 3, dly10);
}

// ---------- Left Hand -------------------
void leftHandUpForward() {
  int updateServos[] = {8, 9, 10};
  int newPositions[] = {90, 90, 0};
  updateServoPosition(updateServos, newPositions, 3, dly5);
}

void leftHandDown() {
  int updateServos[] = {8, 9, 10};
  int newPositions[] = {90, 90, 90};
  updateServoPosition(updateServos, newPositions, 3, dly5);
}

void leftHandUpSide() {
  int updateServos[] = {8, 9, 10};
  int newPositions[] = {90, 0, 90};
  updateServoPosition(updateServos, newPositions, 3, dly5);
}

// ---------- Both Hand -------------------
void bothHandUpForward() {
  int updateServos[] = {0, 1, 8, 9, 2, 10};
  int newPositions[] = {90, 90, 90, 90, 180, 0};
  updateServoPosition(updateServos, newPositions, 6, dly5);
}

void bothHandUpSide() {
  int updateServos[] = {0, 2, 8, 10, 1, 9};
  int newPositions[] = {90, 90, 90, 90, 180, 0};
  updateServoPosition(updateServos, newPositions, 6, dly5);
}

void bothHandDown() {
  int updateServos[] = {0, 1, 8, 9, 2, 10};
  int newPositions[] = {90, 90, 90, 90, 90, 90};
  updateServoPosition(updateServos, newPositions, 6, dly5);
}

// void walk() {
  //   int servoNo11[] = {7, 15};
  // int servoPos11[] = {100, 90};
  // updateServoPosition(servoNo11, servoPos11, 2, 7);

  // int servoNo12[] = {13, 12, 5, 4, 2, 10};
  // int servoPos12[] = {105, 105, 105, 105, 100, 100};
  // updateServoPosition(servoNo12, servoPos12, 6, 2);

  // int servoNo31[] = {15, 7};
  // int servoPos31[] = {80, 90};
  // updateServoPosition(servoNo31, servoPos31, 2, 7);

  // int servoNo13[] = {5, 4, 13, 12, 2, 10};
  // int servoPos13[] = {75, 75, 75, 75, 80, 80};
  // updateServoPosition(servoNo13, servoPos13, 6, 2);
// }

void walk() {
  int servoNo11[] = {7, 15};
  int servoPos11[] = {95, 95};
  updateServoPosition(servoNo11, servoPos11, 2, 10);

  int servoNo12[] = {13, 12, 14, 2};
  int servoPos12[] = {107, 107, 100, 100};
  updateServoPosition(servoNo12, servoPos12, 4, 2);

  int servoNo13[] = {5, 4, 6, 10};
  int servoPos13[] = {107, 107, 86, 100};
  updateServoPosition(servoNo13, servoPos13, 4, 2);

  int servoNo31[] = {15, 7};
  int servoPos31[] = {85, 85};
  updateServoPosition(servoNo31, servoPos31, 2, 10);

  int servoNo42[] = {5, 4, 6, 10};
  int servoPos42[] = {73, 73, 80, 80};
  updateServoPosition(servoNo42, servoPos42, 4, 2);

  int servoNo43[] = {13, 12, 14, 2};
  int servoPos43[] = {73, 73, 94, 80};
  updateServoPosition(servoNo43, servoPos43, 4, 2);
}

void stand() {
  int servoNo[] = {2, 10, 5, 4, 13, 12, 7, 15, 6, 14};
  int servoPos[] = {90, 90, 90, 90, 90, 90, 90, 90, 85, 95};
  updateServoPosition(servoNo, servoPos, 10, dly10);
} 

void blanceBodyPosition() {
  int servoNo[] = {10};
  int servoPos[] = {140};
  updateServoPosition(servoNo, servoPos, 1, dly10);
}

// --------- Temperature and Humidity detection -------------- 
void checkTemperatureAndHumidity() {
  int data = DHT.read11(DHT_PIN);
  float teperature = DHT.temperature;
  float humidity = DHT.humidity;
  Blynk.virtualWrite(vTemperature, teperature);
  Blynk.virtualWrite(vHumidity, humidity);
}

// ----------- Air quality detection -------------
void checkAirQuality() {
  int air_quality = analogRead(gasSensor);
  Blynk.virtualWrite(vAirQuality, air_quality);
}

// ------------ Obstacle detection -----------
void calculateDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.0133 / 2;
}

void rotateServo(int newPosition) {
  if(newPosition > headServoPosition) {
    for(;headServoPosition <= newPosition; headServoPosition++) {
      Head.write(headServoPosition);
      delay(10);
    }
  }else {
    for(; headServoPosition >= newPosition; headServoPosition--) {
      Head.write(headServoPosition);
      delay(10);
    }
  }
  delay(200);
}

int lookRight() {
  rotateServo(10);
  calculateDistance();
  delay(50);
  return distance;
}

int lookLeft() {
  rotateServo(170);
  calculateDistance();
  delay(50);
  return distance;
}

void checkObstacle() {
  calculateDistance();
  if(distance < 12) {
    Serial.println(distance);
    Blynk.logEvent("obstacle_alert");
    int rightDistance = lookRight();
    rotateServo(headInitialPosition);
    int leftDistance = lookLeft();
    if(rightDistance > leftDistance) {
      Serial.println("Moving toward right");
    }else{
      Serial.println("Moving toward left");
    }
    rotateServo(headInitialPosition);
  }
}

// ------------ Flame detection -----------------
void detectFlame() {
  if(digitalRead(flameSensorDI) == 1) {
    // int data = analogRead(flameSensorAI);
    // int flameDistance = map(data, 0, 600, flameMinDistance, flameMaxDistance);
    Blynk.logEvent("fire_alert");
    rightHandUpForward();
    blanceBodyPosition();
    delay(200);
    int updateN0[] = {1};
    int startTime = millis();
    int runTime = 5000;
    int presentTime = millis();
    digitalWrite(pumpRelay, LOW);
    while(presentTime - startTime < runTime){
      int newPositions1[] = {60};
      updateServoPosition(updateN0, newPositions1, 1, 15);
      int newPositions2[] = {110};
      updateServoPosition(updateN0, newPositions2, 1, 15);
      presentTime = millis();
    }
    digitalWrite(pumpRelay, HIGH);
    int newPositions3[] = {90};
    updateServoPosition(updateN0, newPositions3, 1, 15);
    rightHandDown();
    stand();
    detectFlame();
  }
}

// -------------- Motion detection -----------------
void detectMotion() {
  int val = digitalRead(pir);   // read sensor value
  if (val == HIGH) {    
    if (state == LOW) {
      Serial.println("Motion Detected");
      Blynk.logEvent("motion_detection_alert");     
      state = HIGH;       // update variable state to HIGH
    }
  } 
  else {
      if (state == HIGH){
        Serial.println("Motion stopped!");
        state = LOW;       // update variable state to LOW
    }
  }
}

// Controll Hand
BLYNK_WRITE(V2) {
  int value = param.asInt(); // Get value from app widget
  switch (value) {
    case 1:
      rightHandDown();
      break;
    case 2:
      rightHandUpForward();
      break;
    case 3:
      rightHandUpSide();
      break;
    case 4:
      leftHandDown();
      break;
    case 5:
      leftHandUpForward();
      break;
    case 6:
      leftHandUpSide();
      break;
    case 7:
      bothHandDown();
      break;
    case 8:
      bothHandUpForward();
      break;
    default:
      bothHandUpSide();
  }
}

// Controll walk mode
BLYNK_WRITE(V6) {
  int value = param.asInt(); // Get value from app widget
  if(value == 1) {
    stand();
    int servoNo[] = {6, 14};
    int servoPosition[] = {83, 97};
    updateServoPosition(servoNo, servoPosition, 2, dly5);
    onWalkMode = true;
  } else {
    onWalkMode = false;
    stand();
  }
}

// Controll obstacle detection
BLYNK_WRITE(V7) {
  int value = param.asInt(); // Get value from app widget
  if(value == 1) {
    onObstacleDetectionMode = true;
  } else {
    onObstacleDetectionMode = false;
  }
}

// Controll fire detection
BLYNK_WRITE(V8) {
  int value = param.asInt(); // Get value from app widget
  if(value == 1) {
    onFireDetectionMode = true;
  } else {
    onFireDetectionMode = false;
  }
}

// Controll motion detection
BLYNK_WRITE(V3) {
  int value = param.asInt(); // Get value from app widget
  if(value == 1) {
    onMotionDetectionMode = true;
  } else {
    onMotionDetectionMode = false;
  }
}

unsigned long startTime = millis();

void loop() {
  Blynk.run();

  // Check and update the Temperature, Humidity and Air quality data in every 5 seconds
  unsigned long presentTime = millis();
  int interval = presentTime - startTime;
  if(interval > 5000){
    startTime = millis();
    checkTemperatureAndHumidity();
    checkAirQuality();
  }

  if(onWalkMode) {
    walk();
  }

  if(onObstacleDetectionMode) {
    checkObstacle();
  }
  
  if(onFireDetectionMode) {
    detectFlame();
  }

  if(onMotionDetectionMode) {
    detectMotion();
  }
}
