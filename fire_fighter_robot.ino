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

char ssid[] = "NO-INTERNET";
char pass[] = "#p@ssword535909#";

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

const int DHT_PIN = 46;
const int flameSensor = 47;
const int gasSensor = A15;
const int trigPin = 2;
const int echoPin = 3;
const int headServo = 29;

const int servoPin[] = {
  30, 32, 34, 36, 38, 40, 42, 44,
  31, 33, 35, 37, 39, 41, 43, 45
};

const int initialPosition[] = {
  90, 90, 90, 90, 90, 90, 85, 90, 
  90, 90, 90, 90, 90, 90, 95, 90
};

long duration;
int distance;
const int headInitialPosition = 82;
int headServoPosition = headInitialPosition;

int servoPositions[] = {
  90, 90, 90, 90, 90, 90, 85, 90, 
  90, 90, 90, 90, 90, 90, 95, 90
};

int numServos = sizeof(servos) / sizeof(servos[0]);

void setup() {
  Serial.begin(9600);
  EspSerial.begin(ESP8266_BAUD);
  Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass);

  pinMode(DHT_PIN, INPUT);
  pinMode(flameSensor, INPUT);
  pinMode(gasSensor, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(headServo, OUTPUT);

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

void updatePosition(int servoNo, int servoPosition) {
  servos[servoNo].write(servoPosition);
  servoPositions[servoNo] = servoPosition;
  delay(5);
}

void updateServoPosition(int updateServos[], int positions[], int numServos) {
  bool flag = true; // Flag to track if the array has been changed
  int i = 0;        // Loop variable to iterate through servos
  while (numServos > 0) {
    int prevPosition = servoPositions[updateServos[i]];  // Previous position of the servo
    int newPosition = positions[i];                      // New position for the servo
    int position = 0;                                    // Intermediate position for gradual update
    if (prevPosition < newPosition) {
      position = prevPosition + 1;                       // Increment position gradually
      updatePosition(updateServos[i], position);         // Update the servo position
    } else if (prevPosition > newPosition) {
      position = prevPosition - 1;                       // Decrement position gradually
      updatePosition(updateServos[i], position);         // Update the servo position
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
  updateServoPosition(updateServos1, newPositions1, 3);
}

void rightHandDown() {
  int updateServos2[] = {0, 1, 2};
  int newPositions2[] = {90, 90, 90};
  updateServoPosition(updateServos2, newPositions2, 3);
}

void rightHandUpSide() {
  int updateServos3[] = {0, 1, 2};
  int newPositions3[] = {90, 180, 90};
  updateServoPosition(updateServos3, newPositions3, 3);
}

// ---------- Left Hand -------------------
void leftHandUpForward() {
  int updateServos[] = {8, 9, 10};
  int newPositions[] = {90, 90, 0};
  updateServoPosition(updateServos, newPositions, 3);
}

void leftHandDown() {
  int updateServos[] = {8, 9, 10};
  int newPositions[] = {90, 90, 90};
  updateServoPosition(updateServos, newPositions, 3);
}

void leftHandUpSide() {
  int updateServos[] = {8, 9, 10};
  int newPositions[] = {90, 0, 90};
  updateServoPosition(updateServos, newPositions, 3);
}

// ---------- Both Hand -------------------
void bothHandUpForward() {
  int updateServos[] = {0, 1, 8, 9, 2, 10};
  int newPositions[] = {90, 90, 90, 90, 180, 0};
  updateServoPosition(updateServos, newPositions, 6);
}

void bothHandUpSide() {
  int updateServos[] = {0, 2, 8, 10, 1, 9};
  int newPositions[] = {90, 90, 90, 90, 180, 0};
  updateServoPosition(updateServos, newPositions, 6);
}

void bothHandDown() {
  int updateServos[] = {0, 2, 8, 10, 1, 9};
  int newPositions[] = {90, 90, 90, 90, 90, 90};
  updateServoPosition(updateServos, newPositions, 6);
}

void walk() {
  int servoNo12[] = {2, 10, 13, 12, 5, 4};
  int servoPos12[] = {110, 110, 120, 120, 120, 120};
  updateServoPosition(servoNo12, servoPos12, 6);

  int servoNo13[] = {2, 10, 5, 4, 13, 12};
  int servoPos13[] = {70, 70, 60, 60, 60, 60};
  updateServoPosition(servoNo13, servoPos13, 6);
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

// -------------- Fire detection ----------------

unsigned long startTime = millis();
bool onWalkMode = false, onFireDetectionMode = false, onObstacleDetectionMode = false;

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
  
}

// Controll left hand
BLYNK_WRITE(V2) {
  int value = param.asInt(); // Get value from app widget
  switch (value) {
    case 1:
      leftHandDown();
      break;
    case 2:
      leftHandUpForward();
      break;
    default:
      leftHandUpSide();
  }
}

// Controll right hand
BLYNK_WRITE(V3) {
  int value = param.asInt(); // Get value from app widget
  switch (value) {
    case 1:
      rightHandDown();
      break;
    case 2:
      rightHandUpForward();
      break;
    default:
      rightHandUpSide();
  }
}

// Controll both hand
BLYNK_WRITE(V5) {
  int value = param.asInt(); // Get value from app widget
  switch (value) {
    case 1:
      bothHandDown();
      break;
    case 2:
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
    onWalkMode = true;
  } else {
    onWalkMode = false;
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
