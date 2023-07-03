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
const int trigPin = 49;
const int echoPin = 50;

const int servoPin[] = {
  30, 32, 34, 36, 38, 40, 42, 44,
  31, 33, 35, 37, 39, 41, 43, 45
};

const int initialPosition[] = {
  90, 90, 90, 90, 90, 90, 85, 90, 
  90, 90, 90, 90, 90, 90, 95, 90
};

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

void updateData(int servoNo, int servoPosition) {
  servos[servoNo].write(servoPosition);
  servoPositions[servoNo] = servoPosition;
  delay(5);
}

void updateServoPosition(int updateServos[], int positions[], int numServos) {
  int flag = 0;
  int i = 0;
  while (flag < numServos) {
    int prevPosition = servoPositions[updateServos[i]];
    int newPosition = positions[i];
    Serial.print(prevPosition);
    Serial.print("-");
    Serial.println(newPosition);
    if (prevPosition < newPosition) {
      int position = prevPosition + 1;
      updateData(updateServos[i], position);
    } else if (prevPosition > newPosition) {
      int position = prevPosition - 1;
      updateData(updateServos[i], position);
    } else if (prevPosition == newPosition) {
      flag = flag + 1;
      Serial.println(flag);
    }

    if (i == numServos - 1) {
      i = 0;
    } else {
      i++;
    }
  }
}

void rightHandUpForward() {
  Serial.println("rightHandUpForward");
  int updateServos[] = {0, 1, 2};
  int newPositions[] = {90, 90, 180};
  updateServoPosition(updateServos, newPositions, 3);
}

void rightHandDown() {
  Serial.println("rightHandDown");
  int updateServos[] = {0, 1, 2};
  int newPositions[] = {90, 90, 90};
  updateServoPosition(updateServos, newPositions, 3);
}

void rightHandUpSide() {
  Serial.println("rightHandUpSide");
  int updateServos[] = {0, 1, 2};
  int newPositions[] = {90, 180, 90};
  updateServoPosition(updateServos, newPositions, 3);
}


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
  int servoPos12[] = {110, 110, 120, 110, 120, 110};
  updateServoPosition(servoNo12, servoPos12, 6);

  int servoNo13[] = {2, 10, 5, 4, 13, 12};
  int servoPos13[] = {70, 70, 60, 70, 60, 70};
  updateServoPosition(servoNo13, servoPos13, 6);
}

// DHT sensor 
void checkTemperatureAndHumidity() {
  int data = DHT.read11(DHT_PIN);
  float teperature = DHT.temperature;
  float humidity = DHT.humidity;
  Blynk.virtualWrite(vTemperature, teperature);
  Blynk.virtualWrite(vHumidity, humidity);
}

void checkAirQuality() {
  int air_quality = analogRead(gasSensor);
  Blynk.virtualWrite(vAirQuality, air_quality);
}


unsigned long startTime = millis();
bool callRightHandUpForward = false, callRightHandUpSide = false, callRightHandDown = false;
bool callLeftHandUpForward = false, callLeftHandUpSide = false, callLeftHandDown = false;
bool callBothHandUpForward = false, callBothHandUpSide = false, callBothHandDown = false;

void loop() {
  Blynk.run();

  unsigned long presentTime = millis();
  int interval = presentTime - startTime;
  if(interval > 5000){
    startTime = millis();
    checkTemperatureAndHumidity();
    checkAirQuality();
  }

  // if(callRightHandUpForward) {
  //   rightHandUpForward();
  // }else if(callRightHandUpSide) {
  //   rightHandUpSide();
  // }else if(callRightHandDown) {
  //   rightHandDown();
  // }

  // if(callLeftHandUpForward) {
  //   leftHandUpForward();
  // }else if(callLeftHandUpSide) {
  //   leftHandUpSide();
  // }else if(callLeftHandDown) {
  //   leftHandDown();
  // }

  // if(callBothHandUpForward) {
  //   bothHandUpForward();
  // }else if(callBothHandUpSide) {
  //   bothHandUpSide();
  // }else if(callBothHandDown) {
  //   bothHandDown();
  // }

  // rightHandUpForward();
  // rightHandDown();
  // rightHandUpSide();

  // leftHandUpForward();
  // leftHandDown();
  // leftHandUpSide();

  // bothHandUpSide();
  // bothHandUpForward();

  // walk();
}

// left hand
BLYNK_WRITE(V2) {
  int value = param.asInt(); // Get value from app widget
  switch (value) {
    case 1:
      callLeftHandUpForward = false;
      callLeftHandUpSide = false;
      callLeftHandDown = true;
      break;
    case 2:
      callLeftHandUpForward = true;
      callLeftHandUpSide = false;
      callLeftHandDown = false;
      break;
    case 3:
      callLeftHandUpForward = false;
      callLeftHandUpSide = true;
      callLeftHandDown = false;
      break;
    default:
      callLeftHandUpForward = false;
      callLeftHandUpSide = false;
      callLeftHandDown = true;
  }

}

// right hand
BLYNK_WRITE(V3) {
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
    default:
      rightHandDown();
  }
}

// both hand
BLYNK_WRITE(V5) {
  int value = param.asInt(); // Get value from app widget
  switch (value) {
    case 1:
      callBothHandUpForward = false;
      callBothHandUpSide = false;
      callBothHandDown = true;
      break;
    case 2:
      callBothHandUpForward = true;
      callBothHandUpSide = false;
      callBothHandDown = false;
      break;
    case 3:
      callBothHandUpForward = false;
      callBothHandUpSide = true;
      callBothHandDown = false;
      break;
    default:
      callBothHandUpForward = false;
      callBothHandUpSide = false;
      callBothHandDown = true;
  }
}
