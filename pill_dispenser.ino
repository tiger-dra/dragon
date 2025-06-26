#include <Stepper.h>
#include "HX711.h"
#include <DFRobotDFPlayerMini.h>
#include <Wire.h>
#include "RTClib.h"

const int stepsPerRevolution = 2048;

// Stepper motor pin configuration
Stepper stepper1(stepsPerRevolution, 50, 51, 52, 53);
Stepper stepper2(stepsPerRevolution, 56, 57, 58, 59);
Stepper stepper3(stepsPerRevolution, 42, 43, 44, 45);
Stepper stepper4(stepsPerRevolution, 10, 11, 12, 13);
Stepper stepper5(stepsPerRevolution, 6, 7, 8, 9);

// IR sensor pins
const int irPins[5] = {33, 34, 35, 26, 27};

// HX711 weight sensor pins (DOUT, SCK)
const int hxDoutPins[5] = {40, 38, 36, 22, 24};
const int hxSckPins[5]  = {41, 39, 37, 23, 25};
HX711 scales[5];

// Stepper control variables
volatile int targetCount[5] = {0};
volatile int currentCount[5] = {0};
bool activeStepper[5] = {false};

unsigned long lastStepTime[5] = {0};
const unsigned long stepInterval = 3; // ms between steps

bool lastIrState[5] = {false};

// DFPlayer (Serial3: D14 TX3, D15 RX3)
DFRobotDFPlayerMini dfplayer;

// Nextion LCD (Serial1: D18 TX1, D19 RX1)
#define nexSerial Serial1

// Bluetooth module (Serial2: D16 TX2, D17 RX2)
#define btSerial Serial2

RTC_DS3231 rtc;

void setup() {
  Serial.begin(9600);      // USB debug
  nexSerial.begin(9600);   // Nextion LCD
  btSerial.begin(9600);    // Bluetooth
  Serial3.begin(9600);     // DFPlayer
  dfplayer.begin(Serial3);

  stepper1.setSpeed(5);
  stepper2.setSpeed(5);
  stepper3.setSpeed(5);
  stepper4.setSpeed(5);
  stepper5.setSpeed(5);

  for (int i = 0; i < 5; i++) {
    pinMode(irPins[i], INPUT);
    scales[i].begin(hxDoutPins[i], hxSckPins[i]);
    scales[i].set_scale();
    scales[i].tare();
  }

  if (!rtc.begin()) {
    Serial.println("RTC not found");
  }

  updateWeights();
}

void loop() {
  readBluetooth();
  runDispensers();
}

void readBluetooth() {
  while (btSerial.available()) {
    String cmd = btSerial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("S")) {
      int idx = cmd.substring(1, 2).toInt() - 1; // S1, S2 ...
      int cnt = cmd.substring(3).toInt();       // amount
      if (idx >= 0 && idx < 5) {
        targetCount[idx] = cnt;
      }
    } else if (cmd == "START") {
      for (int i = 0; i < 5; i++) {
        if (targetCount[i] > 0) {
          activeStepper[i] = true;
          currentCount[i] = 0;
        }
      }
    }
  }
}

void runDispensers() {
  unsigned long now = millis();

  if (activeStepper[0] && now - lastStepTime[0] >= stepInterval) {
    stepper1.step(1);
    lastStepTime[0] = now;
  }
  if (activeStepper[1] && now - lastStepTime[1] >= stepInterval) {
    stepper2.step(1);
    lastStepTime[1] = now;
  }
  if (activeStepper[2] && now - lastStepTime[2] >= stepInterval) {
    stepper3.step(1);
    lastStepTime[2] = now;
  }
  if (activeStepper[3] && now - lastStepTime[3] >= stepInterval) {
    stepper4.step(1);
    lastStepTime[3] = now;
  }
  if (activeStepper[4] && now - lastStepTime[4] >= stepInterval) {
    stepper5.step(1);
    lastStepTime[4] = now;
  }

  for (int i = 0; i < 5; i++) {
    bool state = digitalRead(irPins[i]) == LOW; // LOW when blocked
    if (state && !lastIrState[i]) {
      currentCount[i]++;
      if (currentCount[i] >= targetCount[i]) {
        activeStepper[i] = false;
        stopStepper(i);
        checkAllDone();
      }
    }
    lastIrState[i] = state;
  }
}

void stopStepper(int i) {
  switch (i) {
    case 0:
      digitalWrite(50, LOW); digitalWrite(51, LOW); digitalWrite(52, LOW); digitalWrite(53, LOW);
      break;
    case 1:
      digitalWrite(56, LOW); digitalWrite(57, LOW); digitalWrite(58, LOW); digitalWrite(59, LOW);
      break;
    case 2:
      digitalWrite(42, LOW); digitalWrite(43, LOW); digitalWrite(44, LOW); digitalWrite(45, LOW);
      break;
    case 3:
      digitalWrite(10, LOW); digitalWrite(11, LOW); digitalWrite(12, LOW); digitalWrite(13, LOW);
      break;
    case 4:
      digitalWrite(6, LOW); digitalWrite(7, LOW); digitalWrite(8, LOW); digitalWrite(9, LOW);
      break;
  }
}

void checkAllDone() {
  for (int i = 0; i < 5; i++) {
    if (activeStepper[i]) return;
  }
  dfplayer.play(1); // play file 0001.mp3 when all finished
  updateWeights();
}

void updateWeights() {
  for (int i = 0; i < 5; i++) {
    float w = scales[i].get_units(5);
    // Convert w to percent relative to max weight if needed
    // Example: send to Nextion "p1.txt=\"50%\""
    String cmd = "p" + String(i + 1) + ".txt=\"" + String(w, 1) + "%\"";
    nexSerial.print(cmd);
    nexSerial.write(0xff); nexSerial.write(0xff); nexSerial.write(0xff);
  }
}

