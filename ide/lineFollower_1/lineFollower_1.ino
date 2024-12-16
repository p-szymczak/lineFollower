#include <QTRSensors.h>

// zdefiniowanie pinow
#define NUM_SENSORS 8        // liczba pinow na czujniku
#define TIMEOUT 2500         // timeout dla czujnikow
#define EMITTER_PIN 2        //emmiter pin na czujniku

#define leftMotorPWM 5
#define leftMotorDir 6
#define rightMotorPWM 9
#define rightMotorDir 10

#define demoButtonPin 3      // przycisk demo
#define raceButtonPin 4      // przycisk race

// wspolczynniki PID do zmiany
float Kp = 0.0, Ki = 0.0, Kd = 0.0;
int lastError = 0;
int integral = 0;

// predkosc bazowa do zmiany
int baseSpeed = 150;

// zainicjowanie czujnikow

// Stany robota
bool inRaceMode = false;
bool inDemoMode = false;

void setup() {
  // silniki
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorDir, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorDir, OUTPUT);

  // przyciski
  pinMode(demoButtonPin);
  pinMode(raceButtonPin);

  // kalibracja czujnikow

}

void loop() {
  // czy wcisniety przycisk demo
  if (digitalRead(demoButtonPin) == LOW && !inDemoMode && !inRaceMode) {
    inDemoMode = true;
    demoMode();
  }

  // czy wcisniety przycisk race
  if (digitalRead(raceButtonPin) == LOW && !inRaceMode && !inDemoMode) {
    delay(3000);
    inRaceMode = true;
    raceMode();
  }
}

// tryb demo
void demoMode() {

}

// tryb race
void raceMode() {

}

// podazanie za linia
void followLine() {

}

// wykryj mete
bool isFinishLine() {

}

// sterowanie PID
int calculatePID() {

}

// zmiana predkosci silnikow
void setMotorSpeed(int leftSpeed, int rightSpeed) {

}

// zatrzymanie silnikow
void stopMotors() {
  setMotorSpeed(0, 0);
}