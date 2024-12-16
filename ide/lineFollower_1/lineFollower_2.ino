#include <QTRSensors.h>

// zdefiniowanie pinow
#define TIMEOUT 2500
#define EMITTER_PIN 2

#define leftMotorPWM 5
#define leftMotorDir 6
#define rightMotorPWM 9
#define rightMotorDir 10

#define demoButtonPin 3
#define raceButtonPin 4

// wspolczynniki PID do zmiany
float Kp = 1.2, Kd = 0.5, Ki = 0.05;
int lastError = 0;
int integral = 0;

// predkosc bazowa do zmiany
int baseSpeed = 150;

// zainicjowanie czujnikow
QTRSensors qtr;
const uint8_t SensorCount = 8; //liczba czujnikow
uint16_t sensorValues[SensorCount]; //macierz z wartosciami czujnikow

//tryby
enum Mode { STANDBY, DEMO, RACE };
Mode currentMode = STANDBY;

void setup() {
  // piny silnikow
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorDir, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorDir, OUTPUT);

  // piny przyciskow
  pinMode(demoButtonPin, INPUT_PULLUP);
  pinMode(raceButtonPin, INPUT_PULLUP);

  Serial.begin(9600);

  qtr.setTypeRC(); //listwa cyfrowa
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, 12, 13}, SensorCount); //ustawienie pinow czujnikow
  qtr.setEmitterPin(2); //pin do diody czujnikow

  // kalibracja automatyczna
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }


void loop() {
    if (digitalRead(demoButtonPin) == LOW && currentMode == STANDBY) {
        currentMode = DEMO;
        demoMode();
        currentMode = STANDBY;
    }

    if (digitalRead(raceButtonPin) == LOW && currentMode == STANDBY) {
        countdownToRace();
        currentMode = RACE;
        raceMode();
        currentMode = STANDBY;
    }
}
