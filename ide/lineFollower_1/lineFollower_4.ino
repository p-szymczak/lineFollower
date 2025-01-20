//#include <QTRSensors.h>

// zdefiniowanie pinow
#define TIMEOUT 2500
#define EMITTER_PIN 2

#define leftMotorPWM 6
#define leftMotorDirA 7
#define leftMotorDirB 5
#define rightMotorPWM 9
#define rightMotorDirA 8
#define rightMotorDirB 10

#define demoButtonPin 12
#define raceButtonPin 13

// wspolczynniki PID do zmiany
float Kp = 1.2, Kd = 0.5, Ki = 0.05;
int lastError = 0;
int integral = 0;

// predkosc bazowa do zmiany
int baseSpeed = 150;

// zainicjowanie czujnikow
//QTRSensors qtr;
//const uint8_t SensorCount = 8; //liczba czujnikow
//uint16_t sensorValues[SensorCount]; //macierz z wartosciami czujnikow

//tryby
enum Mode { STANDBY, DEMO, RACE };
Mode currentMode = STANDBY;

void setup() {
  // piny silnikow
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorDirA, OUTPUT);
  pinMode(leftMotorDirB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorDirA, OUTPUT);
  pinMode(rightMotorDirB, OUTPUT);

  // piny przyciskow
  pinMode(demoButtonPin, INPUT_PULLUP);
  pinMode(raceButtonPin, INPUT_PULLUP);

  Serial.begin(9600);

  analogWrite(leftMotorPWM, 0);
  analogWrite(rightMotorPWM, 0);

  digitalWrite(leftMotorDirA, HIGH);
  digitalWrite(leftMotorDirB, LOW);
  digitalWrite(rightMotorDirA, HIGH);
  digitalWrite(rightMotorDirB, LOW);
  //qtr.setTypeRC(); //listwa cyfrowa
  //qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, 3, 4}, SensorCount); //ustawienie pinow czujnikow
  //qtr.setEmitterPin(2); //pin do diody czujnikow

  // kalibracja automatyczna
  //for (uint16_t i = 0; i < 400; i++)
  //{
  //  qtr.calibrate();
  //}
}

void loop() {
    if (digitalRead(demoButtonPin) == LOW && currentMode == STANDBY) {
        currentMode = DEMO;
        demoMode();
        currentMode = STANDBY;
    }

    if (digitalRead(raceButtonPin) == LOW && currentMode == STANDBY) {
        currentMode = RACE;
        raceMode();
        currentMode = STANDBY;
    }
}

// demo mode
void demoMode() {
  Serial.println("Entering Demo Mode");

  unsigned long demoStartTime = millis();
  unsigned long demoDuration = 20000;

  while (millis() - demoStartTime < demoDuration) {
    //followLine();
    setMotorSpeed(0,125);
  }

  stopMotors();
  currentMode = STANDBY;
  Serial.println("Demo Mode Complete. Returning to standby.");
}

// race mode
void raceMode() {
  //unsigned long startTime = millis();
  Serial.println("Entering Race Mode.");
/*
  while (true) {
    followLine();

    // sprawdzenie czy jest meta
    if (isFinishLine()) {
      stopMotors();
      unsigned long lapTime = millis() - startTime;
      Serial.print("Lap Time: ");
      Serial.print(lapTime / 1000.0);
      Serial.println(" seconds.");
      break;
    }
  }
*/
  currentMode = STANDBY;
  Serial.println("Returning to standby.");
}
/*
// podazanie za linia
void followLine() {
  int position = qtrrc.readLine(sensorValues);
  int error = position - 3500;
  int correction = calculatePID(error);

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  setMotorSpeed(leftSpeed, rightSpeed);
}

// wykrycie mety
// bool isFinishLine() {
//   int blackCount = 0;
//   for (int i = 0; i < NUM_SENSORS; i++) {
//     if (sensorValues[i] > 900) blackCount++; // wartosc do zmienienia
//   }
//   return blackCount >= NUM_SENSORS;
// }

// obliczanie pid
// int calculatePID(int error) {
//     integral = constrain(integral + error, -500, 500); //granice do zmienienia
//     int derivative = error - lastError;
//     lastError = error;

//     int correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
//     return correction;
// }
*/

// korekcja predkosci
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  //bool leftForward = (leftSpeed >= 0);
  //bool rightForward = (rightSpeed >= 0);

  //digitalWrite(leftMotorDir, leftForward ? HIGH : LOW);
  //digitalWrite(rightMotorDir, rightForward ? HIGH : LOW);

  analogWrite(leftMotorPWM, leftSpeed);
  analogWrite(rightMotorPWM, rightSpeed);
}

/*
void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(250);
}
*/
// zatrzymanie silnikow
void stopMotors() {
  setMotorSpeed(0, 0);
}
