#include <QTRSensors.h>

// zdefiniowanie pinow
#define TIMEOUT 2500

#define leftMotorPWM 10 
#define leftMotorDirA 7
#define leftMotorDirB 5
#define rightMotorPWM 9
#define rightMotorDirA 8
#define rightMotorDirB 6

#define demoButtonPin 11
#define raceButtonPin 12

// wspolczynniki PID do zmiany
float Kp = 1.0, Kd = 0.05, Ki = 0.005;
int lastError = 0;
int sensorPositions[8] = {0,1,2,3,4,5,6,7};
bool finishLine = false;
int bufforMeta = 0;

// predkosc bazowa do zmiany
uint8_t baseSpeed = 100;

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
  digitalWrite(rightMotorDirA, LOW);
  digitalWrite(rightMotorDirB, HIGH);

  qtr.setTypeRC(); //listwa cyfrowa
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, 3, 4}, SensorCount); //ustawienie pinow czujnikow
  qtr.setEmitterPin(2); //pin do diody czujnikow

  TCCR1B = (TCCR1B & 0b11111000) | 0x01; // zmiana czestotliwosci pwm dla pinow 9 i 10
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
  uint16_t demoDuration = 30000;

  while (millis() - demoStartTime < demoDuration) {
    followLine();
  }

  stopMotors();
  currentMode = STANDBY;
  Serial.println("Demo Mode Complete. Returning to standby.");
}

// race mode
void raceMode() {
  Serial.println("Entering Race Mode.");
  delay(3000);
  unsigned long startTime = millis();

  while (true) {
    followLine();

  // sprawdzenie czy jest meta
    if (finishLine == true) {
      stopMotors();
      unsigned long lapTime = millis() - startTime;
      Serial.print("Lap Time: ");
      Serial.print(lapTime / 1000.0);
      Serial.println(" seconds.");
      finishLine = false;
      bufforMeta = 0;
      break;
    }
  }

  currentMode = STANDBY;
  Serial.println("Returning to standby.");
}

// podazanie za linia
void followLine() {
  float error = readLine(sensorValues);
  int intError = error * 100;
  int correction = calculatePID(intError);

  uint8_t leftSpeed = constrain(baseSpeed + correction, 0, 255);
  uint8_t rightSpeed = constrain(baseSpeed - correction, 0, 255);

  setMotorSpeed(leftSpeed, rightSpeed);
}

float readLine(int sensorValues[]) {

  qtr.read(sensorValues);
  float error = 0;
  int count = 0;
  int binaryValues[SensorCount];
  float center = (SensorCount - 1) / 2.0;

  for (int i = 0; i < SensorCount; i++) {
    int state = (sensorValues[i] < 1200) ? 0 : 1;  // > na biala linie, < na czarna
    binaryValues[i] = state;
    
    // Serial.print(state);
    // Serial.print('\t');

    if (state == 1) {
      error = error + (i - center);  // Sum up active sensor positions
      count++;  // Count active sensors
    }
  }
  
  if (count > 0) {
    error /= count;
    if (count >= SensorCount) {
      bufforMeta++;
    }
    else {
      bufforMeta = 0;
    }
  }
  if (bufforMeta >= 150) {
    isFinishLine(true);
  }
  error = error * (-1);
  return error;  // Return average position
}

// wykrycie mety
void isFinishLine(bool meta) {
  if (meta == true) {
    finishLine = true;
  }
  else {
    finishLine = false;
  }
}

// obliczanie pid
int calculatePID(int error) {

  int integral = 0;
  integral = integral + error;
  int derivative = error - lastError;
  lastError = error;

  int correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  correction = correction * 100 / 255;
  Serial.println(correction);
  return correction;
}

// korekcja predkosci
void setMotorSpeed(int leftSpeed, int rightSpeed) {

  // Serial.print(leftSpeed);
  // Serial.print('\t');
  // Serial.print(rightSpeed);
  // Serial.println();

  analogWrite(leftMotorPWM, leftSpeed);
  analogWrite(rightMotorPWM, rightSpeed);
}

// zatrzymanie silnikow
void stopMotors() {
  setMotorSpeed(0, 0);
}

void motorsForward() {
  digitalWrite(leftMotorDirA, HIGH);
  digitalWrite(leftMotorDirB, LOW);
  digitalWrite(rightMotorDirA, LOW);
  digitalWrite(rightMotorDirB, HIGH);
}

void motorReverse() {
  digitalWrite(leftMotorDirA, LOW);
  digitalWrite(leftMotorDirB, HIGH);
  digitalWrite(rightMotorDirA, HIGH);
  digitalWrite(rightMotorDirB, LOW);
}
