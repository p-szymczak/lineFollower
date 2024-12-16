int pwmA = 5;  // PWM Motor A
int inA1 = 7;  // INA Motor A
int inA2 = 8;  // INB Motor A

int pwmB = 6;  // PWM Motor B
int inB1 = 9;  // INA Motor B
int inB2 = 10; // INB Motor B

void setup() {
  pinMode(pwmA, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);

  pinMode(pwmB, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);

  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);
}

void loop() {
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);

  for (int speed = 0; speed <= 255; speed += 25) {
    analogWrite(pwmA, speed);
    delay(1000);
  }

  for (int speed = 255; speed >= 0; speed -= 25) {
    analogWrite(pwmA, speed);
    delay(1000);
  }

  analogWrite(pwmA, 0);
  delay(2000);

  digitalWrite(inB1, HIGH);
  digitalWrite(inB2, LOW);

  for (int speed = 0; speed <= 255; speed += 25) {
    analogWrite(pwmB, speed);
    delay(1000);
  }

  for (int speed = 255; speed >= 0; speed -= 25) {
    analogWrite(pwmB, speed);
    delay(1000);
  }

  analogWrite(pwmB, 0);
  delay(2000);

  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);

  for (int speed = 0; speed <= 255; speed += 25) {
    analogWrite(pwmA, speed);
    delay(1000);
  }

  for (int speed = 255; speed >= 0; speed -= 25) {
    analogWrite(pwmA, speed);
    delay(1000);
  }

  analogWrite(pwmA, 0);
  delay(2000);

  digitalWrite(inB1, LOW);
  digitalWrite(inB2, HIGH);

  for (int speed = 0; speed <= 255; speed += 25) {
    analogWrite(pwmB, speed);
    delay(1000);
  }

  for (int speed = 255; speed >= 0; speed -= 25) {
    analogWrite(pwmB, speed);
    delay(1000);
  }

  analogWrite(pwmB, 0);
  delay(2000);
}
