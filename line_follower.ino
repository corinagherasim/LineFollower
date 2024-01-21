#include <QTRSensors.h>
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

int m1Speed = 0;
int m2Speed = 0;

const int calibrationSpeed = 190;

const int rightTurn = 1;
const int leftTurn = -1;
const int calibrationTurns = 5;
int turn = 0;

float kp = 10.5;
float ki = 0.0005;
float kd = 18.5;

float p = 0;
float i = 0;
float d = 0;

int error = 0;
int lastError = 0;
int cummulativeError = 0;

const int maxSpeed = 255;
const int minSpeed = -255;

const int baseSpeed = 255;  // 240 -> 17.76s

const int stoppingSpeed = -150;  // -190
const int breakingSpeed = 45;    // 100 + 240 -> 17.46s

QTRSensors qtr;

char state = 'R';
const int sensorCount = 6;
int sensorValues[sensorCount];

const uint8_t leftmostSensor = 0;
const uint8_t rightmostSensor = 5;
const int sensorThreshold = 500;

const int minSensorValue = 0;
const int maxSensorValue = 5000;
const int maxErrorThreshold = 50;  // check with greater value (45 works)
const int minErrorThreshold = 0;

void setup() {
  Serial.begin(9600);

  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);

  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  calibration();
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  pidControl();
  setMotorSpeed(m1Speed, m2Speed);
}

void makeTurn(char direction, int sensorValue) {
  if (direction == 'R') {
    if (sensorValue < sensorThreshold) {
      setMotorSpeed(calibrationSpeed, -calibrationSpeed);
    } else {
      state = 'L';
    }
  } else if (direction == 'L') {
    if (sensorValue < sensorThreshold) {
      setMotorSpeed(-calibrationSpeed, calibrationSpeed);
    } else {
      state = 'R';
      turn++;
    }
  }
}

void calibration() {
  Serial.println("Calibrating...");
  while (turn <= calibrationTurns) {
    qtr.calibrate();
    qtr.read(sensorValues);
    if (state == 'R') {
      Serial.println("Right");
      makeTurn('R', sensorValues[rightmostSensor]);
    } else if (state == 'L') {
      Serial.println("Left");
      makeTurn('L', sensorValues[leftmostSensor]);
    }
  }
  Serial.println("Done calibrating!");
}

void pidControl() {
  float error = map(qtr.readLineBlack(sensorValues), minSensorValue, maxSensorValue, -50, 50);
  error = (-1) * error;
  p = error;
  i = i + error;
  d = error - lastError;
  lastError = error;

  int motorSpeed = kp * p + ki * cummulativeError + kd * d;
  m1Speed = baseSpeed;
  m2Speed = baseSpeed;
  cummulativeError += error;

  if (error < minErrorThreshold) {
    m1Speed += motorSpeed;
  } else if (error > minErrorThreshold) {
    m2Speed -= motorSpeed;
  }

  if (error <= -maxErrorThreshold) {
    m2Speed -= breakingSpeed;
  } else if (error >= maxErrorThreshold) {
    m1Speed -= breakingSpeed;
  }
  m1Speed = constrain(m1Speed, stoppingSpeed, maxSpeed);
  m2Speed = constrain(m2Speed, stoppingSpeed, maxSpeed);
}

void setMotorSpeed(int motor1Speed, int motor2Speed) {
  motor2Speed = -motor2Speed;

  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  } else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }

    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }

  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  } else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }

    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}