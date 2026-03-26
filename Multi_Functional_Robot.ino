#include <SoftwareSerial.h>
#include <IRremote.h>
#include <Servo.h>

SoftwareSerial BT(2, 3);

// -------- IR --------
#define IR_PIN 7

// -------- Servo --------
Servo myServo;

// -------- Motor Pins --------
#define ENA 5
#define ENB 6
#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11

// -------- Ultrasonic --------
#define trigPin 12
#define echoPin 13

// -------- Line Sensors --------
#define IR_LEFT A0
#define IR_CENTER A1
#define IR_RIGHT A2

// -------- VARIABLES --------
int mode = 0;
int motorSpeed = 150;

// HOLD SYSTEM
unsigned long lastCommandTime = 0;
const int holdTimeout = 200;

// -------- MOTOR FUNCTIONS --------
void forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void backward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}
void left() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void right() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// -------- DISTANCE --------
long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  return duration / 58;
}

// -------- OBSTACLE MODE --------
void obstacleMode() {
  long distance = getDistance();

  if (distance > 30) {
    forward();
  } else {
    stopMotors();
    delay(200);

    myServo.write(150);
    delay(400);
    long leftDist = getDistance();

    myServo.write(30);
    delay(400);
    long rightDist = getDistance();

    myServo.write(90);
    delay(200);

    if (leftDist > rightDist) left();
    else right();

    delay(500);
  }
}

// -------- LINE FOLLOW MODE (2 SENSOR LOGIC) --------
void lineFollowMode() {

  int L = digitalRead(IR_LEFT);
  int R = digitalRead(IR_RIGHT);

  int MOTOR_SPEED = 60;
  int TURN_SPEED = 70;
  int RIGHT_TRIM = -10;

  if (L == LOW && R == LOW) {
    // Straight
    analogWrite(ENA, MOTOR_SPEED + RIGHT_TRIM);
    analogWrite(ENB, MOTOR_SPEED - RIGHT_TRIM);
    forward();
  }
  else if (R == HIGH && L == LOW) {
    // Turn RIGHT
    analogWrite(ENA, TURN_SPEED);
    analogWrite(ENB, TURN_SPEED);
    right();
  }
  else if (L == HIGH && R == LOW) {
    // Turn LEFT
    analogWrite(ENA, TURN_SPEED);
    analogWrite(ENB, TURN_SPEED);
    left();
  }
  else {
    // Both black → STOP
    stopMotors();
  }
}

// -------- SETUP --------
void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);

  myServo.attach(4);
  myServo.write(90);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_CENTER, INPUT);
  pinMode(IR_RIGHT, INPUT);

  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

// -------- LOOP --------
void loop() {

  // -------- BLUETOOTH CONTROL --------
  if (BT.available()) {
    char cmd = BT.read();
    lastCommandTime = millis();

    if (cmd == 'F') { mode = 0; forward(); }
    else if (cmd == 'B') { mode = 0; backward(); }
    else if (cmd == 'L') { mode = 0; left(); }
    else if (cmd == 'R') { mode = 0; right(); }
    else if (cmd == 'S') {
      stopMotors();
      mode = 0;
    }
    else if (cmd == '1') mode = 1; // obstacle
    else if (cmd == '3') mode = 3; // line follow
    else if (cmd == '4') mode = 0;
    else if (cmd == '2') {
      stopMotors();
      mode = 0;
    }
  }

  // -------- IR CONTROL --------
  if (IrReceiver.decode()) {

    unsigned long val = IrReceiver.decodedIRData.decodedRawData;
    Serial.println(val, HEX);

    if (val == 0xFFFFFFFF) {
      lastCommandTime = millis();
      IrReceiver.resume();
      return;
    }

    lastCommandTime = millis();

    if (val == 0xE718FF00) { mode = 0; forward(); }
    else if (val == 0xAD52FF00) { mode = 0; backward(); }
    else if (val == 0xF708FF00) { mode = 0; left(); }
    else if (val == 0xA55AFF00) { mode = 0; right(); }

    else if (val == 0xE31CFF00) {
      stopMotors();
      mode = 0;
    }

    else if (val == 0xBA45FF00) mode = 1;
    else if (val == 0xB847FF00) mode = 3;

    else if (val == 0xB946FF00) {
      mode = 0;
      stopMotors();
    }
    else if (val == 0xBB44FF00) {
      mode = 0;
      stopMotors();
    }

    IrReceiver.resume();
  }

  // -------- HOLD STOP --------
  if (mode == 0 && (millis() - lastCommandTime > holdTimeout)) {
    stopMotors();
  }

  // -------- AUTO MODES --------
  if (mode == 1) obstacleMode();
  else if (mode == 3) lineFollowMode();
}