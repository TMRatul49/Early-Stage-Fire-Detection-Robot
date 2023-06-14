#include <SoftwareSerial.h>
#include <Servo.h>
#include <NewPing.h>

const int TXD = 5;
const int RXD = 6;
SoftwareSerial bluetooth(TXD, RXD);

char datainput, dataoutput;

const int LeftMotorForward = 9;
const int LeftMotorBackward = 10;
const int RightMotorForward = 8;
const int RightMotorBackward = 7;

int forward_flame_value, left_flame_value, right_flame_value;
#define trig_pin 12
#define echo_pin 13
#define forward_flame A3
#define right_flame A4
#define left_flame A5
#define MQ_2 A2
#define maximum_distance 200

boolean goesForward = false;

const int buzzr = 2;  //buzzr
int distance;
int gasValue;
int distanceRight = 0;
int distanceLeft = 0;
char command;
int mode;

NewPing sonar(trig_pin, echo_pin, maximum_distance);
Servo servo_motor;

void setup() {

  Serial.begin(9600);
  bluetooth.begin(9600);
  pinMode(buzzr, OUTPUT);  //buzzr
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(forward_flame, INPUT);
  pinMode(left_flame, INPUT);
  pinMode(right_flame, INPUT);
  pinMode(MQ_2, INPUT);
  servo_motor.attach(11);

  servo_motor.write(115);
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
}
int lookRight() {
  servo_motor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int lookLeft() {
  servo_motor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
  delay(100);
}

int readPing() {
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 250;
  }
  return cm;
}

void moveStop() {

  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
  delay(50);
}

void moveBackward() {  //lin c....

  if (!goesForward) {

    goesForward = true;

    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);

    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW);
    delay(50);
  }
}

void moveForward() {  //lin c............

  goesForward = false;

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);

  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);
  delay(50);
}

void turnRight() {

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);

  delay(250);

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}

void turnLeft() {

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);

  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);

  delay(250);

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}
void flame() {                           //flame mode
  Serial.print("distance  ");
  Serial.println(distance);

  forward_flame_value = analogRead(forward_flame);
  left_flame_value = analogRead(left_flame);
  right_flame_value = analogRead(right_flame);
  gasValue = analogRead(MQ_2);

  Serial.println(forward_flame_value);
  Serial.println(left_flame_value);
  Serial.println(right_flame_value);
  Serial.println(gasValue);

  if (forward_flame_value < 500) {

    moveStop();
    digitalWrite(buzzr, HIGH);
    delay(3000);
    moveStop();
    goesForward = false;
    Serial.println("front fire");
  } else if (right_flame_value < 400) {

    moveBackward();
    turnRight();
    moveStop();
    digitalWrite(buzzr, HIGH);
    delay(3000);
    moveStop();
    goesForward = false;
    Serial.println("right fire");

  } else if (left_flame_value < 400) {
    moveBackward();
    turnLeft();
    moveStop();
    digitalWrite(buzzr, HIGH);
    delay(3000);
    moveStop();

    goesForward = false;
    Serial.println("left fire");
  } else {

    digitalWrite(buzzr, LOW);
    delay(50);

  if (gasValue > 250){                 //gas mode
    digitalWrite(buzzr, HIGH);
    delay(3000); 
    moveStop();

    goesForward = false;
    Serial.println("Gas Detected"); 
  } else {
    digitalWrite(buzzr, LOW);
    delay(50);
  }

    if (distance <= 30) {
      moveStop();
      delay(300);
      moveBackward();
      delay(400);
      moveStop();
      delay(300);
      distanceRight = lookRight();
      delay(300);
      distanceLeft = lookLeft();
      delay(300);

      if (distance >= distanceLeft) {
        turnRight();
        moveStop();
        delay(50);
      } else {
        turnLeft();
        moveStop();
        delay(50);
      }
    } else {
      moveForward();
    }
    distance = readPing();
  }
}
void moveCar(char command) {
  switch (command) {
    case 'u':  //forward
      digitalWrite(LeftMotorForward, HIGH);
      digitalWrite(LeftMotorBackward, LOW);
      digitalWrite(RightMotorForward, HIGH);
      digitalWrite(RightMotorBackward, LOW);
      break;
    case 'd':  //backward
      digitalWrite(LeftMotorForward, LOW);
      digitalWrite(LeftMotorBackward, HIGH);
      digitalWrite(RightMotorForward, LOW);
      digitalWrite(RightMotorBackward, HIGH);
      break;
    case 'r':  //right
      digitalWrite(LeftMotorForward, HIGH);
      digitalWrite(LeftMotorBackward, LOW);
      digitalWrite(RightMotorForward, LOW);
      digitalWrite(RightMotorBackward, LOW);
      break;
    case 'l':  //left
      digitalWrite(LeftMotorForward, LOW);
      digitalWrite(LeftMotorBackward, LOW);
      digitalWrite(RightMotorForward, HIGH);
      digitalWrite(RightMotorBackward, LOW);
      break;
    default:  //stop
      digitalWrite(LeftMotorForward, LOW);
      digitalWrite(LeftMotorBackward, LOW);
      digitalWrite(RightMotorForward, LOW);
      digitalWrite(RightMotorBackward, LOW);
      break;
  }
}
void loop() {

  analogWrite(3, 128);
  if (bluetooth.available()) {
    command = bluetooth.read();
    moveCar(command);
    Serial.print(command);
    if (command == 'm') {
      mode = 99;
    } else if (command == 'a') {
      mode = 88;
    }

  } else if (Serial.available()) {
    dataoutput = Serial.read();
    bluetooth.write(command);
  }
  if (mode == 99) {
    flame();
    if (mode == 88) {
      moveCar(command);
    }
  }
}