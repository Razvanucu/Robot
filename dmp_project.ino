#include "Servo.h"
#include <stdio.h>

const int trigPin1 = 13;
const int echoPin1 = 12;
const int trigPin2 = 8;
const int echoPin2 = 7;

const int motor00 = 11;
const int motor01 = 10;
const int motor10 = 6;
const int motor11 = 5;

boolean stringComplete = false;
String inputString = "";

Servo myServo1, myServo2;

struct accelerometer{
  float x;
  float y;
  float z;
} accl;

struct gyroscope{
  float x;
  float y;
  float z;
} gyro;

void setup() {
  Serial.begin(9600);
  while(!Serial){
    delay(10); //wait for Serial Monitor to open
  }

  Serial1.begin(9600);
  while(!Serial1){
    delay(10); //wait for Serial Monitor to open
  }
  //initialize motor pins
  stop();
  //reset Servo  motors to angle 0
  resetServo();


  //pin modes for ultrasonic sensors
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  //pin modes for motors
  pinMode(motor00, OUTPUT);
  pinMode(motor01, OUTPUT);
  pinMode(motor10, OUTPUT);
  pinMode(motor11, OUTPUT);
}
void loop() {

  ultrasonicSensor(echoPin1, trigPin1, 0x00);
  ultrasonicSensor(echoPin2, trigPin2, 0x01);

  if(stringComplete){
    sscanf(inputString.c_str(),"| Accelerometer (g) >>> x: %+07.2f y: %+07.2f z: %+07.2f || Gyroscope (°/sec) >>> x: %+07.2f y: %+07.2f z: %+07.2f |",&accl.x,&accl.y,&accl.z,
    &gyro.x,&gyro.y,&gyro.z);
    
    int angleX = map(accl.x, -1.0, 1.0, 0, 180); // Map accelerometer values to angles for servo motor movement
    int angleY = map(accl.y, -1.0, 1.0, 0, 180);
    
    controlServo(angleX, angleY, 2);

    if (abs(accl.x) > 0.5 || abs(accl.y) > 0.5) {
      stop(); // Stop motors on excessive tilt
    } else {
      start(motor00, motor01, true, 100); // Move forward
      start(motor10, motor11, true, 100);
    }

    if (gyro.x > 50.0 || gyro.y > 50.0) {
      controlServo(0, 0, 2); 
    }
    stringComplete=false;
  }
  
}
bool ultrasonicSensor(const int echoPin, const int trigPin, char which) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  float distance = (duration * .0343) / 2;

  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance < 10.0f) {
    stop();
    delay(1000);
    start(motor00,motor01,false,30); //go back
    start(motor10,motor11,false,30); //go back
    delay(1000);
    stop();
  }
}
void stop() {
  digitalWrite(motor00, LOW);
  digitalWrite(motor01, LOW);
  digitalWrite(motor10, LOW);
  digitalWrite(motor11, LOW);
}
void start(const int motorPin0, const int motorPin1, bool forward, uint8_t speed) {
  if (forward) {
    analogWrite(motorPin0, speed);
    digitalWrite(motorPin1, LOW);
  } else {
    digitalWrite(motorPin0, LOW);
    analogWrite(motorPin1, speed);
  }
}
void resetServo() {
  myServo1.attach(9);
  myServo2.attach(4);
  delay(30);
  myServo1.write(0);
  myServo2.write(0);

  delay(1000);
  myServo1.detach();
  myServo2.detach();
}
void controlServo(int angle1, int angle2, int which) {

  switch (which) {
    case 0:
      myServo1.attach(9);
      delay(30);
      myServo1.write(angle1);
      delay(1000);
      myServo1.detach();
      break;
    case 1:
      myServo2.attach(4);
      delay(30);
      myServo2.write(angle2);
      delay(1000);
      myServo2.detach();
      break;
    case 2:
      myServo1.attach(9);
      myServo2.attach(4);
      
      delay(30);

      myServo1.write(angle1);
      myServo2.write(angle2);

      delay(1000);
      
      myServo1.detach();
      myServo2.detach();
      break;
    default:
     Serial.println("Wrong command for controlling Servos!");
  }
}
void serialEvent1(){
    while (Serial1.available()) {
    char inChar = (char)Serial1.read();

    if (inChar != '\n') {
      inputString += inChar;
    }

    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}