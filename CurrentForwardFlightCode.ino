#include "PWM.hpp"
#include <Servo.h>
#include <Wire.h>
#define MIN_PULSE_LENGTH 1000
#define MAX_PULSE_LENGTH 2000
PWM eCH(2);
PWM tCH(3);
PWM dCH(4);
int LMotorPin = 6;
int RMotorPin = 7;
int elevatorPin = 8;
int throttleVal;
int LMotorVal;
int RMotorVal;
int elevatorVal;
int directionVal;
int multiplier = 1;
int val2;
int val;
int ogval;
Servo elevator;
Servo LMotor;
Servo RMotor;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  eCH.begin(true);
  tCH.begin(true);
  LMotor.attach(LMotorPin);
  RMotor.attach(RMotorPin);
  elevator.attach(elevatorPin);

  LMotor.write(0); // send "stop" signal to ESC. Also necessary to arm the ESC.
  RMotor.write(0);
  elevator.write(90);



}

void loop() {
  LMotor.writeMicroseconds(MAX_PULSE_LENGTH);
  LMotor.writeMicroseconds(MIN_PULSE_LENGTH);
  RMotor.writeMicroseconds(MAX_PULSE_LENGTH);
  RMotor.writeMicroseconds(MIN_PULSE_LENGTH);

  throttleVal = tCH.getValue();
  throttleVal = throttleVal - 1000;
  throttleVal = map(throttleVal, 1000, 2000, 0, 180);

  directionVal = map(dCH.getValue(), 1192, 1700, 0, 180);
  if (directionVal < 5) {
    directionVal = 0;
  }
  if (directionVal > 175) {
    directionVal = 180;
  }

  if (directionVal < 85) {
    LMotorVal = throttleVal - (multiplier * (90 - directionVal));
    RMotorVal = throttleVal + (multiplier * (90 - directionVal));
  }
  else if (directionVal > 95) {
    LMotorVal = throttleVal - (multiplier * (90 - directionVal));
    RMotorVal = throttleVal + (multiplier * (90 - directionVal));
  }
  else {
    LMotorVal = throttleVal;
    RMotorVal = throttleVal;
  }

LMotor.write(LMotorVal);
RMotor.write(RMotorVal);
  elevatorVal = map(eCH.getValue(), 1192, 1700, 0, 180);
  if (elevatorVal < 5) {
    elevatorVal = 0;
  }
  if (elevatorVal > 175) {
    elevatorVal = 180;
  }

  //Serial.println(elevatorVal);
  elevator.write(elevatorVal);




  // Serial.print(",");
  //Serial.println(ch2.getValue());
  // throttleVal = throttleVal - 1000;
  //  throttleVal = map(throttleVal, 1000, 2000, 0, 180);
  // servo.write(val2);
  //  Serial.println(val2);
  // servo.write(val2); // Send signal to ESC.
  //for(int i=0; i<10000; i++){
  // servo.write(i);
  // Serial.println(i);
  //delay(50);
  //}
  // analogWrite(motorPin, 1000);
}
