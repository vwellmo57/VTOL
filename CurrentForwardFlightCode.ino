#include "PWM.hpp"
#include <Servo.h>
#include <Wire.h>
#define MIN_PULSE_LENGTH 1000
#define MAX_PULSE_LENGTH 2000
PWM eCH(2);
PWM tCH(3);
PWM dCH(13);
PWM aCH(0);
int LMotorPin = 6;
int RMotorPin = 7;
int elevatorPin = 8;
int aileronPin=9;
int throttleVal;
int LMotorVal;
int RMotorVal;
int elevatorVal;
int directionVal;
int aileronVal;
int multiplier = 1;
Servo aileron;
Servo elevator;
Servo LMotor;
Servo RMotor;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  eCH.begin(true);
  tCH.begin(true);
  dCH.begin(true);
  aCH.begin(true);
  LMotor.attach(LMotorPin);
  RMotor.attach(RMotorPin);
  elevator.attach(elevatorPin);
  aileron.attach(aileronPin);

  LMotor.write(0); // send "stop" signal to ESC. Also necessary to arm the ESC.
  RMotor.write(0);
  elevator.write(90);
  aileron.write(90);



}

void loop() {
  Serial.println(aCH.getValue());
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
  aileronVal = map(aCH.getValue(), 1192, 1700, 0, 180);
  if (aileronVal < 5) {
    aileronVal = 0;
  }
  if (aileronVal > 175) {
    aileronVal = 180;
  }

  
  //Serial.println(aileronVal);
  elevator.write(elevatorVal);
  aileron.write(aileronVal);




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
