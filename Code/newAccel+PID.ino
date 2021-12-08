#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "math.h"
#include <Servo.h>
#include "PWM.hpp"
Servo lefty;     // create servo object to control the ESC
PWM ch1(2);
Servo righty;
int check = 1;
int motorPin = 5;

int motorValue = 0;
long oldMillis = 0;
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double x;
double y;
double z;
 
int Drive2;
int Error;
int Previous_error;
int Setpoint;
int Actual;
float Derivative;
float Drive;
float Integral;

//Variables(Use a graph)
float kP = 5;//Start increasing this until it starts to oscillate(go up and down)
double kI = 0;//Finally use this to center your lines
float kD = 0;//Second Increase this until it stops oscillating
float dt =  10;

int outputDrive;
int outputLeft;
int outputRight;
PWM ch2(3);
int left = 1500;
int right = 1500;
int potValue;  // value from the analog pin
/* Assign a unique ID to this sensor at the same time */
float accAngle;
float accAngleYZ;
float accAngleZX;
int input = 1500;
int output;
void getAngles(void) {
Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
AcX=Wire.read()<<8|Wire.read();
AcY=Wire.read()<<8|Wire.read();
AcZ=Wire.read()<<8|Wire.read();
int xAng = map(AcX,minVal,maxVal,-90,90);
int yAng = map(AcY,minVal,maxVal,-90,90);
int zAng = map(AcZ,minVal,maxVal,-90,90);
 
x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
}




void setup(void) {
  lefty.attach(9, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds)
  righty.attach(10, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds)
  ch1.begin(true);
  ch2.begin(true);
  Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(9600);

}

void loop(void) {
  getAngles();
  /* Get a new sensor event */

  /* Display the results (acceleration is measured in m/s^2) */
  /* Serial.print("X: ");
    Serial.print(event.acceleration.x);
    Serial.print("  ");
    Serial.print("Y: ");
    Serial.print(event.acceleration.y);
    Serial.print("  ");
    Serial.print("Z: ");
    Serial.print(event.acceleration.z);
    Serial.print("  ");
    Serial.println("m/s^2");
  
  getAngles();
  while (accAngleYZ < -4) {
    left = left - 5;
    right = right + 5;
    //potValue = analogRead(A0);   // reads the value of the potentiometer (value between 0 and 1023)
    outputLeft = map(left, 1000, 2000, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
    outputRight = map(right, 1000, 2000, 0, 180);
    lefty.write(45);
    righty.write(0);
    getAngles();
    Serial.println(accAngleYZ);

  }
  
  while (accAngleYZ > 4) {
    left = left + 5;
    right = right - 5;
    //potValue = analogRead(A0);   // reads the value of the potentiometer (value between 0 and 1023)
    outputLeft = map(left, 1000, 2000, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
    outputRight = map(right, 1000, 2000, 0, 180);
    lefty.write(0);
    righty.write(45);
    getAngles();
*/
  if (millis() > (oldMillis + dt)) //checks once per second
  {
    getAngles();
 if(z>180){
  z=z-360;
 }
    Actual = z;
  Setpoint =0;
    oldMillis = millis();  //reset milliseconds
  }

  
  Error = Setpoint - Actual;
  Integral = Integral + (Error * dt) / 10; 
  Derivative = (Error - Previous_error) / dt;
  Drive = (Error * kP) + (Integral * kI) + (Derivative * kD);

  Previous_error = Error;
  motorValue = motorValue + Drive;
  
if(Drive>0){
  Drive2=abs(Drive);
    outputDrive = map(Drive2, 0, 2000, 0, 180);
    lefty.write(outputDrive);
  //right motor no change
  //left motor + abs(Drive)
}
  
 if(Drive<0){
  Drive2=abs(Drive);
    outputDrive = map(Drive2, 0, 2000, 0, 180);
    righty.write(outputDrive);
  //right motor no change
  //left motor + abs(Drive)
} 
  
  
 //Serial.println(map(z, 0,360, -180, 180));
  //if(z>=270&&z<360){
   // z=z-270;
  //}
  //if(z>=90&&z<270){
   // z=z+90;
  //}

    Serial.println(z);
  
  /* Delay before the next sample */
  delay(50);
}
