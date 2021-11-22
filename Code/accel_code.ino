#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "math.h"
#include <Servo.h>
#include "PWM.hpp"
Servo lefty;     // create servo object to control the ESC
PWM ch1(2);
Servo righty;
int outputLeft;
int outputRight;
PWM ch2(3);
int left = 1500;
int right = 1500;
int potValue;  // value from the analog pin
/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
float accAngle;
float accAngleYZ;
float accAngleZX;
int input = 1500;
int output;
void getAngles(void) {
  sensors_event_t event;
  accel.getEvent(&event);
  accAngleYZ = atan2(event.acceleration.y, event.acceleration.z) * RAD_TO_DEG;
  accAngleZX = atan2(event.acceleration.z, event.acceleration.x) * RAD_TO_DEG;
  /*if (isnan(accAngleZX));
  else
    Serial.print("YX: ");
  Serial.print(accAngleZX);


  if (isnan(accAngleYZ));
  else
    Serial.print(" YZ: ");
  Serial.println(accAngleYZ);*/
}


void displaySensorDetails(void) {
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" m/s^2");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" m/s^2");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup(void) {
  lefty.attach(9, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds)
  righty.attach(10, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds)
  ch1.begin(true);
  ch2.begin(true);
#ifndef ESP8266


  while (!Serial)
    ; // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  Serial.println("Accelerometer Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();

  accel.setRange(LSM303_RANGE_4G);
  Serial.print("Range set to: ");
  lsm303_accel_range_t new_range = accel.getRange();
  switch (new_range) {
    case LSM303_RANGE_2G:
      Serial.println("+- 2G");
      break;
    case LSM303_RANGE_4G:
      Serial.println("+- 4G");
      break;
    case LSM303_RANGE_8G:
      Serial.println("+- 8G");
      break;
    case LSM303_RANGE_16G:
      Serial.println("+- 16G");
      break;
  }

  accel.setMode(LSM303_MODE_NORMAL);
  Serial.print("Mode set to: ");
  lsm303_accel_mode_t new_mode = accel.getMode();
  switch (new_mode) {
    case LSM303_MODE_NORMAL:
      Serial.println("Normal");
      break;
    case LSM303_MODE_LOW_POWER:
      Serial.println("Low Power");
      break;
    case LSM303_MODE_HIGH_RESOLUTION:
      Serial.println("High Resolution");
      break;
  }
  delay(8000);
}

void loop(void) {
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
  */
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
Serial.println(accAngleYZ);
  }

  /* Delay before the next sample */
  delay(50);
}
