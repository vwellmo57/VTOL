#include <Servo.h>
#define MIN_PULSE_LENGTH 1000
#define MAX_PULSE_LENGTH 2000
#include <EnableInterrupt.h>
#define EI_NOTINT0
#define EI_NOTINT1
#define SERIAL_PORT_SPEED 9600
#define RC_NUM_CHANNELS  4

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1
#define RC_CH3_INPUT  A2
#define RC_CH4_INPUT  A3

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

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
float multiplier = .3;
Servo aileron;
Servo elevator;
Servo LMotor;
Servo RMotor;

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}


void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}


void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIAL_PORT_SPEED);

  LMotor.attach(LMotorPin);
  RMotor.attach(RMotorPin);
  elevator.attach(elevatorPin);
  aileron.attach(aileronPin);

  LMotor.write(0); // send "stop" signal to ESC. Also necessary to arm the ESC.
  RMotor.write(0);
  elevator.write(90);
  aileron.write(90);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);

}

void loop() {
  rc_read_values();
  LMotor.writeMicroseconds(MAX_PULSE_LENGTH);
  LMotor.writeMicroseconds(MIN_PULSE_LENGTH);
  RMotor.writeMicroseconds(MAX_PULSE_LENGTH);
  RMotor.writeMicroseconds(MIN_PULSE_LENGTH);

  throttleVal = rc_values[RC_CH1];
  //throttleVal = throttleVal - 1000;
  throttleVal = map(throttleVal, 1000, 2000, 0, 190);
//throttleVal = map(throttleVal, 30, 164, 0, 180);
//Serial.println(throttleVal);
throttleVal=throttleVal+8;
  directionVal = map(rc_values[RC_CH4], 1000, 2000, 0, 180);
  //directionVal=direction
  directionVal=directionVal+6;
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
  elevatorVal = map(rc_values[RC_CH3], 1192, 1700, 0, 180);
  if (elevatorVal < 5) {
    elevatorVal = 0;
  }
  if (elevatorVal > 175) {
    elevatorVal = 180;
  }
  aileronVal = map(rc_values[RC_CH2], 1192, 1700, 0, 180);
  if (aileronVal < 5) {
    aileronVal = 0;
  }
  if (aileronVal > 175) {
    aileronVal = 180;
  }

  
  //Serial.println(aileronVal);
  elevator.write(elevatorVal);
  aileron.write(aileronVal);

//  Serial.print("Throttle:"); Serial.print(rc_values[RC_CH1]); Serial.print("/"); Serial.print(throttleVal);  Serial.print("\t");
//  Serial.print("Aileron:"); Serial.print(rc_values[RC_CH2]); Serial.print("/"); Serial.print(aileronVal); Serial.print("\t");
 // Serial.print("Elevator:"); Serial.print(rc_values[RC_CH3]); Serial.print("/"); Serial.print(elevatorVal); Serial.print("\t");
  Serial.print("Diff:"); Serial.println(directionVal); Serial.print("\t");
  Serial.print("LMotor: "); Serial.println(LMotorVal); Serial.print("\t");
Serial.print("RMotor: "); Serial.println(RMotorVal); 
//delay(20);
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
