#include "PWM.hpp"
#include <Servo.h>
#include <Wire.h>
#define MIN_PULSE_LENGTH 1000
#define MAX_PULSE_LENGTH 2000
PWM aCH(1);
PWM bCH(2);
PWM cCH(3);
PWM dCH(4);
PWM eCH(5);
PWM fCH(6);


void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
aCH.begin(true);
bCH.begin(true);
cCH.begin(true);
dCH.begin(true);
eCH.begin(true);
fCH.begin(true);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(aCH.getValue());
    Serial.println(bCH.getValue());
      Serial.println(cCH.getValue());
        Serial.println(dCH.getValue());
          Serial.println(eCH.getValue());
            Serial.println(fCH.getValue());
              Serial.println("-------------------------");
}
