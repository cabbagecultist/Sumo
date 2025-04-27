#include <Arduino.h>
#include "PController.h"

PController angleController(0.5);

void setup() {
  Serial.begin(9600);
  angleController.setSetpoint(0.5);
}

void loop() {
  float measurement = cos(((float)millis())*0.001);
  Serial.println(angleController.calculate(measurement));
}
