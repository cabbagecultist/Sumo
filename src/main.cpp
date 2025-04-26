#include <Arduino.h>
#include "PController.h"

void setup() {
  PController angleController(0.1);
  angleController.setSetpoint(1);
}

void loop() {
  
}
