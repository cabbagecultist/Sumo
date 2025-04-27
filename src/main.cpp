#include <Arduino.h>
#include "Wire.h"

#include <Constants.h>
#include "subsystems/DistanceSensors.h"


// TODO Organise includes because this is a mess

DistanceSensors distanceSensors;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize I2C communication
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_SPEED_NORMAL);

  // Initialize distance sensors
  distanceSensors.begin();
}

void loop() {
}








