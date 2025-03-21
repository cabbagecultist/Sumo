#include <Arduino.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <VL53L4CD.h>

void setupReflectance();
void setupDistance();

QTRSensors qtr;

const uint8_t SensorCountReflectance = 1;
uint16_t sensorValues[SensorCountReflectance];

const uint8_t SensorCountDistance = 2;
const uint8_t xshutPins[SensorCountDistance] = {16, 17};
VL53L4CD distanceSensors[SensorCountDistance];

enum TEST_MODE {
  TEST_DISTANCE,
  TEST_REFLECTANCE
};

// Change to change test mode
TEST_MODE MODE = TEST_DISTANCE;

void setup() {
  // Wait for microcontroller cereal init

  // Init comms
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);

  // Init sensors
  switch (MODE)
  {
  case TEST_DISTANCE:
    setupDistance();
    break;
  
  case TEST_REFLECTANCE:
    setupReflectance();
    break;

  default:
    break;
  }
}

void loop() {
  switch (MODE) {
    case TEST_DISTANCE:
      {
        // Print distance sensor values
        for (uint8_t i = 0; i < SensorCountDistance; i++)
        {
          Serial.print(distanceSensors[i].read());
          if (distanceSensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
          Serial.print('\t');
        }
        Serial.println();
        break;
      }
  
    case TEST_REFLECTANCE:
      {
        // read calibrated sensor values and obtain a measure of the line position
        // from 0 to 5000 (for a white line, use readLineWhite() instead)
        uint16_t position = qtr.readLineBlack(sensorValues);

        // print the sensor values as numbers from 0 to 1000, where 0 means maximum
        // reflectance and 1000 means minimum reflectance, followed by the line
        // position
        for (uint8_t i = 0; i < SensorCountReflectance; i++)
        {
          Serial.print(sensorValues[i]);
          Serial.print('\t');
        }
        Serial.println(position);
        break;
      }

    default:
      break;
  }

  delay(100);
}

void setupDistance() {
  for (uint8_t i = 0; i < SensorCountDistance; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < SensorCountDistance; i++) {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    pinMode(xshutPins[i], INPUT);
    delay(10);

    distanceSensors[i].setTimeout(500);
    if (!distanceSensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    distanceSensors[i].setAddress(0x2A + i);

    distanceSensors[i].startContinuous();
  }
}

void setupReflectance() {
  // put your setup code here, to run once:
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){27}, SensorCountReflectance);
  qtr.setEmitterPin(26);

  delay(500);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(2, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCountReflectance; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCountReflectance; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}