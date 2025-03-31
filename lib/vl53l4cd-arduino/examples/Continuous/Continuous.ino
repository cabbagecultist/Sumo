/*
This example shows how to take simple range measurements with the VL53L4CD. The
range readings are in units of mm.
*/

#include <Wire.h>
#include <VL53L4CD.h>

VL53L4CD sensor;

void setup()
{
  while (!Serial) {}
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  // The sensor is configured with a default timing budget of 50 ms and
  // inter-measurement period of 0 (continuous ranging). You can optionally
  // uncomment one of the following lines to use one of the example
  // configurations listed in ST's VL53L4CD ULD user manual UM2931 (section 2.4,
  // Example of configurations) or call setRangeTiming() with your own values.
  //
  // According to UM2931 table 4, Minimum and maximum range timing (in section
  // 3.2, Range timing):
  // - Timing budget can be 10 to 200 ms.
  // - Inter-measurement can be 0 for continuous ranging (as often as the timing
  //   budget allows) or else must be greater than the timing budget, up to a
  //   maximum of 5000 ms.
  //sensor.setRangeTiming(10, 0); // fast ranging (10 ms budget continuously)
  //sensor.setRangeTiming(200, 0); // high accuracy (200 ms budget continuously)
  //sensor.setRangeTiming(50, 1000); // autonomous low power (50 ms budget, 1 reading every second)

  // Start continuous readings.
  sensor.startContinuous();
}

void loop()
{
  Serial.print(sensor.read());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
}
