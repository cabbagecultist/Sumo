#include <Arduino.h>
#include <VL53L4CD.h>
#include <I2CHelper.h>
#include <Constants.h>

// Sensor Definitions
const uint8_t DISTANCE_SENSOR_XSHUT_PINS[DISTANCE_SENSOR_COUNT] = {19, 16, 18, 5, 17};
VL53L4CD distanceSensors[DISTANCE_SENSOR_COUNT];

// Setup Functions
void setupDistanceSensors();

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize I2C communication
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_SPEED_NORMAL);

  setupDistanceSensors();
}

void loop() {
}

void setupDistanceSensors()
{
  Serial.println("Starting VL53L4CD sensor initialization...");

  // Remind user about pullup resistors
  Serial.println("NOTE: For reliable I2C with multiple VL53L4CD sensors, add");
  Serial.println("2.2-4.7kΩ pullup resistors to both SDA and SCL lines if not already done.");

  // Power cycle all sensors by driving XSHUT pins low
  Serial.println("Power cycling all distance sensors...");
  for (uint8_t i = 0; i < DISTANCE_SENSOR_COUNT; i++)
  {
    pinMode(DISTANCE_SENSOR_XSHUT_PINS[i], OUTPUT);
    digitalWrite(DISTANCE_SENSOR_XSHUT_PINS[i], LOW);
  }
  delay(100); // Power-down time

  // Initialize each sensor one by one
  for (uint8_t i = 0; i < DISTANCE_SENSOR_COUNT; i++)
  {
    Serial.printf("Initializing distance sensor %d...\n", i);

    // Activate only this sensor
    // IMPORTANT: Do NOT drive XSHUT HIGH directly (as the pin isn't level-shifted)
    // Instead, set it as INPUT and let the carrier board's pull-up resistors handle it
    pinMode(DISTANCE_SENSOR_XSHUT_PINS[i], INPUT); // Allow carrier board to pull XSHUT high
    delay(50);                                     // Boot delay

    // Slow down I2C for initialization
    Wire.setClock(I2C_CLOCK_SPEED_INIT);

    // Multiple init attempts
    bool initialized = false;
    for (int attempts = 0; attempts < 3 && !initialized; attempts++)
    {
      Serial.printf("Attempt %d for sensor %d\n", attempts + 1, i);

      initialized = distanceSensors[i].init();

      if (!initialized)
      {
        Serial.printf("Failed attempt %d to initialize sensor %d\n", attempts + 1, i);
        delay(50);
        I2CHelper::resetI2CBus();
        delay(50);
      }
    }

    if (!initialized)
    {
      Serial.printf("Failed to initialize sensor %d after multiple attempts\n", i);
      continue;
    }

    Serial.printf("Successfully initialized sensor %d\n", i);

    // Set unique address
    byte newAddress = VL53L4CD_ADDRESS_BASE + i; // Create unique address for each sensor
    distanceSensors[i].setAddress(newAddress);
    Serial.printf("Set address 0x%02X for sensor %d\n", newAddress, i);

    // Verify communication with the new address
    if (I2CHelper::testI2CAddress(newAddress))
    {
      Serial.printf("Successfully verified communication at address 0x%02X\n", newAddress);
    }

    // Configure sensor for continuous measurement
    distanceSensors[i].startContinuous();
    distanceSensors[i].setTimeout(I2C_DEFAULT_TIMEOUT); // Set timeout

    // Test the sensor
    int16_t testReading = distanceSensors[i].read();
    if (distanceSensors[i].timeoutOccurred())
    {
      Serial.printf("Warning: Timeout occurred when testing sensor %d\n", i);
    }
    else
    {
      Serial.printf("Sensor %d test reading: %d mm\n", i, testReading);
    }
  }

  // Restore normal I2C speed
  Wire.setClock(I2C_CLOCK_SPEED_NORMAL);

  Serial.println("Distance sensor initialization completed");
  Serial.println("--------------------------------------------------");
}








