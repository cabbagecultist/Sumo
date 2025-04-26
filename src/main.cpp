//==============================================================================
//                              SUMO BOT CONTROL SOFTWARE
//==============================================================================
/*
 * This program controls a sumo robot with multiple VL53L4CD distance sensors,
 * reflectance sensors for line detection, and a dual motor controller.
 *
 * Features:
 * - Multiple test modes for individual components
 * - Robust I2C error detection and recovery
 * - Sequential and parallel sensor reading capabilities
 */

#include <Arduino.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <VL53L4CD.h>
#include "DualVNH5019MotorShield.h"

//==============================================================================
// HARDWARE RECOMMENDATIONS FOR RELIABLE I2C OPERATION
//==============================================================================
/*
  For maximum reliability with VL53L4CD sensors:

  1. PULLUP RESISTORS
     - Add 2.2kΩ pullup resistors on both SDA and SCL lines to 3.3V
     - This value is stronger than typical 4.7kΩ pullups but helps overcome noise
     - Resistors should be placed near the ESP32 controller board

  2. NOISE SUPPRESSION
     - Add 100Ω series resistors on both SDA and SCL lines right after the pullups
     - Place 0.1μF ceramic capacitors between each sensor's VCC and GND pins
     - Add a larger 10μF capacitor on the power supply near where it connects to ESP32

  3. WIRING
     - Keep I2C wires as short as possible, ideally under 10cm
     - Use twisted pairs for SDA/SCL if longer wires are needed
     - Keep I2C wires away from motor wires and other noise sources
     - Ensure solid ground connections between ESP32 and all sensors

  4. XSHUT PIN HANDLING (CRITICAL)
     - NEVER drive XSHUT pins HIGH directly from ESP32
     - The Pololu carrier boards have onboard pullups for XSHUT
     - Only set XSHUT pins as either OUTPUT LOW (to disable) or INPUT (to enable)
*/

//==============================================================================
// CONSTANTS AND PIN DEFINITIONS
//==============================================================================

// I2C Configuration
#define I2C_SDA_PIN 21                // ESP32 SDA pin
#define I2C_SCL_PIN 22                // ESP32 SCL pin
#define I2C_CLOCK_SPEED_NORMAL 100000 // 100kHz for normal operation
#define I2C_CLOCK_SPEED_INIT 50000    // 50kHz for initialization (more reliable)
#define I2C_DEFAULT_TIMEOUT 500       // Default I2C timeout in milliseconds

// I2C Recovery Constants
#define I2C_CLOCK_PULSE_COUNT 16      // Number of clock pulses to send when resetting bus
#define I2C_PULSE_DELAY_US 5          // Microseconds between clock transitions
#define USE_FAST_I2C_RESET false      // Set to true to use the faster I2C reset method

// VL53L4CD Distance Sensor Constants
#define VL53L4CD_DEFAULT_ADDRESS 0x52 // Default address before changing
#define VL53L4CD_ADDRESS_BASE 0x2A    // Base address for assigning unique addresses
#define DISTANCE_SENSOR_COUNT 5       // Number of distance sensors

// Reflectance Sensor Constants
#define REFLECTANCE_SENSOR_COUNT 4 // Number of reflectance sensors
#define REFLECTANCE_EMITTER_PIN 13 // Emitter pin for reflectance sensors

// Test Mode Timing Constants
#define RESET_INTERVAL_MS 2000         // Minimum time between I2C bus resets (ms)
#define SENSOR_SWITCH_INTERVAL_MS 1000 // Time between switching sensors in sequential test
#define SCAN_INTERVAL_MS 5000          // Time between I2C scans
#define READ_INTERVAL_MS 100           // Time between sensor readings to avoid flooding

//==============================================================================
// PIN ARRAYS AND GLOBAL VARIABLES
//==============================================================================

// Distance sensor XSHUT pins
const uint8_t DISTANCE_SENSOR_XSHUT_PINS[DISTANCE_SENSOR_COUNT] = {19, 16, 18, 5, 17};
VL53L4CD distanceSensors[DISTANCE_SENSOR_COUNT];

// Reflectance sensor pins
const uint8_t REFLECTANCE_SENSOR_PINS[REFLECTANCE_SENSOR_COUNT] = {4, 14, 27, 15};
uint16_t reflectanceSensorValues[REFLECTANCE_SENSOR_COUNT];

// Motor controller (pin assignments for Dual VNH5019 Shield)
DualVNH5019MotorShield motorController(26, 25, 33, 255, 254, 34, 35, 32, 253, 252);

// QTR reflectance sensor object
QTRSensors qtr;

//==============================================================================
// OPERATING MODES
//==============================================================================

enum TestMode
{
  TEST_DISTANCE,           // Test all distance sensors
  TEST_REFLECTANCE,        // Test all reflectance sensors
  TEST_MOTOR,              // Test motor control
  TEST_I2C_SCANNER,        // Scan I2C bus for devices
  TEST_SINGLE_SENSOR,      // Test one distance sensor at a time
  TEST_SEQUENTIAL_SENSORS, // Test each distance sensor sequentially
  TEST_INTEGRATED          // Test all systems simultaneously
};

// CONFIGURATION - Set the desired test mode here
TestMode CURRENT_MODE = TEST_INTEGRATED;

// For single sensor testing - change this value to test a different sensor (0-4)
const uint8_t SINGLE_SENSOR_INDEX = 0;

//==============================================================================
// STATE TRACKING VARIABLES
//==============================================================================

// Variables for single sensor test
static bool sensorInitialized = false;
static uint32_t lastResetTime = 0;

// Variables for sequential sensor test
static uint8_t currentSensor = 0;
static uint32_t lastSensorSwitchTime = 0;
static bool sequentialInitComplete = false;

//==============================================================================
// FUNCTION DECLARATIONS
//==============================================================================

// Setup functions
void setupReflectanceSensors();
void setupDistanceSensors();
void resetI2CBus();
void resetI2CBusFast();
bool testI2CAddress(byte address);
void printI2CStatus(byte status);
bool monitorI2CHealth();

// Test mode functions
void scanI2CBus();
void testSingleSensor();
void testSequentialSensors();
void testAllDistanceSensors();
void testReflectanceSensors();
void testMotorControl();
void testIntegratedSystems();

//==============================================================================
// SETUP FUNCTION
//==============================================================================

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize I2C communication
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_SPEED_NORMAL);

  Serial.println("SYSTEM INITIALIZED");
  Serial.println("--------------------------------------------------");

  // Run the appropriate test based on the selected mode
  switch (CURRENT_MODE)
  {
  case TEST_DISTANCE:
    setupDistanceSensors();
    break;

  case TEST_REFLECTANCE:
    setupReflectanceSensors();
    break;

  case TEST_MOTOR:
    motorController.init();
    motorController.setM1Brake(400);
    Serial.println("Motor test mode activated");
    break;

  case TEST_I2C_SCANNER:
    scanI2CBus();
    break;

  case TEST_SINGLE_SENSOR:
    testSingleSensor();
    break;

  case TEST_SEQUENTIAL_SENSORS:
    Serial.println("Initializing sensors for sequential testing");
    setupDistanceSensors();
    sequentialInitComplete = true;
    currentSensor = 0;
    lastSensorSwitchTime = millis();
    testSequentialSensors();
    break;
    
  case TEST_INTEGRATED:
    Serial.println("Initializing all systems for integrated testing");
    setupDistanceSensors();
    setupReflectanceSensors();
    motorController.init();
    motorController.setM1Brake(0);
    motorController.setM2Brake(0);
    Serial.println("All systems initialized for integrated testing");
    break;
  }
}

//==============================================================================
// MAIN LOOP
//==============================================================================

void loop()
{
  // Execute the appropriate test function based on current mode
  switch (CURRENT_MODE)
  {
  case TEST_DISTANCE:
    testAllDistanceSensors();
    break;

  case TEST_REFLECTANCE:
    testReflectanceSensors();
    break;

  case TEST_MOTOR:
    testMotorControl();
    break;

  case TEST_I2C_SCANNER:
    scanI2CBus();
    break;

  case TEST_SINGLE_SENSOR:
    testSingleSensor();
    break;

  case TEST_SEQUENTIAL_SENSORS:
    testSequentialSensors();
    break;
    
  case TEST_INTEGRATED:
    testIntegratedSystems();
    break;
  }

  // Periodically monitor I2C bus health
  static uint32_t lastI2CHealthCheck = 0;
  if (millis() - lastI2CHealthCheck > 5000)
  { // Check every 5 seconds
    if (!monitorI2CHealth())
    {
      // Bus had issues and was reset
      if (CURRENT_MODE == TEST_SEQUENTIAL_SENSORS || CURRENT_MODE == TEST_DISTANCE || CURRENT_MODE == TEST_INTEGRATED)
      {
        // Reinitialize sensors if in a test mode that uses them heavily
        setupDistanceSensors();
      }
    }
    lastI2CHealthCheck = millis();
  }
}

//==============================================================================
// SENSOR SETUP FUNCTIONS
//==============================================================================

/**
 * Sets up all distance sensors by:
 * 1. Power cycling all sensors
 * 2. Initializing each one sequentially
 * 3. Assigning unique addresses
 * 4. Configuring for continuous measurement
 */
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
        resetI2CBus();
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
    if (testI2CAddress(newAddress))
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

/**
 * Sets up the reflectance sensors for line detection
 */
void setupReflectanceSensors()
{
  Serial.println("Setting up reflectance sensors...");

  qtr.setTypeRC(); // Configure sensors as RC type
  qtr.setSensorPins(REFLECTANCE_SENSOR_PINS, REFLECTANCE_SENSOR_COUNT);
  qtr.setEmitterPin(REFLECTANCE_EMITTER_PIN);

  Serial.println("Reflectance sensors initialized");
  Serial.println("--------------------------------------------------");

  // Uncomment this block to perform calibration if needed
  /*
  Serial.println("Starting calibration...");
  for (uint16_t i = 0; i < 400; i++) {
      qtr.calibrate();
      delay(5);
  }

  Serial.println("Calibration complete");

  // Print calibration values
  Serial.print("Minimum values: ");
  for (uint8_t i = 0; i < REFLECTANCE_SENSOR_COUNT; i++) {
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(' ');
  }
  Serial.println();

  Serial.print("Maximum values: ");
  for (uint8_t i = 0; i < REFLECTANCE_SENSOR_COUNT; i++) {
      Serial.print(qtr.calibrationOn.maximum[i]);
      Serial.print(' ');
  }
  Serial.println();
  */
}

//==============================================================================
// TEST MODE FUNCTIONS
//==============================================================================

/**
 * Tests all distance sensors simultaneously, reading from each one
 * and handling any timeout conditions by resetting the I2C bus if needed.
 */
void testAllDistanceSensors()
{
  static uint32_t lastReset = 0;
  bool needsReset = false;

  // Read and print values from all distance sensors
  for (uint8_t i = 0; i < DISTANCE_SENSOR_COUNT; i++)
  {
    int16_t distance = distanceSensors[i].read();

    if (distanceSensors[i].timeoutOccurred())
    {
      Serial.print("TIMEOUT");
      needsReset = true;
    }
    else
    {
      Serial.print(distance);
    }
    Serial.print('\t');
  }
  Serial.println();

  // Reset the I2C bus if needed (not more often than once per RESET_INTERVAL_MS)
  if (needsReset && (millis() - lastReset > RESET_INTERVAL_MS))
  {
    Serial.println("Timeout detected, resetting I2C bus...");
    resetI2CBus();
    lastReset = millis();
    setupDistanceSensors();
  }
}

/**
 * Tests reflectance sensors by reading and displaying their values
 */
void testReflectanceSensors()
{
  // Read reflectance sensor values
  qtr.read(reflectanceSensorValues);

  // Print values
  for (uint8_t i = 0; i < REFLECTANCE_SENSOR_COUNT; i++)
  {
    Serial.print(reflectanceSensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();
}

/**
 * Tests motor control by generating a smooth oscillating speed pattern
 */
void testMotorControl()
{
  // Generate a smooth oscillating speed for motor testing
  int speed = (int)(cos(((float)millis()) * 0.001) * 400);
  Serial.println(speed);
  motorController.setM1Speed(speed);
  delay(2);
}

/**
 * Scans the I2C bus for connected devices and reports their addresses
 */
void scanI2CBus()
{
  Serial.println("Scanning I2C bus for devices...");
  byte error, address;
  int deviceCount = 0;

  // Try all possible I2C addresses
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      deviceCount++;
    }
    else if (error == 4)
    {
      Serial.printf("Unknown error at address 0x%02X\n", address);
    }
  }

  if (deviceCount == 0)
  {
    Serial.println("No I2C devices found");
  }
  else
  {
    Serial.printf("Found %d device(s)\n", deviceCount);
  }

  Serial.println("--------------------------------------------------");
  delay(SCAN_INTERVAL_MS); // Wait before next scan
}

/**
 * Tests a single distance sensor, handling initialization and readings
 */
void testSingleSensor()
{
  if (!sensorInitialized)
  {
    // Initialize just one sensor
    Serial.printf("Initializing only sensor %d for testing\n", SINGLE_SENSOR_INDEX);

    // Power cycle all sensors by driving XSHUT pins low
    for (uint8_t i = 0; i < DISTANCE_SENSOR_COUNT; i++)
    {
      pinMode(DISTANCE_SENSOR_XSHUT_PINS[i], OUTPUT);
      digitalWrite(DISTANCE_SENSOR_XSHUT_PINS[i], LOW);
    }
    delay(100);

    // Only activate the test sensor
    // IMPORTANT: Do NOT drive XSHUT HIGH directly (as the pin isn't level-shifted)
    // Instead, set it as INPUT and let the carrier board's pull-up resistors handle it
    pinMode(DISTANCE_SENSOR_XSHUT_PINS[SINGLE_SENSOR_INDEX], INPUT);
    delay(50);

    // Try to initialize with default address
    Serial.println("Attempting to initialize sensor with default address");

    if (distanceSensors[SINGLE_SENSOR_INDEX].init())
    {
      Serial.println("Sensor initialized successfully!");
      distanceSensors[SINGLE_SENSOR_INDEX].startContinuous();
      distanceSensors[SINGLE_SENSOR_INDEX].setTimeout(I2C_DEFAULT_TIMEOUT);
      sensorInitialized = true;
    }
    else
    {
      Serial.println("Failed to initialize sensor");
    }
  }
  else
  {
    // Read from the initialized sensor
    int16_t distance = distanceSensors[SINGLE_SENSOR_INDEX].read();
    if (distanceSensors[SINGLE_SENSOR_INDEX].timeoutOccurred())
    {
      Serial.println("Timeout occurred when reading sensor");

      // Try to recover if repeated timeouts (not more often than RESET_INTERVAL_MS)
      if (millis() - lastResetTime > RESET_INTERVAL_MS)
      {
        Serial.println("Attempting sensor recovery...");
        resetI2CBus();
        sensorInitialized = false; // Force re-initialization
        lastResetTime = millis();
      }
    }
    else
    {
      Serial.printf("Sensor reading: %d mm\n", distance);
    }
  }

  delay(READ_INTERVAL_MS); // Longer delay for single sensor test
}

/**
 * Tests each distance sensor one at a time, cycling through all of them
 */
void testSequentialSensors()
{
  // Test the current sensor
  if (currentSensor < DISTANCE_SENSOR_COUNT)
  {
    int16_t distance = distanceSensors[currentSensor].read();
    if (distanceSensors[currentSensor].timeoutOccurred())
    {
      Serial.printf("Sensor %d: TIMEOUT\n", currentSensor);

      // Try to recover the I2C bus if a timeout occurs (not more often than RESET_INTERVAL_MS)
      static uint32_t lastResetAttempt = 0;
      if (millis() - lastResetAttempt > RESET_INTERVAL_MS)
      {
        Serial.printf("Attempting to recover sensor %d\n", currentSensor);
        resetI2CBus();

        // Try reading again after reset
        delay(50);
        distance = distanceSensors[currentSensor].read();

        if (!distanceSensors[currentSensor].timeoutOccurred())
        {
          Serial.printf("Recovery successful! Sensor %d reading: %d mm\n", currentSensor, distance);
        }
        else
        {
          Serial.printf("Recovery failed for sensor %d\n", currentSensor);
        }

        lastResetAttempt = millis();
      }
    }
    else
    {
      Serial.printf("Sensor %d: %d mm\n", currentSensor, distance);
    }

    // Switch to next sensor after SENSOR_SWITCH_INTERVAL_MS
    if (millis() - lastSensorSwitchTime > SENSOR_SWITCH_INTERVAL_MS)
    {
      currentSensor = (currentSensor + 1) % DISTANCE_SENSOR_COUNT;
      lastSensorSwitchTime = millis();
      Serial.printf("\n--- Switching to sensor %d ---\n", currentSensor);
    }
  }

  delay(READ_INTERVAL_MS); // Don't flood the serial console
}

//==============================================================================
// I2C BUS UTILITIES
//==============================================================================

/**
 * Resets the I2C bus if it hangs by:
 * 1. Checking if SDA or SCL lines are stuck low
 * 2. Using special recovery for stuck SDA
 * 3. Sending stop condition and re-initializing the bus
 * 
 * This function uses the mode configured by USE_FAST_I2C_RESET to select between
 * the thorough, reliable reset method or a faster but potentially less robust method.
 */
void resetI2CBus()
{
  if (USE_FAST_I2C_RESET) {
    resetI2CBusFast();
  } else {
    Serial.println("Resetting I2C bus");
    Wire.end();
  
    // Check if SDA line is stuck low (common failure mode)
    pinMode(I2C_SDA_PIN, INPUT);
    pinMode(I2C_SCL_PIN, INPUT);
  
    bool sclHigh = digitalRead(I2C_SCL_PIN);
    bool sdaHigh = digitalRead(I2C_SDA_PIN);
  
    if (!sclHigh)
    {
      Serial.println("WARNING: SCL line is stuck LOW - severe bus error");
    }
  
    if (!sdaHigh)
    {
      Serial.println("WARNING: SDA line is stuck LOW - attempting special recovery");
  
      // Special recovery for stuck SDA: Force SCL cycles until SDA is released
      pinMode(I2C_SCL_PIN, OUTPUT_OPEN_DRAIN);
      digitalWrite(I2C_SCL_PIN, HIGH);
  
      // Toggle SCL up to 20 times to try to get slave to complete transaction
      for (int i = 0; i < 20 && !digitalRead(I2C_SDA_PIN); i++)
      {
        digitalWrite(I2C_SCL_PIN, LOW);
        delayMicroseconds(I2C_PULSE_DELAY_US);
        digitalWrite(I2C_SCL_PIN, HIGH);
        delayMicroseconds(I2C_PULSE_DELAY_US);
      }
  
      // If SDA is still low, we have a serious problem
      if (!digitalRead(I2C_SDA_PIN))
      {
        Serial.println("ERROR: Could not clear SDA line. Hardware problem likely!");
      }
      else
      {
        Serial.println("SDA line successfully released");
      }
    }
  
    // Standard bus recovery procedure
    pinMode(I2C_SDA_PIN, OUTPUT_OPEN_DRAIN);
    pinMode(I2C_SCL_PIN, OUTPUT_OPEN_DRAIN);
  
    // Pull up SCL to ensure it's high (when in open-drain mode)
    digitalWrite(I2C_SCL_PIN, HIGH);
    digitalWrite(I2C_SDA_PIN, HIGH);
    delayMicroseconds(I2C_PULSE_DELAY_US);
  
    // Toggle SCL multiple times to release stuck devices
    for (int i = 0; i < I2C_CLOCK_PULSE_COUNT; i++)
    {
      digitalWrite(I2C_SCL_PIN, LOW);
      delayMicroseconds(I2C_PULSE_DELAY_US);
      digitalWrite(I2C_SCL_PIN, HIGH);
      delayMicroseconds(I2C_PULSE_DELAY_US);
    }
  
    // Send STOP condition (SDA low->high while SCL is high)
    digitalWrite(I2C_SDA_PIN, LOW);
    delayMicroseconds(I2C_PULSE_DELAY_US);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(I2C_PULSE_DELAY_US);
    digitalWrite(I2C_SDA_PIN, HIGH);
    delayMicroseconds(I2C_PULSE_DELAY_US * 2);
  
    // Return pins to normal INPUT mode
    pinMode(I2C_SDA_PIN, INPUT);
    pinMode(I2C_SCL_PIN, INPUT);
  
    // Check if the bus is clear now
    if (!digitalRead(I2C_SDA_PIN) || !digitalRead(I2C_SCL_PIN))
    {
      Serial.println("WARNING: I2C bus lines still not HIGH after reset attempt");
    }
    else
    {
      Serial.println("I2C bus lines are both HIGH after reset");
    }
  
    // Reinitialize I2C bus
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_SPEED_NORMAL);
    delay(100); // Allow time for devices to reset
  
    Serial.println("I2C bus reset complete");
  }
}

/**
 * Fast I2C bus reset function - uses a more aggressive approach with minimal delays
 * This is experimental and may not work with all devices/setups, but can recover
 * much faster from transient bus issues.
 */
void resetI2CBusFast()
{
  Serial.println("Fast I2C reset...");
  Wire.end();
  
  // Quick check of bus state
  pinMode(I2C_SDA_PIN, INPUT);
  pinMode(I2C_SCL_PIN, INPUT);
  
  // Fast reset procedure - minimal timing, fewer checks
  pinMode(I2C_SDA_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(I2C_SCL_PIN, OUTPUT_OPEN_DRAIN);
  
  // Send 9 clock pulses (the minimum required to ensure the slave releases the bus)
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(1); // Minimal delay
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(1); // Minimal delay
  }
  
  // Quick STOP condition
  digitalWrite(I2C_SDA_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(I2C_SDA_PIN, HIGH);
  delayMicroseconds(2);
  
  // Reinitialize I2C bus immediately
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_SPEED_NORMAL);
  
  Serial.println("Fast I2C reset complete");
}

/**
 * Tests if a device responds at the given I2C address
 * @param address The I2C address to test
 * @return true if the device responds, false otherwise
 */
bool testI2CAddress(byte address)
{
  Wire.beginTransmission(address);
  byte status = Wire.endTransmission();

  Serial.printf("I2C test at address 0x%02X: ", address);
  printI2CStatus(status);

  return (status == 0);
}

/**
 * Prints human-readable I2C status codes
 * @param status The status code from Wire.endTransmission()
 */
void printI2CStatus(byte status)
{
  switch (status)
  {
  case 0:
    Serial.println("Success");
    break;
  case 1:
    Serial.println("Data too long");
    break;
  case 2:
    Serial.println("NACK on address (device not connected)");
    break;
  case 3:
    Serial.println("NACK on data");
    break;
  case 4:
    Serial.println("Other error");
    break;
  default:
    Serial.println("Unknown status");
    break;
  }
}

/**
 * Monitors I2C bus health and attempts recovery if needed
 * @return true if the bus is healthy, false if it needed recovery
 */
bool monitorI2CHealth()
{
  // Check if SDA and SCL lines are both high (idle bus state)
  pinMode(I2C_SDA_PIN, INPUT);
  pinMode(I2C_SCL_PIN, INPUT);

  bool sdaHigh = digitalRead(I2C_SDA_PIN);
  bool sclHigh = digitalRead(I2C_SCL_PIN);

  if (!sdaHigh || !sclHigh)
  {
    Serial.println("WARNING: I2C bus lines not in idle state (HIGH)");
    Serial.printf("SDA: %s, SCL: %s\n",
                  sdaHigh ? "HIGH" : "LOW",
                  sclHigh ? "HIGH" : "LOW");

    // Try to reset the bus
    resetI2CBus();
    return false;
  }

  // Test communication with at least one sensor
  bool deviceResponding = false;
  for (int i = 0; i < DISTANCE_SENSOR_COUNT; i++)
  {
    byte address = VL53L4CD_ADDRESS_BASE + i;
    Wire.beginTransmission(address);
    byte status = Wire.endTransmission();

    if (status == 0)
    {
      deviceResponding = true;
      break;
    }
  }

  if (!deviceResponding)
  {
    Serial.println("WARNING: No VL53L4CD sensors responding on I2C bus");
    resetI2CBus();
    return false;
  }

  return true;
}

/**
 * Integrated test for all systems - tests distance sensors, reflectance sensors, and motors simultaneously
 */
void testIntegratedSystems()
{
  static uint32_t lastDistanceSensorUpdate = 0;
  static uint32_t lastReflectanceSensorUpdate = 0;
  static uint32_t lastMotorUpdate = 0;
  static uint32_t lastI2CReset = 0;
  static bool distanceSensorError = false;
  
  unsigned long currentTime = millis();
  
  // Test distance sensors (less frequently to avoid bus congestion)
  if (currentTime - lastDistanceSensorUpdate > 100) {
    Serial.println("--- DISTANCE SENSORS ---");
    bool sensorTimeout = false;
    
    for (uint8_t i = 0; i < DISTANCE_SENSOR_COUNT; i++) {
      int16_t distance = distanceSensors[i].read();
      
      if (distanceSensors[i].timeoutOccurred()) {
        Serial.printf("Sensor %d: TIMEOUT\n", i);
        sensorTimeout = true;
        distanceSensorError = true;
      } else {
        Serial.printf("Sensor %d: %d mm\n", i, distance);
      }
    }
    
    // Reset I2C bus if needed and not reset recently
    if (sensorTimeout && (currentTime - lastI2CReset > 1000)) {
      Serial.println("Timeout detected, performing fast I2C reset...");
      resetI2CBus();
      lastI2CReset = currentTime;
    }
    
    lastDistanceSensorUpdate = currentTime;
  }
  
  // Test reflectance sensors
  if (currentTime - lastReflectanceSensorUpdate > 50) {
    Serial.println("--- REFLECTANCE SENSORS ---");
    qtr.read(reflectanceSensorValues);
    
    for (uint8_t i = 0; i < REFLECTANCE_SENSOR_COUNT; i++) {
      Serial.printf("R%d: %d\t", i, reflectanceSensorValues[i]);
    }
    Serial.println();
    
    lastReflectanceSensorUpdate = currentTime;
  }
  
  // Test motors - gentle movement pattern
  if (currentTime - lastMotorUpdate > 250) {
    // Only run motors if no sensor errors to avoid dangerous movement with no sensing
    if (!distanceSensorError) {
      int speedM1 = (int)(sin(((float)currentTime) * 0.001) * 200);
      int speedM2 = (int)(cos(((float)currentTime) * 0.001) * 200);
      
      Serial.printf("--- MOTORS --- M1: %d  M2: %d\n", speedM1, speedM2);
      
      // motorController.setM1Speed(speedM1);
      // motorController.setM2Speed(speedM2);
      motorController.setM1Speed(0);
      motorController.setM2Speed(0);
    } else {
      // Safety stop if sensors are having issues
      motorController.setM1Brake(400);
      motorController.setM2Brake(400);
      Serial.println("--- MOTORS --- EMERGENCY STOP (sensor error)");
    }
    
    lastMotorUpdate = currentTime;
  }
  
  // Small delay to prevent serial output flooding
  delay(10);
}
