//==============================================================================
//                     SUMO BOT CONTROL SOFTWARE
//==============================================================================
/*
 * This program controls a sumo robot with:
 * - Multiple VL53L4CD distance sensors for opponent detection
 * - QTR reflectance sensors for edge/line detection
 * - Dual VNH5019 motor controller for movement
 *
 * Features:
 * - Multiple test modes for individual component testing
 * - Robust I2C error detection and recovery mechanisms
 * - Integrated testing for all systems simultaneously
 */

#include <Arduino.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <VL53L4CD.h>
#include "DualVNH5019MotorShield.h"

//==============================================================================
// CONFIGURATION - ADJUST THESE SETTINGS AS NEEDED
//==============================================================================

// Test Mode Selection
enum TestMode
{
  TEST_DISTANCE,           // Test only distance sensors
  TEST_REFLECTANCE,        // Test only reflectance sensors
  TEST_MOTOR,              // Test only motor control
  TEST_I2C_SCANNER,        // Scan I2C bus for devices
  TEST_SINGLE_SENSOR,      // Test one distance sensor at a time
  TEST_SEQUENTIAL_SENSORS, // Test each distance sensor sequentially
  TEST_ALL_SENSORS         // Test all systems simultaneously (integrated)
};

// ACTIVE TEST MODE - Change this to select mode
TestMode CURRENT_MODE = TEST_ALL_SENSORS;

// Single sensor testing - which sensor to test (0-4)
const uint8_t SINGLE_SENSOR_INDEX = 0;

// I2C Settings
#define I2C_SDA_PIN 21                // ESP32 SDA pin
#define I2C_SCL_PIN 22                // ESP32 SCL pin
#define I2C_CLOCK_SPEED_NORMAL 400000 // 400kHz for normal operation
#define I2C_CLOCK_SPEED_INIT 50000    // 50kHz for initialization (more reliable)
#define I2C_DEFAULT_TIMEOUT 100       // Default I2C timeout in milliseconds
#define USE_FAST_I2C_RESET false      // Set to true for faster but less robust I2C reset

// VL53L4CD Distance Sensor Configuration
#define VL53L4CD_DEFAULT_ADDRESS 0x52 // Default address before changing
#define VL53L4CD_ADDRESS_BASE 0x2A    // Base address for assigning unique addresses
#define DISTANCE_SENSOR_COUNT 5       // Number of distance sensors

// Reflectance Sensor Configuration
#define REFLECTANCE_SENSOR_COUNT 2 // Number of reflectance sensors
#define REFLECTANCE_EMITTER_PIN 13 // Emitter pin (not used with ESP32 due to input-only pins)

// Timing Constants
#define RESET_INTERVAL_MS 2000         // Minimum time between I2C bus resets
#define SENSOR_SWITCH_INTERVAL_MS 1000 // Time between switching sensors in sequential test
#define SCAN_INTERVAL_MS 5000          // Time between I2C scans
#define READ_INTERVAL_MS 100           // Time between sensor readings

// I2C Recovery Constants
#define I2C_CLOCK_PULSE_COUNT 16 // Pulses for thorough reset
#define I2C_PULSE_DELAY_US 5     // Delay between clock transitions

//==============================================================================
// PIN ASSIGNMENTS
//==============================================================================

// Distance sensor XSHUT pins (to control sensor power)
const uint8_t DISTANCE_SENSOR_XSHUT_PINS[DISTANCE_SENSOR_COUNT] = {16, 17, 5, 18, 19};

// Reflectance sensor pins - IMPORTANT: ESP32 pins 34-39 are INPUT ONLY
// If using RC mode with QTR sensors, pins must support both INPUT and OUTPUT
const uint8_t REFLECTANCE_SENSOR_PINS[REFLECTANCE_SENSOR_COUNT] = {4, 2}; // Pin 0 and 15 available as backup

// Motor controller pins for Dual VNH5019 Shield
// (INA, INB, PWM, DIAG, CS, INB, INA, PWM, DIAG, CS)
DualVNH5019MotorShield motorController(27, 14, 12, 255, 254, 33, 25, 26, 253, 252);

//==============================================================================
// GLOBAL VARIABLES
//==============================================================================

// Distance sensor objects
VL53L4CD distanceSensors[DISTANCE_SENSOR_COUNT];

// Reflectance sensor values
uint16_t reflectanceSensorValues[REFLECTANCE_SENSOR_COUNT];

// Reflectance sensor object
QTRSensors qtr;

// State tracking variables
static bool sensorInitialized = false;
static uint32_t lastResetTime = 0;
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
  delay(500); // Short delay to ensure serial is ready

  // Initialize I2C communication
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_SPEED_NORMAL);

  Serial.println("===== SUMO BOT SYSTEM INITIALIZED =====");
  Serial.println("--------------------------------------------------");

  // Initialize components based on selected test mode
  switch (CURRENT_MODE)
  {
  case TEST_DISTANCE:
    Serial.println("MODE: DISTANCE SENSORS TEST");
    setupDistanceSensors();
    break;

  case TEST_REFLECTANCE:
    Serial.println("MODE: REFLECTANCE SENSORS TEST");
    setupReflectanceSensors();
    break;

  case TEST_MOTOR:
    Serial.println("MODE: MOTOR CONTROL TEST");
    motorController.init();
    motorController.setM1Brake(400);
    motorController.setM2Brake(400);
    break;

  case TEST_I2C_SCANNER:
    Serial.println("MODE: I2C BUS SCANNER");
    scanI2CBus();
    break;

  case TEST_SINGLE_SENSOR:
    Serial.println("MODE: SINGLE SENSOR TEST");
    Serial.printf("Testing sensor index: %d\n", SINGLE_SENSOR_INDEX);
    testSingleSensor();
    break;

  case TEST_SEQUENTIAL_SENSORS:
    Serial.println("MODE: SEQUENTIAL SENSOR TEST");
    setupDistanceSensors();
    sequentialInitComplete = true;
    currentSensor = 0;
    lastSensorSwitchTime = millis();
    testSequentialSensors();
    break;

  case TEST_ALL_SENSORS:
    Serial.println("MODE: INTEGRATED SYSTEMS TEST");
    setupDistanceSensors();
    setupReflectanceSensors();
    motorController.init();
    motorController.setM1Brake(0);
    motorController.setM2Brake(0);
    break;
  }

  Serial.println("Setup complete - entering main loop");
  Serial.println("--------------------------------------------------");
}

//==============================================================================
// MAIN LOOP
//==============================================================================

void loop()
{
  // Execute test function based on selected mode
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

  case TEST_ALL_SENSORS:
    testIntegratedSystems();
    break;
  }

  // Note: I2C health monitoring is intentionally commented out since current code works well
  /*
  static uint32_t lastI2CHealthCheck = 0;
  if (millis() - lastI2CHealthCheck > 5000) { // Check every 5 seconds
    if (!monitorI2CHealth()) {
      // Bus had issues and was reset
      if (CURRENT_MODE == TEST_SEQUENTIAL_SENSORS || CURRENT_MODE == TEST_DISTANCE ||
          CURRENT_MODE == TEST_ALL_SENSORS) {
        setupDistanceSensors();
      }
    }
    lastI2CHealthCheck = millis();
  }
  */
}

//==============================================================================
// SENSOR SETUP FUNCTIONS
//==============================================================================

/**
 * Sets up distance sensors:
 * 1. Power cycles all sensors by controlling XSHUT pins
 * 2. Initializes each sensor one by one
 * 3. Assigns unique I2C addresses to each sensor
 * 4. Configures sensors for continuous measurement
 */
void setupDistanceSensors()
{
  Serial.println("Starting VL53L4CD distance sensor initialization...");

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

    // Activate only this sensor with XSHUT pin
    pinMode(DISTANCE_SENSOR_XSHUT_PINS[i], INPUT); // Allow carrier board to pull XSHUT high
    delay(50);                                     // Boot delay

    // Slow down I2C for more reliable initialization
    Wire.setClock(I2C_CLOCK_SPEED_INIT);

    // Multiple initialization attempts
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

    // Assign unique address to this sensor
    byte newAddress = VL53L4CD_ADDRESS_BASE + i;
    distanceSensors[i].setAddress(newAddress);
    Serial.printf("Set address 0x%02X for sensor %d\n", newAddress, i);

    // Verify communication with the new address
    if (testI2CAddress(newAddress))
    {
      Serial.printf("Verified communication at address 0x%02X\n", newAddress);
    }

    // Configure for continuous measurement
    distanceSensors[i].startContinuous();
    distanceSensors[i].setTimeout(I2C_DEFAULT_TIMEOUT);

    // Test reading
    int16_t testReading = distanceSensors[i].read();
    if (distanceSensors[i].timeoutOccurred())
    {
      Serial.printf("Warning: Timeout for sensor %d\n", i);
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
 * Sets up reflectance sensors for edge detection
 * Note: For ESP32, pins 34-39 are input-only and cannot be used in RC mode
 */
void setupReflectanceSensors()
{
  Serial.println("Setting up reflectance sensors...");

  qtr.setTypeRC(); // Configure as RC sensors
  qtr.setSensorPins(REFLECTANCE_SENSOR_PINS, REFLECTANCE_SENSOR_COUNT);
  // Emitter pin is not used/needed for this configuration

  Serial.println("Reflectance sensors initialized");
  Serial.println("--------------------------------------------------");
}

//==============================================================================
// TEST MODE FUNCTIONS
//==============================================================================

/**
 * Tests all distance sensors simultaneously
 * Handles timeouts by resetting I2C bus when needed
 */
void testAllDistanceSensors()
{
  static uint32_t lastReset = 0;
  bool needsReset = false;

  // Read and display values from all distance sensors
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
      if (distanceSensors[i].ranging_data.range_status == 0)
      {
        Serial.printf("%7d", distance);
      }
      else
      {
        Serial.printf("NOTHING");
      }
    }
    Serial.print('\t');
  }
  Serial.println();

  // Reset I2C bus if needed (with cooldown period)
  if (needsReset && (millis() - lastReset > RESET_INTERVAL_MS))
  {
    Serial.println("Timeout detected, resetting I2C bus...");
    resetI2CBus();
    lastReset = millis();
  }
}

/**
 * Tests reflectance sensors and displays if black or white is detected
 */
void testReflectanceSensors()
{
  // Read reflectance values
  qtr.read(reflectanceSensorValues);

  // Interpret readings as black/white
  bool left = reflectanceSensorValues[0] < 1000;
  bool right = reflectanceSensorValues[1] < 1000;

  Serial.printf("LEFT: %5s (%4d) RIGHT: %5s (%4d)\n",
                left ? "White" : "Black",
                reflectanceSensorValues[0],
                right ? "White" : "Black",
                reflectanceSensorValues[1]);
}

/**
 * Tests motors with a gentle oscillating pattern
 */
void testMotorControl()
{
  int speed = (int)(cos(((float)millis()) * 0.001) * 400);
  Serial.println(speed);
  motorController.setM1Speed(speed);
  delay(2);
}

/**
 * Scans the I2C bus for all connected devices
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
  delay(SCAN_INTERVAL_MS);
}

/**
 * Integrated test for all systems simultaneously
 * Tests distance sensors and reflectance sensors, with motor capability
 */
void testIntegratedSystems()
{
  static uint32_t lastReset = 0;
  bool needsReset = false;

  // Test distance sensors
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
      if (distanceSensors[i].ranging_data.range_status == 0)
      {
        Serial.printf("%7d", distance);
      }
      else
      {
        Serial.printf("NOTHING");
      }
    }
    Serial.print('\t');
  }

  // Test reflectance sensors
  qtr.read(reflectanceSensorValues);
  bool left = reflectanceSensorValues[0] < 1000;
  bool right = reflectanceSensorValues[1] < 1000;
  Serial.printf(" | LEFT: %5s (%4d) RIGHT: %5s (%4d)\n",
                left ? "White" : "Black",
                reflectanceSensorValues[0],
                right ? "White" : "Black",
                reflectanceSensorValues[1]);

  // Reset the I2C bus if needed
  if (needsReset && (millis() - lastReset > RESET_INTERVAL_MS))
  {
    Serial.println("Timeout detected, resetting I2C bus...");
    resetI2CBus();
    lastReset = millis();
  }

  // Small delay to prevent serial output flooding
  delay(10);
}

/**
 * Tests a single distance sensor
 * Useful for isolating and diagnosing sensor issues
 */
void testSingleSensor()
{
  if (!sensorInitialized)
  {
    // Initialize just one sensor for testing
    Serial.printf("Initializing only sensor %d for testing\n", SINGLE_SENSOR_INDEX);

    // Power cycle all sensors
    for (uint8_t i = 0; i < DISTANCE_SENSOR_COUNT; i++)
    {
      pinMode(DISTANCE_SENSOR_XSHUT_PINS[i], OUTPUT);
      digitalWrite(DISTANCE_SENSOR_XSHUT_PINS[i], LOW);
    }
    delay(100);

    // Activate only the test sensor
    pinMode(DISTANCE_SENSOR_XSHUT_PINS[SINGLE_SENSOR_INDEX], INPUT);
    delay(50);

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

      // Recovery attempt with cooldown
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

  delay(READ_INTERVAL_MS);
}

/**
 * Tests each distance sensor one at a time in sequence
 * Useful for isolating which sensor might be causing issues
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

      // Attempt recovery with cooldown
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

    // Switch to next sensor after interval
    if (millis() - lastSensorSwitchTime > SENSOR_SWITCH_INTERVAL_MS)
    {
      currentSensor = (currentSensor + 1) % DISTANCE_SENSOR_COUNT;
      lastSensorSwitchTime = millis();
      Serial.printf("\n--- Switching to sensor %d ---\n", currentSensor);
    }
  }

  delay(READ_INTERVAL_MS);
}

//==============================================================================
// I2C BUS MANAGEMENT FUNCTIONS
//==============================================================================

/**
 * Resets the I2C bus when it hangs or has communication issues
 * This function selects between the thorough or fast reset method
 * based on the USE_FAST_I2C_RESET configuration
 */
void resetI2CBus()
{
  if (USE_FAST_I2C_RESET)
  {
    resetI2CBusFast();
  }
  else
  {
    Serial.println("Resetting I2C bus (thorough method)");
    Wire.end();

    // Check bus state
    pinMode(I2C_SDA_PIN, INPUT);
    pinMode(I2C_SCL_PIN, INPUT);
    bool sclHigh = digitalRead(I2C_SCL_PIN);
    bool sdaHigh = digitalRead(I2C_SDA_PIN);

    // Report bus condition
    if (!sclHigh)
    {
      Serial.println("WARNING: SCL line is stuck LOW - severe bus error");
    }

    if (!sdaHigh)
    {
      Serial.println("WARNING: SDA line is stuck LOW - attempting special recovery");

      // Special recovery for stuck SDA: Force SCL cycles to free SDA
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

      // Check if SDA was freed
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

    // Pull up lines to ensure they're high
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

    // Verify bus is clear
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
 * Fast I2C bus reset function with minimal delays
 * This is experimental and may not work with all devices
 * but recovers more quickly from transient issues
 */
void resetI2CBusFast()
{
  Serial.println("Fast I2C reset...");
  Wire.end();

  // Quick pin setup
  pinMode(I2C_SDA_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(I2C_SCL_PIN, OUTPUT_OPEN_DRAIN);

  // Send 9 clock pulses (minimum required to free bus)
  for (int i = 0; i < 9; i++)
  {
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

  // Restore pins to input mode
  pinMode(I2C_SDA_PIN, INPUT);
  pinMode(I2C_SCL_PIN, INPUT);

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
 * Provides human-readable descriptions of I2C status codes
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
 * This function is intentionally not called in the current code
 * but is preserved for debugging purposes if needed
 *
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

    // Reset the bus
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
