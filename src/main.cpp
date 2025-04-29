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
 * - Object-oriented architecture for better maintainability
 */

#include <Arduino.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <VL53L4CD.h>
#include "DualVNH5019MotorShield.h"

//==============================================================================
// CONFIGURATION - ADJUST THESE SETTINGS AS NEEDED
//==============================================================================

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

// PID Controller Settings
#define PID_KP 100.0                 // Proportional gain
#define PID_KI 0.0                  // Integral gain
#define PID_KD 0.0                  // Derivative gain
#define PID_MAX_OUTPUT 400          // Maximum output value for motors
#define PID_MIN_VALID_DISTANCE 10   // Minimum valid distance (mm)
#define PID_MAX_VALID_DISTANCE 1200 // Maximum valid distance (mm)

//==============================================================================
// PIN ASSIGNMENTS
//==============================================================================

// Distance sensor XSHUT pins (to control sensor power)
const uint8_t DISTANCE_SENSOR_XSHUT_PINS[DISTANCE_SENSOR_COUNT] = {16, 17, 5, 18, 19};

// Reflectance sensor pins - IMPORTANT: ESP32 pins 34-39 are INPUT ONLY
// If using RC mode with QTR sensors, pins must support both INPUT and OUTPUT
const uint8_t REFLECTANCE_SENSOR_PINS[REFLECTANCE_SENSOR_COUNT] = {4, 2}; // Pin 0 and 15 available as backup

//==============================================================================
// CLASS DECLARATIONS
//==============================================================================

// Forward declarations of classes
class I2CManager;
class DistanceSensorArray;
class PIDController;
class MotorDriver;
class ReflectanceSensorArray;
class TestMode;
class SumoBot;

/**
 * I2C bus manager class that handles initialization, reset, and recovery
 */
class I2CManager
{
private:
  const int sdaPin;
  const int sclPin;
  const unsigned long clockSpeedNormal;
  const unsigned long clockSpeedInit;
  unsigned long lastResetTime = 0;
  bool useFastReset;

public:
  I2CManager(int sda, int scl, unsigned long normal, unsigned long init, bool fast = false)
      : sdaPin(sda), sclPin(scl), clockSpeedNormal(normal), clockSpeedInit(init), useFastReset(fast) {}

  void begin()
  {
    Wire.begin(sdaPin, sclPin);
    Wire.setClock(clockSpeedNormal);
    Serial.printf("I2C initialized on pins SDA=%d, SCL=%d at %d Hz\n",
                  sdaPin, sclPin, clockSpeedNormal);
  }

  void setSlowSpeed()
  {
    Wire.setClock(clockSpeedInit);
  }

  void setNormalSpeed()
  {
    Wire.setClock(clockSpeedNormal);
  }

  bool resetBus()
  {
    unsigned long now = millis();
    if (now - lastResetTime < RESET_INTERVAL_MS)
    {
      return false; // Don't reset too frequently
    }

    if (useFastReset)
    {
      resetBusFast();
    }
    else
    {
      resetBusThorough();
    }

    lastResetTime = now;
    return true;
  }

  bool scanBus()
  {
    Serial.println("Scanning I2C bus for devices...");
    byte error, address;
    int deviceCount = 0;

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
      return false;
    }
    else
    {
      Serial.printf("Found %d device(s)\n", deviceCount);
      return true;
    }
  }

  bool testAddress(byte address)
  {
    Wire.beginTransmission(address);
    byte status = Wire.endTransmission();

    Serial.printf("I2C test at address 0x%02X: ", address);
    switch (status)
    {
    case 0:
      Serial.println("Success");
      break;
    case 1:
      Serial.println("Data too long");
      break;
    case 2:
      Serial.println("NACK on address");
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

    return (status == 0);
  }

private:
  void resetBusThorough()
  {
    Serial.println("Resetting I2C bus (thorough method)");
    Wire.end();

    // Check bus state
    pinMode(sdaPin, INPUT);
    pinMode(sclPin, INPUT);
    bool sclHigh = digitalRead(sclPin);
    bool sdaHigh = digitalRead(sdaPin);

    // Report bus condition
    if (!sclHigh)
    {
      Serial.println("WARNING: SCL line is stuck LOW - severe bus error");
    }

    if (!sdaHigh)
    {
      Serial.println("WARNING: SDA line is stuck LOW - attempting special recovery");

      pinMode(sclPin, OUTPUT_OPEN_DRAIN);
      digitalWrite(sclPin, HIGH);

      // Toggle SCL up to 20 times to try to get slave to complete transaction
      for (int i = 0; i < 20 && !digitalRead(sdaPin); i++)
      {
        digitalWrite(sclPin, LOW);
        delayMicroseconds(I2C_PULSE_DELAY_US);
        digitalWrite(sclPin, HIGH);
        delayMicroseconds(I2C_PULSE_DELAY_US);
      }

      if (!digitalRead(sdaPin))
      {
        Serial.println("ERROR: Could not clear SDA line. Hardware problem likely!");
      }
      else
      {
        Serial.println("SDA line successfully released");
      }
    }

    // Standard bus recovery procedure
    pinMode(sdaPin, OUTPUT_OPEN_DRAIN);
    pinMode(sclPin, OUTPUT_OPEN_DRAIN);

    // Pull up lines to ensure they're high
    digitalWrite(sclPin, HIGH);
    digitalWrite(sdaPin, HIGH);
    delayMicroseconds(I2C_PULSE_DELAY_US);

    // Toggle SCL multiple times to release stuck devices
    for (int i = 0; i < I2C_CLOCK_PULSE_COUNT; i++)
    {
      digitalWrite(sclPin, LOW);
      delayMicroseconds(I2C_PULSE_DELAY_US);
      digitalWrite(sclPin, HIGH);
      delayMicroseconds(I2C_PULSE_DELAY_US);
    }

    // Send STOP condition (SDA low->high while SCL is high)
    digitalWrite(sdaPin, LOW);
    delayMicroseconds(I2C_PULSE_DELAY_US);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(I2C_PULSE_DELAY_US);
    digitalWrite(sdaPin, HIGH);
    delayMicroseconds(I2C_PULSE_DELAY_US * 2);

    // Return pins to normal INPUT mode
    pinMode(sdaPin, INPUT);
    pinMode(sclPin, INPUT);

    // Verify bus is clear
    if (!digitalRead(sdaPin) || !digitalRead(sclPin))
    {
      Serial.println("WARNING: I2C bus lines still not HIGH after reset attempt");
    }
    else
    {
      Serial.println("I2C bus lines are both HIGH after reset");
    }

    // Reinitialize I2C bus
    Wire.begin(sdaPin, sclPin);
    Wire.setClock(clockSpeedNormal);
    delay(100); // Allow time for devices to reset

    Serial.println("I2C bus reset complete");
  }

  void resetBusFast()
  {
    Serial.println("Fast I2C reset...");
    Wire.end();

    // Quick pin setup
    pinMode(sdaPin, OUTPUT_OPEN_DRAIN);
    pinMode(sclPin, OUTPUT_OPEN_DRAIN);

    // Send 9 clock pulses (minimum required to free bus)
    for (int i = 0; i < 9; i++)
    {
      digitalWrite(sclPin, LOW);
      delayMicroseconds(1);
      digitalWrite(sclPin, HIGH);
      delayMicroseconds(1);
    }

    // Quick STOP condition
    digitalWrite(sdaPin, LOW);
    delayMicroseconds(1);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(sdaPin, HIGH);
    delayMicroseconds(2);

    // Restore pins to input mode
    pinMode(sdaPin, INPUT);
    pinMode(sclPin, INPUT);

    // Reinitialize I2C bus immediately
    Wire.begin(sdaPin, sclPin);
    Wire.setClock(clockSpeedNormal);

    Serial.println("Fast I2C reset complete");
  }
};

/**
 * PID Controller class for closed-loop control
 */
class PIDController
{
private:
  float kp;
  float ki;
  float kd;
  float maxOutput;

  float integral = 0;
  float lastError = 0;
  unsigned long lastTime = 0;

public:
  PIDController(float p, float i, float d, float max)
      : kp(p), ki(i), kd(d), maxOutput(max)
  {
    reset();
  }

  void reset()
  {
    integral = 0;
    lastError = 0;
    lastTime = millis();
  }

  float calculate(float error)
  {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0; // Convert to seconds
    if (dt <= 0)
      dt = 0.01; // Prevent division by zero

    // Proportional term
    float proportional = kp * error;

    // Integral term
    integral += error * dt;
    float integralTerm = ki * integral;

    // Derivative term
    float derivative = 0;
    if (dt > 0)
    {
      derivative = kd * (error - lastError) / dt;
    }

    // Calculate total output
    float output = proportional + integralTerm + derivative;

    // Update state variables
    lastError = error;
    lastTime = now;

    // Constrain output to limits
    if (output > maxOutput)
      output = maxOutput;
    if (output < -maxOutput)
      output = -maxOutput;

    return (int)output;
  }
};

/**
 * Motor driver class to control the robot motors
 */
class MotorDriver
{
private:
  DualVNH5019MotorShield shield;

public:
  MotorDriver(int ina1, int inb1, int pwm1, int diag1, int cs1,
              int inb2, int ina2, int pwm2, int diag2, int cs2)
      : shield(ina1, inb1, pwm1, diag1, cs1, inb2, ina2, pwm2, diag2, cs2) {}

  void init()
  {
    shield.init();
  }

  void setSpeeds(int leftSpeed, int rightSpeed)
  {
    shield.setM1Speed(leftSpeed);
    shield.setM2Speed(rightSpeed);
  }

  void setBrakes(int leftBrake, int rightBrake)
  {
    shield.setM1Brake(leftBrake);
    shield.setM2Brake(rightBrake);
  }

  void rotate(int speed)
  {
    // Positive speed = clockwise rotation, negative = counter-clockwise
    shield.setM1Speed(-speed);
    shield.setM2Speed(speed);
  }

  void stop()
  {
    shield.setM1Brake(0);
    shield.setM2Brake(0);
  }
};

/**
 * Class to manage an array of VL53L4CD distance sensors
 */
class DistanceSensorArray
{
private:
  VL53L4CD sensors[DISTANCE_SENSOR_COUNT];
  const uint8_t *xshutPins;
  I2CManager &i2c;
  int count;
  bool initialized = false;

public:
  DistanceSensorArray(const uint8_t *pins, int sensorCount, I2CManager &i2cManager)
      : xshutPins(pins), i2c(i2cManager), count(sensorCount) {}

  bool init()
  {
    Serial.println("Starting VL53L4CD distance sensor initialization...");

    // Power cycle all sensors by driving XSHUT pins low
    Serial.println("Power cycling all distance sensors...");
    for (int i = 0; i < count; i++)
    {
      pinMode(xshutPins[i], OUTPUT);
      digitalWrite(xshutPins[i], LOW);
    }
    delay(100); // Power-down time

    bool allSuccess = true;

    // Initialize each sensor one by one
    for (int i = 0; i < count; i++)
    {
      Serial.printf("Initializing distance sensor %d...\n", i);

      // Activate only this sensor with XSHUT pin
      pinMode(xshutPins[i], INPUT); // Allow carrier board to pull XSHUT high
      delay(50);                    // Boot delay

      // Slow down I2C for more reliable initialization
      i2c.setSlowSpeed();

      // Multiple initialization attempts
      bool success = false;
      for (int attempts = 0; attempts < 3 && !success; attempts++)
      {
        Serial.printf("Attempt %d for sensor %d\n", attempts + 1, i);
        success = sensors[i].init();

        if (!success)
        {
          Serial.printf("Failed attempt %d to initialize sensor %d\n", attempts + 1, i);
          delay(50);
          i2c.resetBus();
          delay(50);
        }
      }

      if (!success)
      {
        Serial.printf("Failed to initialize sensor %d after multiple attempts\n", i);
        allSuccess = false;
        continue;
      }

      Serial.printf("Successfully initialized sensor %d\n", i);

      // Assign unique address to this sensor
      byte newAddress = VL53L4CD_ADDRESS_BASE + i;
      sensors[i].setAddress(newAddress);
      Serial.printf("Set address 0x%02X for sensor %d\n", newAddress, i);

      // Verify communication with the new address
      if (i2c.testAddress(newAddress))
      {
        Serial.printf("Verified communication at address 0x%02X\n", newAddress);
      }

      // Configure for continuous measurement
      sensors[i].startContinuous();
      sensors[i].setTimeout(I2C_DEFAULT_TIMEOUT);

      // Test reading
      int16_t testReading = sensors[i].read();
      if (sensors[i].timeoutOccurred())
      {
        Serial.printf("Warning: Timeout for sensor %d\n", i);
      }
      else
      {
        Serial.printf("Sensor %d test reading: %d mm\n", i, testReading);
      }
    }

    // Restore normal I2C speed
    i2c.setNormalSpeed();
    Serial.println("Distance sensor initialization completed");

    initialized = allSuccess;
    return allSuccess;
  }

  bool readSensor(int index, int &distance)
  {
    if (index < 0 || index >= count)
      return false;

    distance = sensors[index].read();

    if (sensors[index].timeoutOccurred())
    {
      return false;
    }

    return (sensors[index].ranging_data.range_status == 0 &&
            distance >= PID_MIN_VALID_DISTANCE &&
            distance <= PID_MAX_VALID_DISTANCE);
  }

  bool getClosestObject(int &distance, int &sensorIndex)
  {
    int closestDistance = INT_MAX;
    int closestIndex = -1;
    bool foundValid = false;

    for (int i = 0; i < count; i++)
    {
      int reading;
      bool valid = readSensor(i, reading);

      if (valid && reading < closestDistance)
      {
        closestDistance = reading;
        closestIndex = i;
        foundValid = true;
      }
    }

    if (foundValid)
    {
      distance = closestDistance;
      sensorIndex = closestIndex;
    }

    return foundValid;
  }

  VL53L4CD &getSensor(int index)
  {
    return sensors[index];
  }

  bool isInitialized()
  {
    return initialized;
  }

  int getCount()
  {
    return count;
  }
};

/**
 * Class to manage QTR reflectance sensors
 */
class ReflectanceSensorArray
{
private:
  QTRSensors qtr;
  const uint8_t *pins;
  int count;
  uint16_t sensorValues[REFLECTANCE_SENSOR_COUNT];

public:
  ReflectanceSensorArray(const uint8_t *sensorPins, int sensorCount)
      : pins(sensorPins), count(sensorCount) {}

  void init()
  {
    qtr.setTypeRC();
    qtr.setSensorPins(pins, count);
    Serial.println("Reflectance sensors initialized");
  }

  void read()
  {
    qtr.read(sensorValues);
  }

  uint16_t getValue(int index)
  {
    if (index >= 0 && index < count)
    {
      return sensorValues[index];
    }
    return 0;
  }

  bool isDetectingLine(int index, int threshold = 1000)
  {
    return getValue(index) < threshold;
  }
};

/**
 * Base class for all test modes
 */
class TestMode
{
public:
  virtual ~TestMode() {}
  virtual void setup() = 0;
  virtual void loop() = 0;
  virtual const char *getName() = 0;
};

/**
 * Distance sensor testing mode
 */
class DistanceSensorTestMode : public TestMode
{
private:
  DistanceSensorArray &sensors;
  I2CManager &i2c;
  unsigned long lastResetTime = 0;

public:
  DistanceSensorTestMode(DistanceSensorArray &sensorArray, I2CManager &i2cManager)
      : sensors(sensorArray), i2c(i2cManager) {}

  void setup() override
  {
    sensors.init();
  }

  void loop() override
  {
    bool needsReset = false;

    // Get closest object information
    int closestDistance;
    int closestSensorIndex;
    bool targetFound = sensors.getClosestObject(closestDistance, closestSensorIndex);

    // Display closest object info and all sensor readings on the same line
    // Serial.print("Closest: ");
    // if (targetFound) {
    //   Serial.printf("S%d:%4dmm | ", closestSensorIndex, closestDistance);
    // } else {
    //   Serial.print("None      | ");
    // }

    // Read and display values from all distance sensors
    for (int i = 0; i < sensors.getCount(); i++)
    {
      int distance;
      bool valid = sensors.readSensor(i, distance);

      if (!valid && sensors.getSensor(i).timeoutOccurred())
      {
        Serial.print("TOUT ");
        needsReset = true;
      }
      else if (valid)
      {
        // Highlight the closest sensor reading
        if (targetFound && i == closestSensorIndex)
        {
          Serial.printf("[%3d]", distance);
        }
        else
        {
          Serial.printf("%5d", distance);
        }
      }
      else
      {
        Serial.print("INVL ");
      }

      Serial.print(" ");
    }
    Serial.println();

    // Reset I2C bus if needed (with cooldown period)
    if (needsReset && (millis() - lastResetTime > RESET_INTERVAL_MS))
    {
      Serial.println("Timeout detected, resetting I2C bus...");
      i2c.resetBus();
      lastResetTime = millis();
    }

    // delay(10); // Small delay to prevent serial flooding
  }

  const char *getName() override
  {
    return "DISTANCE SENSORS TEST";
  }
};

/**
 * Reflectance sensor testing mode
 */
class ReflectanceTestMode : public TestMode
{
private:
  ReflectanceSensorArray &sensors;

public:
  ReflectanceTestMode(ReflectanceSensorArray &sensorArray)
      : sensors(sensorArray) {}

  void setup() override
  {
    sensors.init();
  }

  void loop() override
  {
    sensors.read();

    for (int i = 0; i < REFLECTANCE_SENSOR_COUNT; i++)
    {
      bool isLine = sensors.isDetectingLine(i);
      Serial.printf("Sensor %d: %5s (%4d) | ",
                    i,
                    isLine ? "WHITE" : "BLACK",
                    sensors.getValue(i));
    }
    Serial.println();
  }

  const char *getName() override
  {
    return "REFLECTANCE SENSORS TEST";
  }
};

/**
 * Motor testing mode
 */
class MotorTestMode : public TestMode
{
private:
  MotorDriver &motors;

public:
  MotorTestMode(MotorDriver &motorDriver)
      : motors(motorDriver) {}

  void setup() override
  {
    motors.init();
    motors.setBrakes(400, 400);
  }

  void loop() override
  {
    int speed = (int)(cos(((float)millis()) * 0.001) * 400);
    Serial.println(speed);
    motors.setSpeeds(speed, speed);
    delay(2);
  }

  const char *getName() override
  {
    return "MOTOR CONTROL TEST";
  }
};

/**
 * I2C scanner mode
 */
class I2CScannerMode : public TestMode
{
private:
  I2CManager &i2c;

public:
  I2CScannerMode(I2CManager &i2cManager)
      : i2c(i2cManager) {}

  void setup() override
  {
    // Nothing specific to set up
  }

  void loop() override
  {
    i2c.scanBus();
    delay(SCAN_INTERVAL_MS);
  }

  const char *getName() override
  {
    return "I2C BUS SCANNER";
  }
};

/**
 * Tracking mode that turns toward the closest detected object with improved algorithms
 */
class TrackingMode : public TestMode
{
private:
  DistanceSensorArray &sensors;
  MotorDriver &motors;
  PIDController pid;
  I2CManager &i2c;
  unsigned long lastResetTime = 0;

  // Sensor physical configuration
  const float SENSOR_ANGLES[DISTANCE_SENSOR_COUNT] = {-36.0, -18.0, 0.0, 18.0, 36.0}; // Angles in degrees
  const float ROBOT_WIDTH = 200.0;                                                    // Robot width in mm (20cm)

  // Temporal averaging variables
  static const int HISTORY_SIZE = 3;
  float angleHistory[HISTORY_SIZE] = {0}; // Store recent target angles
  int historyIndex = 0;

  // Sensor readings cache
  struct SensorReading
  {
    int distance;
    bool valid;
    bool timeout;
  };
  SensorReading readings[DISTANCE_SENSOR_COUNT];

public:
  TrackingMode(DistanceSensorArray &sensorArray, MotorDriver &motorDriver,
               I2CManager &i2cManager)
      : sensors(sensorArray),
        motors(motorDriver),
        pid(PID_KP, PID_KI, PID_KD, PID_MAX_OUTPUT),
        i2c(i2cManager) {}

  void setup() override
  {
    sensors.init();
    motors.init();
    motors.stop();
    pid.reset();

    // Initialize history
    for (int i = 0; i < HISTORY_SIZE; i++)
    {
      angleHistory[i] = 0;
    }

    Serial.println("Improved tracking mode initialized - robot will track objects with angle-aware algorithms");
  }

  void loop() override
  {
    bool needsReset = false;
    
    // Read all sensors once at the beginning and cache results
    needsReset = readAllSensors();
    
    // Determine target angle using the closest sensor and simple averaging with neighbors when needed
    float targetAngle = calculateWeightedTargetAngle();
    
    // Apply temporal averaging to smooth out oscillations
    targetAngle = applyTemporalAveraging(targetAngle);
    
    // Display sensor readings
    displaySensorReadings();
    
    // Get the closest valid distance for forward movement control
    int closestDistance = getClosestDistance();
    
    // Apply PID control and drive motors
    if (closestDistance > 0) // Only check if a target is detected, angle can be 0 if enemy is straight ahead
    {
      // Target angle is in degrees. Convert to a PID error value.
      // Scale angle to appropriate error range for PID controller
      float error = targetAngle / 18.0; // 18 degrees = 1.0 error unit
      
      // Calculate rotation speed based on error
      int rotationSpeed = pid.calculate(error);
      
      // Calculate forward speed based on distance - using strategic approach
      int forwardSpeed = calculateStrategicSpeed(closestDistance, abs(error));
      
      // Apply forward movement and rotation
      int leftSpeed = forwardSpeed + rotationSpeed;
      int rightSpeed = forwardSpeed - rotationSpeed;
      
      // Apply speeds to motors directly
      motors.setSpeeds(leftSpeed, rightSpeed);
      
      Serial.printf("| Target Angle: %+4.1f° | Error: %+4.2f | Distance: %4d | Speed: %3d | Rotation: %+3d| LeftM:%+3d| RightM:%+3d\n", 
                  targetAngle, error, closestDistance, forwardSpeed, rotationSpeed, leftSpeed, rightSpeed);
    }
    else
    {
      // No valid targets found, stop
      motors.setBrakes(400, 400);
      Serial.println("| No valid target, stopping...");
      
      // Reset PID when no target is found
      pid.reset();
    }

    // Reset I2C bus if needed
    if (needsReset && (millis() - lastResetTime > RESET_INTERVAL_MS))
    {
      Serial.println("Timeout detected, resetting I2C bus...");
      i2c.resetBus();
      lastResetTime = millis();
    }
    
    // Small delay to prevent serial output flooding
    delay(10);
  }

  const char *getName() override
  {
    return "IMPROVED TRACKING WITH ANGLE AWARENESS";
  }

private:
  // Read all sensors once and store the readings
  bool readAllSensors()
  {
    bool needsReset = false;
    for (int i = 0; i < sensors.getCount(); i++)
    {
      readings[i].valid = sensors.readSensor(i, readings[i].distance);
      readings[i].timeout = !readings[i].valid && sensors.getSensor(i).timeoutOccurred();
      if(readings[i].timeout){
        needsReset = true;
      }
    }
    return needsReset;
  }

  // Calculate target angle using the closest sensor or average with its neighbor
  float calculateWeightedTargetAngle()
  {
    // Find the two sensors with lowest distance readings
    int lowestDistance = INT_MAX;
    int secondLowestDistance = INT_MAX;
    int lowestIndex = -1;
    int secondLowestIndex = -1;
    
    // Find the two closest sensors
    for (int i = 0; i < sensors.getCount(); i++)
    {
      if (readings[i].valid)
      {
        if (readings[i].distance < lowestDistance)
        {
          // This is now the closest, push previous closest to second place
          secondLowestDistance = lowestDistance;
          secondLowestIndex = lowestIndex;
          
          lowestDistance = readings[i].distance;
          lowestIndex = i;
        }
        else if (readings[i].distance < secondLowestDistance)
        {
          // This is the second closest
          secondLowestDistance = readings[i].distance;
          secondLowestIndex = i;
        }
      }
    }
    
    // If we don't have a valid reading, return 0
    if (lowestIndex == -1)
    {
      return 0;
    }
    
    // If we don't have a second valid reading, just use the closest sensor
    if (secondLowestIndex == -1)
    {
      return SENSOR_ANGLES[lowestIndex];
    }
    
    // Check if the two closest sensors are neighbors
    if (abs(lowestIndex - secondLowestIndex) == 1 && secondLowestDistance < lowestDistance*1.5)
    {
      // They are neighbors, return the average of their angles
      return (SENSOR_ANGLES[lowestIndex] + SENSOR_ANGLES[secondLowestIndex]) / 2.0;
    }
    
    // Otherwise just return the angle of the closest sensor
    return SENSOR_ANGLES[lowestIndex];
  }

  // Apply temporal averaging to smooth out oscillations
  float applyTemporalAveraging(float newAngle)
  {
    // Add new angle to history
    angleHistory[historyIndex] = newAngle;
    historyIndex = (historyIndex + 1) % HISTORY_SIZE;

    // Calculate average
    float sum = 0;
    for (int i = 0; i < HISTORY_SIZE; i++)
    {
      sum += angleHistory[i];
    }

    return sum / (float)HISTORY_SIZE;
  }

  // Display all sensor readings for debugging
  void displaySensorReadings()
  {
    for (int i = 0; i < sensors.getCount(); i++)
    {
      if (readings[i].timeout)
      {
        Serial.print("TOUT     | ");
      }
      else if (readings[i].valid)
      {
        Serial.printf("S%d: %4d | ", i, readings[i].distance);
      }
      else
      {
        Serial.printf("INVL     | ");
      }
    }
  }

  // Calculate strategic speed based on distance to target and alignment error
  int calculateStrategicSpeed(int distance, float alignmentError)
  {
    // Define distance thresholds
    const int STOP_DISTANCE = 10;       // mm - minimum safe distance, stop when closer than this
    const int IMPACT_DISTANCE = 300;    // mm - distance to start accelerating for impact
    const int ALIGNMENT_DISTANCE = 500; // mm - distance to start precision alignment
    const int MAX_DISTANCE = 800;       // mm - maximum distance to consider for speed scaling
    
    // Define speed settings
    const int MAX_SPEED = 400;          // maximum motor speed for far distances
    const int ALIGNMENT_SPEED = 200;    // reduced speed during alignment phase
    const int IMPACT_SPEED = 350;       // high speed for final impact
    
    // If too close, don't move forward
    if (distance < STOP_DISTANCE)
    {
      return 0;
    }
    
    // If within impact range and well-aligned, use impact speed for maximum push force
    if (distance < IMPACT_DISTANCE && alignmentError < 0.8)
    {
      return IMPACT_SPEED;
    }
    
    // If in alignment range, use reduced speed for better precision
    if (distance < ALIGNMENT_DISTANCE)
    {
      // Reduce speed more when alignment error is high
      return ALIGNMENT_SPEED * (1.0 - (alignmentError * 0.3)); // max alignmentError = 2
    }
    
    // For far distances, use full speed to close distance quickly
    if (distance >= MAX_DISTANCE)
    {
      return MAX_SPEED;
    }
    
    // Between alignment distance and max distance, scale from max speed to alignment speed
    float speedRatio = (float)(distance - ALIGNMENT_DISTANCE) / (MAX_DISTANCE - ALIGNMENT_DISTANCE);
    return ALIGNMENT_SPEED + (int)((MAX_SPEED - ALIGNMENT_SPEED) * speedRatio);
  }
  
  // Get the closest valid distance reading
  int getClosestDistance()
  {
    int minDistance = INT_MAX;
    
    for (int i = 0; i < sensors.getCount(); i++)
    {
      if (readings[i].valid && readings[i].distance < minDistance)
      {
        minDistance = readings[i].distance;
      }
    }
    
    return (minDistance == INT_MAX) ? -1 : minDistance;
  }
};

/**
 * Main SumoBot class that coordinates all components and modes
 */
class SumoBot
{
private:
  // Test mode selection
  enum TestModeType
  {
    MODE_DISTANCE,
    MODE_REFLECTANCE,
    MODE_MOTOR,
    MODE_I2C_SCANNER,
    MODE_TRACKING
  };

  // Components
  I2CManager i2c;
  DistanceSensorArray distanceSensors;
  ReflectanceSensorArray reflectanceSensors;
  MotorDriver motors;

  // Test modes
  DistanceSensorTestMode distanceMode;
  ReflectanceTestMode reflectanceMode;
  MotorTestMode motorMode;
  I2CScannerMode scannerMode;
  TrackingMode trackingMode;

  TestMode *currentMode;
  TestModeType selectedMode;

public:
  SumoBot()
      : i2c(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_SPEED_NORMAL, I2C_CLOCK_SPEED_INIT, USE_FAST_I2C_RESET),
        distanceSensors(DISTANCE_SENSOR_XSHUT_PINS, DISTANCE_SENSOR_COUNT, i2c),
        reflectanceSensors(REFLECTANCE_SENSOR_PINS, REFLECTANCE_SENSOR_COUNT),
        motors(27, 14, 12, 255, 254, 33, 25, 26, 253, 252),
        distanceMode(distanceSensors, i2c),
        reflectanceMode(reflectanceSensors),
        motorMode(motors),
        scannerMode(i2c),
        trackingMode(distanceSensors, motors, i2c)
  {
    // Set default test mode
    selectedMode = MODE_TRACKING;
    setMode(selectedMode);
  }

  void begin()
  {
    Serial.begin(115200);
    delay(500);

    i2c.begin();

    Serial.println("===== SUMO BOT SYSTEM INITIALIZED =====");
    Serial.println("--------------------------------------------------");

    if (currentMode)
    {
      Serial.printf("MODE: %s\n", currentMode->getName());
      currentMode->setup();
    }

    Serial.println("Setup complete - entering main loop");
    Serial.println("--------------------------------------------------");
  }

  void update()
  {
    if (currentMode)
    {
      currentMode->loop();
    }
  }

  void setMode(TestModeType mode)
  {
    selectedMode = mode;

    switch (mode)
    {
    case MODE_DISTANCE:
      currentMode = &distanceMode;
      break;

    case MODE_REFLECTANCE:
      currentMode = &reflectanceMode;
      break;

    case MODE_MOTOR:
      currentMode = &motorMode;
      break;

    case MODE_I2C_SCANNER:
      currentMode = &scannerMode;
      break;

    case MODE_TRACKING:
      currentMode = &trackingMode;
      break;

    default:
      currentMode = nullptr;
      break;
    }

    if (currentMode)
    {
      Serial.printf("Switching to mode: %s\n", currentMode->getName());
      currentMode->setup();
    }
  }
};

//==============================================================================
// GLOBAL INSTANCES
//==============================================================================

SumoBot sumoBot;

//==============================================================================
// ARDUINO FUNCTIONS
//==============================================================================

void setup()
{
  sumoBot.begin();
}

void loop()
{
  sumoBot.update();
}
