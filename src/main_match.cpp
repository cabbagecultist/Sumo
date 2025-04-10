#include <Arduino.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <VL53L4CD.h>
#include "DualVNH5019MotorShield.h"

//==============================================================================
// CONFIGURATION AND PIN DEFINITIONS
//==============================================================================

// I2C Configuration
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_CLOCK_SPEED 400000  // Use 400kHz for faster sensor readings in match

// VL53L4CD Distance Sensor Configuration
#define VL53L4CD_DEFAULT_ADDRESS 0x52
#define VL53L4CD_ADDRESS_BASE 0x2A
#define DISTANCE_SENSOR_COUNT 5
#define SENSOR_TIMEOUT_MS 30    // Shorter timeout for faster response
#define MAX_DETECT_DISTANCE 1200 // Maximum distance to consider for opponent detection (mm)

// Reflectance Sensor Configuration
#define REFLECTANCE_SENSOR_COUNT 4
#define REFLECTANCE_EMITTER_PIN 13
#define EDGE_THRESHOLD 500      // Threshold for detecting ring edge (adjust based on testing)

// Reflectance sensor positions
#define FRONT_LEFT_OUTER 0     // Outer left sensor
#define FRONT_LEFT_INNER 1     // Inner left sensor
#define FRONT_RIGHT_INNER 2    // Inner right sensor
#define FRONT_RIGHT_OUTER 3    // Outer right sensor

// Edge Recovery Configuration
#define EDGE_BACKUP_TIME_MS 150    // How long to back up when edge detected
#define EDGE_TURN_TIME_MS 100      // How long to turn when edge detected
#define EDGE_BACKUP_SPEED 300      // Speed to back up at
#define EDGE_TURN_SPEED 250        // Speed to turn at

// Motor Configuration
#define MAX_MOTOR_SPEED 400     // Maximum motor speed
#define TURN_SPEED 300         // Speed for turning
#define SEARCH_SPEED 200       // Speed while searching
#define ATTACK_SPEED 400       // Speed while attacking
#define ESCAPE_SPEED 400       // Speed while escaping from edge

// Strategy Timing
#define SENSOR_READ_INTERVAL_MS 10  // How often to read sensors
#define STRATEGY_UPDATE_INTERVAL_MS 20  // How often to update strategy
#define STARTUP_DELAY_MS 5000    // Delay before starting match

//==============================================================================
// PIN ARRAYS AND GLOBAL VARIABLES
//==============================================================================

// Distance sensor XSHUT pins
const uint8_t DISTANCE_SENSOR_XSHUT_PINS[DISTANCE_SENSOR_COUNT] = {19, 16, 18, 5, 17};
VL53L4CD distanceSensors[DISTANCE_SENSOR_COUNT];

// Reflectance sensor pins
const uint8_t REFLECTANCE_SENSOR_PINS[REFLECTANCE_SENSOR_COUNT] = {4, 14, 27, 15};
uint16_t reflectanceSensorValues[REFLECTANCE_SENSOR_COUNT];

// Motor controller
DualVNH5019MotorShield motors(26, 25, 33, 255, 254, 34, 35, 32, 253, 252);

// QTR reflectance sensor object
QTRSensors qtr;

// State variables
uint32_t lastSensorRead = 0;
uint32_t lastStrategyUpdate = 0;
bool edgeDetected = false;
int opponentDirection = -1;  // -1: not found, 0-4: sensor index that detected opponent

//==============================================================================
// SENSOR DATA STRUCTURES
//==============================================================================

struct SensorData {
    int16_t distances[DISTANCE_SENSOR_COUNT];
    bool timeouts[DISTANCE_SENSOR_COUNT];
    uint16_t reflectance[REFLECTANCE_SENSOR_COUNT];
    bool isValid;
};

//==============================================================================
// FUNCTION DECLARATIONS
//==============================================================================

void setupSensors();
void setupMotors();
SensorData readSensors();
void updateStrategy(const SensorData& sensors);
void executeMove(int leftSpeed, int rightSpeed);
void emergencyStop();
void handleEdgeDetection(const SensorData& sensors);
void handleOpponentDetection(const SensorData& sensors);
void searchPattern();

//==============================================================================
// SETUP AND MAIN LOOP
//==============================================================================

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    
    // Initialize I2C with fast mode
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_SPEED);
    
    // Setup sensors and motors
    setupSensors();
    setupMotors();
    
    // Wait for start signal
    Serial.println("Waiting for match start...");
    delay(STARTUP_DELAY_MS);
    Serial.println("Match started!");
}

void loop() {
    // Read sensors at regular intervals
    if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL_MS) {
        SensorData sensorData = readSensors();
        lastSensorRead = millis();
        
        // Update strategy based on sensor readings
        if (sensorData.isValid && millis() - lastStrategyUpdate >= STRATEGY_UPDATE_INTERVAL_MS) {
            updateStrategy(sensorData);
            lastStrategyUpdate = millis();
        }
    }
}

//==============================================================================
// INITIALIZATION FUNCTIONS
//==============================================================================

void setupSensors() {
    // Initialize reflectance sensors
    qtr.setTypeRC();
    qtr.setSensorPins(REFLECTANCE_SENSOR_PINS, REFLECTANCE_SENSOR_COUNT);
    qtr.setEmitterPin(REFLECTANCE_EMITTER_PIN);
    
    // Initialize distance sensors
    for (uint8_t i = 0; i < DISTANCE_SENSOR_COUNT; i++) {
        // Power cycle sensor
        pinMode(DISTANCE_SENSOR_XSHUT_PINS[i], OUTPUT);
        digitalWrite(DISTANCE_SENSOR_XSHUT_PINS[i], LOW);
    }
    delay(10);
    
    // Initialize each sensor sequentially
    for (uint8_t i = 0; i < DISTANCE_SENSOR_COUNT; i++) {
        pinMode(DISTANCE_SENSOR_XSHUT_PINS[i], INPUT); // Let pull-up activate sensor
        delay(10);
        
        if (!distanceSensors[i].init()) {
            Serial.printf("Failed to initialize sensor %d\n", i);
            continue;
        }
        
        // Set unique address
        byte newAddress = VL53L4CD_ADDRESS_BASE + i;
        distanceSensors[i].setAddress(newAddress);
        
        // Configure for fast ranging
        distanceSensors[i].startContinuous();
        distanceSensors[i].setTimeout(SENSOR_TIMEOUT_MS);
    }
}

void setupMotors() {
    motors.init();
    motors.setM1Brake(0);
    motors.setM2Brake(0);
}

//==============================================================================
// SENSOR READING FUNCTIONS
//==============================================================================

SensorData readSensors() {
    SensorData data;
    data.isValid = true;
    
    // Read distance sensors
    for (uint8_t i = 0; i < DISTANCE_SENSOR_COUNT; i++) {
        data.distances[i] = distanceSensors[i].read();
        data.timeouts[i] = distanceSensors[i].timeoutOccurred();
        
        if (data.timeouts[i]) {
            data.distances[i] = MAX_DETECT_DISTANCE + 1; // Set to beyond max range if timeout
        }
    }
    
    // Read reflectance sensors
    qtr.read(data.reflectance);
    
    return data;
}

//==============================================================================
// STRATEGY FUNCTIONS
//==============================================================================

void updateStrategy(const SensorData& sensors) {
    // First priority: Check for ring edge
    handleEdgeDetection(sensors);
    
    // If we're at the edge, don't process opponent detection
    if (edgeDetected) {
        return;
    }
    
    // Second priority: Look for opponent
    handleOpponentDetection(sensors);
    
    // If no opponent found, execute search pattern
    if (opponentDirection == -1) {
        searchPattern();
    }
}

void handleEdgeDetection(const SensorData& sensors) {
    edgeDetected = false;
    bool leftOuterEdge = sensors.reflectance[FRONT_LEFT_OUTER] > EDGE_THRESHOLD;
    bool leftInnerEdge = sensors.reflectance[FRONT_LEFT_INNER] > EDGE_THRESHOLD;
    bool rightInnerEdge = sensors.reflectance[FRONT_RIGHT_INNER] > EDGE_THRESHOLD;
    bool rightOuterEdge = sensors.reflectance[FRONT_RIGHT_OUTER] > EDGE_THRESHOLD;
    
    // If any sensor detects edge, we need to take action
    if (leftOuterEdge || leftInnerEdge || rightInnerEdge || rightOuterEdge) {
        edgeDetected = true;
        
        // Determine escape direction based on which sensors detected the edge
        if (leftOuterEdge && rightOuterEdge) {
            // Both outer sensors detected edge - back straight up
            executeMove(-EDGE_BACKUP_SPEED, -EDGE_BACKUP_SPEED);
            delay(EDGE_BACKUP_TIME_MS);
            // Turn around
            executeMove(EDGE_TURN_SPEED, -EDGE_TURN_SPEED);
            delay(EDGE_TURN_TIME_MS * 2);
        }
        else if (leftOuterEdge || leftInnerEdge) {
            // Left side detected edge
            // First back up slightly
            executeMove(-EDGE_BACKUP_SPEED, -EDGE_BACKUP_SPEED * 0.7);
            delay(EDGE_BACKUP_TIME_MS);
            // Then turn right
            executeMove(EDGE_TURN_SPEED, -EDGE_TURN_SPEED * 0.5);
            delay(EDGE_TURN_TIME_MS);
        }
        else if (rightOuterEdge || rightInnerEdge) {
            // Right side detected edge
            // First back up slightly
            executeMove(-EDGE_BACKUP_SPEED * 0.7, -EDGE_BACKUP_SPEED);
            delay(EDGE_BACKUP_TIME_MS);
            // Then turn left
            executeMove(-EDGE_TURN_SPEED * 0.5, EDGE_TURN_SPEED);
            delay(EDGE_TURN_TIME_MS);
        }
        
        // Stop after escape maneuver
        executeMove(0, 0);
        return;
    }
}

void handleOpponentDetection(const SensorData& sensors) {
    opponentDirection = -1;
    int minDistance = MAX_DETECT_DISTANCE;
    
    // Find closest detection
    for (uint8_t i = 0; i < DISTANCE_SENSOR_COUNT; i++) {
        if (!sensors.timeouts[i] && sensors.distances[i] < minDistance) {
            minDistance = sensors.distances[i];
            opponentDirection = i;
        }
    }
    
    // Execute attack strategy if opponent found
    if (opponentDirection != -1) {
        // Calculate motor speeds based on which sensor detected the opponent
        switch (opponentDirection) {
            case 0: // Front sensor - full attack
                executeMove(ATTACK_SPEED, ATTACK_SPEED);
                break;
            case 1: // Front-right sensor
                executeMove(ATTACK_SPEED, ATTACK_SPEED * 0.6);
                break;
            case 2: // Right sensor
                executeMove(TURN_SPEED, -TURN_SPEED);
                break;
            case 3: // Back-right sensor
                executeMove(-TURN_SPEED, TURN_SPEED);
                break;
            case 4: // Back sensor
                executeMove(-TURN_SPEED, -TURN_SPEED);
                break;
        }
    }
}

void searchPattern() {
    // Implement a spiral search pattern
    static uint32_t searchStart = millis();
    static bool searchDirection = true;
    
    // Switch direction every 2 seconds
    // if (millis() - searchStart > 2000) {
    //     searchDirection = !searchDirection;
    //     searchStart = millis();
    // }
    
    if (searchDirection) {
        executeMove(SEARCH_SPEED, SEARCH_SPEED * 0.7);
    } else {
        executeMove(SEARCH_SPEED * 0.7, SEARCH_SPEED);
    }
}

//==============================================================================
// MOTOR CONTROL FUNCTIONS
//==============================================================================

void executeMove(int leftSpeed, int rightSpeed) {
    // Constrain speeds to valid range
    leftSpeed = constrain(leftSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    
    // Set motor speeds
    motors.setM1Speed(leftSpeed);
    motors.setM2Speed(rightSpeed);
}

void emergencyStop() {
    motors.setM1Brake(400);
    motors.setM2Brake(400);
} 