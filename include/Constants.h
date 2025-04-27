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
