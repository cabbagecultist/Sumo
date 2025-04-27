#include <Arduino.h>
#include <Wire.h>
#include <Constants.h>

class I2CHelper {
    public:
        static void resetI2CBus();
        static void resetI2CBusFast();
        static void printI2CStatus(byte status);
        static bool testI2CAddress(byte address);
};