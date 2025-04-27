#include "VL53L4CD.h"
#include "Constants.h"
#include "I2CHelper.h"

class DistanceSensors {
    public:
        enum class SENSOR {
            LEFTMOST = 0,
            LEFT = 1,
            MIDDLE = 2,
            RIGHT = 3,
            RIGHTMOST = 4
        };

        void begin();
        float getSensorDistance(DistanceSensors::SENSOR sensor);
        float getDistanceAtAngle(float angle);
        float getLowestDistanceAngle();

    private:
        // Sensor Definitions
        VL53L4CD distanceSensors[DISTANCE_SENSOR_COUNT];
        const uint8_t DISTANCE_SENSOR_XSHUT_PINS[DISTANCE_SENSOR_COUNT] = {19, 16, 18, 5, 17};

        void interpolate();
};