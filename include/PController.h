class PController {
    public:
        float P;
        float setpoint;
        float currentMeasurement;
        PController(float P);
        void setMeasurement(float currentMeasurement);
        void setSetpoint(float setpoint);
        float calculate();
};