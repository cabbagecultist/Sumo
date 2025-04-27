class PController {
    public:
        float P;
        float setpoint;
        PController(float P);
        void setSetpoint(float setpoint);
        float calculate(float measurement);
};