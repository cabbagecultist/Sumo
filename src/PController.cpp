#include "PController.h"

PController::PController(float P) {
    this->P = P;
}

void PController::setMeasurement(float currentMeasurement) {
    this->currentMeasurement = currentMeasurement;
}

void PController::setSetpoint(float setpoint) {
    this->setpoint = setpoint;
}

float PController::calculate() {
    float error = setpoint - currentMeasurement;
    return P * error;
}