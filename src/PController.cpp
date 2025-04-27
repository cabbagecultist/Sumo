#include "PController.h"

PController::PController(float P) {
    this->P = P;
}

void PController::setSetpoint(float setpoint) {
    this->setpoint = setpoint;
}

float PController::calculate(float measurement) {
    float error = setpoint - measurement;
    return P * error;
}