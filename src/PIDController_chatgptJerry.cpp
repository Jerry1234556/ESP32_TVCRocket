#include "PIDController_chatgptJerry.h"

int computePID(float currentAngle, float targetAngle, float dtMs,
               const PIDConfig &config, PIDState &state) {
    float dtSec = dtMs / 1000.0f;
    float error = targetAngle - currentAngle;

    // Integrate and differentiate
    state.integral += error * dtSec;
    float derivative = (error - state.lastError) / dtSec;
    state.lastError = error;

    // Compute raw output
    float output = config.kp * error + config.ki * state.integral + config.kd * derivative;

    // Clamp output
    if (output > config.maxOutput) output = config.maxOutput;
    else if (output < config.minOutput) output = config.minOutput;

    return static_cast<int>(output);
}
