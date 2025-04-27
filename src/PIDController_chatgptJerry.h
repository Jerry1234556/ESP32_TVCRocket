#ifndef PIDController_chatgptJerry_H
#define PIDController_chatgptJerry_H

struct PIDState {
    float integral = 0.0f;
    float lastError = 0.0f;
};

struct PIDConfig {
    float kp, ki, kd;
    float minOutput;
    float maxOutput;
};

// General 1-axis PID controller
int computePID(float currentAngle, float targetAngle, float dtMs,
               const PIDConfig &config, PIDState &state);

#endif
