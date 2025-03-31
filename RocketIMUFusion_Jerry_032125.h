#ifndef ROCKETIMUFUSION_JERRY_032125_H
#define ROCKETIMUFUSION_JERRY_032125_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

class RocketIMUFusion_Jerry_032125 {
public:
    // Constructor
    RocketIMUFusion_Jerry_032125();

    // 1) Initialize the MPU (setup I2C, etc.)
    bool begin();

    // 2) Full calibration (longer duration)
    void calibrate();

    // 3) Mini calibration (shorter duration, less accurate)
    void miniCalibrate();

    // 4) Main update function: read sensor, run fusion
    void update();

    // 5) Retrieve latest orientation angles (rollZ, pitchX, yawY) in degrees
    //    and the raw accel (m/s^2). 
    //    Note: We interpret Z as the rocket axis => "roll" about Z.
    void getYPRAccel(float &rollZ, float &pitchX, float &yawY, 
                     float &ax, float &ay, float &az);

    // (Optional) Change the gating threshold if you want
    void setAccelGateThreshold(float threshold);

    // (Optional) Adjust the Madgwick beta gain
    void setMadgwickBeta(float newBeta);

    // (Optional) Set the update period or sample freq
    // default is 200 Hz (5 ms)
    void setUpdatePeriod(unsigned long ms);

private:
    // Internal helper: Madgwick
    void madgwickUpdate(float ax, float ay, float az, 
                        float gxDeg, float gyDeg, float gzDeg);

    // Internal helper: convert quaternion => rollZ/pitchX/yawY
    void getZXYAngles(float &rollDeg, float &pitchDeg, float &yawDeg);

    // We skip accel correction if net accel deviates from 9.81 too much
    bool doAccelUpdate(float ax, float ay, float az);

    // We do a short or long calibration
    void doCalibrationRoutine(unsigned long discardMs, int samples);

private:
    // Adafruit sensor object
    Adafruit_MPU6050 mpu;

    // Gyro offsets in deg/s, accel offsets in m/s^2
    float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    float accelOffsetX, accelOffsetY, accelOffsetZ;

    // Quaternions for Madgwick
    float q0, q1, q2, q3;

    // Time-based update
    unsigned long lastUpdateMs;
    unsigned long updatePeriodMs;

    // Madgwick parameters
    float beta;        // ~0.01..0.1
    float sampleFreq;  // dynamic: 1/(dt)
    // If library returns gyro in rad/s => we multiply by rad2deg
    static constexpr float rad2deg = 57.29578f;

    // Accel gating
    float accelGateThreshold; // ± this from 9.81 => ignore

};

#endif
