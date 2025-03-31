#include "RocketIMUFusion_Jerry_032125.h"
#include <math.h>

// A small helper for fast inverse sqrt
static float invSqrt(float x) {
    return 1.0f / sqrtf(x);
}

// ---------------------
// Constructor
// ---------------------
RocketIMUFusion_Jerry_032125::RocketIMUFusion_Jerry_032125()
: gyroOffsetX(0), gyroOffsetY(0), gyroOffsetZ(0)
, accelOffsetX(0), accelOffsetY(0), accelOffsetZ(0)
, q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f)
, lastUpdateMs(0)
, updatePeriodMs(5)  // default ~200 Hz
, beta(0.03f)
, sampleFreq(200.0f)
, accelGateThreshold(2.0f) // ±2 m/s^2 from 9.81
{
}

// ---------------------
// Public: begin
// ---------------------
bool RocketIMUFusion_Jerry_032125::begin() {
    if(!mpu.begin()) {
        return false;
    }
    // For a small rocket, ±16g might be good
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

    // ±500 deg/s
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);

    // Possibly set the filter bandwidth
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

    // Let sensor settle
    delay(100);
    // We'll do a short discard
    unsigned long start = millis();
    while(millis() - start < 300) {
        sensors_event_t a,g,t;
        mpu.getEvent(&a,&g,&t);
        delay(5);
    }
    // Reset quaternion
    q0 = 1.f; q1=q2=q3=0.f;
    lastUpdateMs = millis();
    return true;
}

// ---------------------
// Public: calibrate
// (longer routine)
// ---------------------
void RocketIMUFusion_Jerry_032125::calibrate() {
    // e.g. discard 600 ms, then average 200 samples
    doCalibrationRoutine(600, 200);
}

// ---------------------
// Public: miniCalibrate
// (short routine)
// ---------------------
void RocketIMUFusion_Jerry_032125::miniCalibrate() {
    // e.g. discard 300 ms, then average 50 samples
    doCalibrationRoutine(300, 50);
}

// ---------------------
// Private: doCalibrationRoutine
// ---------------------
void RocketIMUFusion_Jerry_032125::doCalibrationRoutine(unsigned long discardMs, int samples) {
    // Discard
    unsigned long start = millis();
    while(millis() - start < discardMs) {
        sensors_event_t a,g,t;
        mpu.getEvent(&a,&g,&t);
        delay(5);
    }

    // Now accumulate
    float sumGx=0, sumGy=0, sumGz=0;
    float sumAx=0, sumAy=0, sumAz=0;

    for(int i=0; i<samples; i++){
        sensors_event_t a,g,t;
        mpu.getEvent(&a,&g,&t);

        float gxDeg = g.gyro.x * rad2deg;
        float gyDeg = g.gyro.y * rad2deg;
        float gzDeg = g.gyro.z * rad2deg;

        sumGx += gxDeg;
        sumGy += gyDeg;
        sumGz += gzDeg;

        sumAx += a.acceleration.x;
        sumAy += a.acceleration.y;
        sumAz += a.acceleration.z;

        delay(5);
    }

    gyroOffsetX = sumGx / samples;
    gyroOffsetY = sumGy / samples;
    gyroOffsetZ = sumGz / samples;

    accelOffsetX = sumAx / samples;
    accelOffsetY = sumAy / samples;
    accelOffsetZ = sumAz / samples;

    // Reset the Madgwick quaternion after calibration
    q0=1.f; q1=q2=q3=0.f;
    lastUpdateMs = millis();
}

// ---------------------
// Public: update
// (reads sensor, runs Madgwick, gating if needed)
// ---------------------
void RocketIMUFusion_Jerry_032125::update() {
    unsigned long now = millis();
    // check time
    if((now - lastUpdateMs) < updatePeriodMs) {
        // not time yet
        return;
    }
    float dt = (now - lastUpdateMs) * 0.001f; 
    lastUpdateMs = now;

    // read sensor
    sensors_event_t a,g,temp;
    mpu.getEvent(&a,&g,&temp);

    // convert gyro to deg/s, subtract offsets
    float gxDeg = (g.gyro.x * rad2deg) - gyroOffsetX;
    float gyDeg = (g.gyro.y * rad2deg) - gyroOffsetY;
    float gzDeg = (g.gyro.z * rad2deg) - gyroOffsetZ;

    // convert accel to m/s^2, subtract offsets
    float ax = a.acceleration.x - accelOffsetX;
    float ay = a.acceleration.y - accelOffsetY;
    float az = a.acceleration.z - accelOffsetZ;

    // decide if we trust accel
    bool trustAccel = doAccelUpdate(ax, ay, az);

    float usedAx=0, usedAy=0, usedAz=0;
    if(trustAccel) {
        usedAx=ax; usedAy=ay; usedAz=az;
    }

    // sampleFreq = 1/dt
    sampleFreq = 1.0f / dt;
    // run Madgwick
    madgwickUpdate(usedAx, usedAy, usedAz, gxDeg, gyDeg, gzDeg);
}

// ---------------------
// Public: getYPRAccel
// (returns rollZ, pitchX, yawY, plus raw accel in m/s^2)
// ---------------------
void RocketIMUFusion_Jerry_032125::getYPRAccel(float &rollZ, float &pitchX, float &yawY,
                                               float &ax, float &ay, float &az)
{
    // convert quaternion => rocket-friendly angles
    getZXYAngles(rollZ, pitchX, yawY);

    // If we want the current raw accel, we can re-read or store from last update
    // For simplicity, let's just do a fresh read:
    sensors_event_t a,g,temp;
    mpu.getEvent(&a,&g,&temp);

    ax = a.acceleration.x - accelOffsetX;
    ay = a.acceleration.y - accelOffsetY;
    az = a.acceleration.z - accelOffsetZ;
}

// ---------------------
// Public: setAccelGateThreshold
// ---------------------
void RocketIMUFusion_Jerry_032125::setAccelGateThreshold(float threshold) {
    accelGateThreshold = threshold;
}

// ---------------------
// Public: setMadgwickBeta
// ---------------------
void RocketIMUFusion_Jerry_032125::setMadgwickBeta(float newBeta) {
    beta = newBeta;
}

// ---------------------
// Public: setUpdatePeriod
// ---------------------
void RocketIMUFusion_Jerry_032125::setUpdatePeriod(unsigned long ms) {
    updatePeriodMs = ms;
}

// ---------------------
// Private: doAccelUpdate
// (skip if net accel is far from 1g)
// ---------------------
bool RocketIMUFusion_Jerry_032125::doAccelUpdate(float ax, float ay, float az) {
    float netA = sqrtf(ax*ax + ay*ay + az*az);
    float diff = fabsf(netA - 9.81f);
    return (diff < accelGateThreshold);
}

// ---------------------
// Private: madgwickUpdate
// (the same as in the user code, but a method now)
// ---------------------
void RocketIMUFusion_Jerry_032125::madgwickUpdate(float ax, float ay, float az, 
                                                  float gxDeg, float gyDeg, float gzDeg)
{
  // Convert deg/s => rad/s
  const float deg2rad = 0.01745329252f;
  float gx = gxDeg * deg2rad;
  float gy = gyDeg * deg2rad;
  float gz = gzDeg * deg2rad;

  // Normalise accelerometer
  float norm = ax*ax + ay*ay + az*az;
  if (norm > 0.0f) {
    norm = invSqrt(norm);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // gradient descent
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;

    float f1 = _2q2*q3 - _2q0*q1 - ax;
    float f2 = _2q0*q0 + _2q1*q3 - ay;
    float f3 = 1.0f - (_2q1*q1 + _2q2*q2) - az;

    float J11 = -_2q1;
    float J12 =  _2q0;
    float J21 =  _2q2;
    float J22 =  _2q3;
    float J31 = -_2q0;
    float J32 = -_2q1;

    float grad0 = (J11*f1 + J21*f2 + J31*f3);
    float grad1 = (J12*f1 + J22*f2 + J32*f3);
    float grad2 = 0.0f;
    float grad3 = 0.0f;

    float gnorm = grad0*grad0 + grad1*grad1 + grad2*grad2 + grad3*grad3;
    if(gnorm > 0.f){
      gnorm = invSqrt(gnorm);
      grad0 *= gnorm;
      grad1 *= gnorm;
      grad2 *= gnorm;
      grad3 *= gnorm;
    }

    float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    // subtract beta * gradient
    qDot0 -= beta * grad0;
    qDot1 -= beta * grad1;
    qDot2 -= beta * grad2;
    qDot3 -= beta * grad3;

    float dt = 1.0f / sampleFreq;
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
  }
  else {
    // no accel => gyro only
    float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    float dt = 1.0f / sampleFreq;
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
  }

  // normalize
  float qNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= qNorm;
  q1 *= qNorm;
  q2 *= qNorm;
  q3 *= qNorm;
}

// ---------------------
// Private: getZXYAngles
// (rollZ about Z, pitchX about X, yawY about Y)
// ---------------------
void RocketIMUFusion_Jerry_032125::getZXYAngles(float &rollDeg, float &pitchDeg, float &yawDeg)
{
  float w = q0;
  float x = q1;
  float y = q2;
  float z = q3;

  // rollZ
  float rZ = atan2f(2.0f*(w*z + x*y), 
                    1.0f - 2.0f*(y*y + z*z));
  // pitchX
  float pX = atan2f(2.0f*(w*x + y*z), 
                    1.0f - 2.0f*(x*x + y*y));
  // yawY
  float yY = asinf(2.0f*(w*y - x*z));

  float conv = 180.0f / M_PI;
  rZ *= conv; 
  pX *= conv;
  yY *= conv;

  if(rZ > 180.f)  rZ -= 360.f; else if(rZ < -180.f) rZ += 360.f;
  if(pX > 180.f)  pX -= 360.f; else if(pX < -180.f) pX += 360.f;
  if(yY > 180.f)  yY -= 360.f; else if(yY < -180.f) yY += 360.f;

  rollDeg  = rZ;
  pitchDeg = pX;
  yawDeg   = yY;
}
