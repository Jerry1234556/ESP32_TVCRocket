# ESP32_TVCRocket
 first LRA small tvc rocket under AR March/22/2025; Seymour prototype

ESP32 TVC Rocket System
This project implements the embedded software running on the ESP32 board in a two-board architecture for a thrust vector controlled (TVC) model rocket. The code handles real-time sensor fusion, data logging, and control signal generation for servo-based actuation.

🚀 System Overview
The system is split across two microcontrollers:

EasyMega Board:
Handles standard model rocketry functions, including:

Apogee detection

Parachute deployment

Power delivery to high-demand components like servos

ESP32 DevKit V1 (This repo):
Responsible for the rest of the flight control stack:

Sensor Fusion

Reads orientation and acceleration data from an MPU6050 via I2C

Performs Madgwick filtering (quaternion-based)

Outputs rocket-centric roll/pitch/yaw and acceleration values

TVC Controller

Computes servo outputs based on orientation error

Sends PWM signals to SG90 servos using a custom ESP32Servo library

Data Logging

Logs flight data to SPIFFS (/log.txt) for post-flight analysis

🔧 Components Used
Component	Role
ESP32 DevKit V1	Main flight computer (controls TVC and logs data)
EasyMega	Handles power distribution and recovery system
MPU6050	6-DOF IMU for orientation and acceleration sensing
SG90 Micro Servos	Control vectoring of rocket nozzle (TVC)
🧠 Sensor Fusion: Lessons Learned
While the MPU6050 served as a decent starting point for early flights and algorithm development, its limitations became evident:

Weaknesses

Fusion output is sensitive to dynamic accelerations (e.g. launch and wind buffeting)

The accelerometer introduces drift during high-thrust events despite gating logic

Recommended Upgrade

Switch to BMI088 IMU for better high-dynamic-range and vibration-resistant performance

📈 Future Improvements
Replace MPU6050 with BMI088

Improve sensor stability under high acceleration and vibration

Implement Kalman or complementary filter as alternative to Madgwick

More robustness under linear acceleration

Optimize control algorithm

Right now it's likely a simple proportional mapping; PID or nonlinear control would enhance performance

Integrate launch detection or flight mode state machine

📁 File Breakdown
RocketIMUFusion_Jerry_032125.* – MPU6050 sensor fusion logic (Madgwick filter with gating)

ESP32_SPIFFS_DataLogger.* – SPIFFS-based flight data logger

ESP32Servo.* – Modified servo library tailored for ESP32 PWM channels
