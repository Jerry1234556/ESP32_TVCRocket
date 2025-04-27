/*
   MAIN TVC CODE EXAMPLE (UPDATED WITH HEADER)
   README: tvc has placeholder

   Key changes:
   - After we detect launch, we call logger.startLogging().
   - Immediately after that, we open /log.txt and append one line of CSV headers.
*/

// ---------------- Include Libraries ----------------
#include <Arduino.h>
#include "RocketIMUFusion_Jerry_032125.h"
#include "ESP32_SPIFFS_DataLogger.h"
#include "ESP32Servo.h"
#include "PIDController_chatgptJerry.h"


// ---------------- Create Global Objects ----------------
RocketIMUFusion_Jerry_032125 rocketIMU;
ESP32_SPIFFS_DataLogger logger;
Servo servoX;
Servo servoY;

// ---------------- User-Configurable Parameters ----------------

// PID values // README: get wifi to ask navya or google what the PID values needs to be; 
//                       send detailed mass and stuff for simulation
// Persistent variables

// Servo angle midpoints & range & TVC
static const int SERVO_MIDPOS = 100;   
static const int SERVO_RANGE  = 40; 
PIDState pitchState, yawState;
PIDConfig pitchConfig = {0.3, 0.001, 0.02, -SERVO_RANGE, SERVO_RANGE};   ////p,i,d
PIDConfig yawConfig   = {0.3, 0.001, 0.02, -SERVO_RANGE, SERVO_RANGE};

// Servo pins
static const int SERVO_X_PIN = 14; // X 18 14
static const int SERVO_Y_PIN = 27; // Y 19 27

// Gains to convert IMU angles => servo commands
static const float SERVO_IMU_GAIN_XZZ = 6;
static const float SERVO_IMU_GAIN_Y   = -9;

// IMU-based launch detection threshold
static const float ACC_THRESHOLD = 2.0f * 9.81f;

// Minimum consecutive loops required for launch detection
static const int CONSEC_LOOPS_REQUIRED = 7;

// Flight duration after launch detection (ms)
static const unsigned long FLIGHT_DURATION = 20000; // 8 seconds

// Mini-calibration cycle (ms) for pre-launch
static const unsigned long MINICALIB_INTERVAL = 5000; // 15 seconds

// Data logging sample interval (ms)
static const int LOG_INTERVAL = 10;  // 10 ms => 100 Hz logging
float dt = LOG_INTERVAL; // README: probably not the right way to do this, but...for now





// ---------------------------------------------------------------
// Global Variables
// ---------------------------------------------------------------

static const int SERVO_MAXPOS = SERVO_MIDPOS + SERVO_RANGE;
static const int SERVO_MINPOS = SERVO_MIDPOS - SERVO_RANGE;

bool launchDetected         = false;  
bool flightActive           = false;  
int  consecutiveAccelCount  = 0;     

unsigned long launchTime    = 0;  
unsigned long lastCalibTime = 0;  
unsigned long lastLogTime   = 0;  

// For reading IMU
float rollZ, pitchX, yawY;
float ax, ay, az; 

// For storing servo outputs in data
int servoCmdX = SERVO_MIDPOS;
int servoCmdY = SERVO_MIDPOS;

unsigned long lastime;





// ---------------- Setup Routine ----------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for Serial */ }

  Serial.println("=== Starting Setup ===");

  // ------------ 1) Initialize Data Logger (but don't start) -------------
  logger.begin();  

  // ------------ 2) Initialize Servos -------------
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoX.attach(SERVO_X_PIN, 1000, 2000); 
  servoY.attach(SERVO_Y_PIN, 1000, 2000);

  servoX.setPeriodHertz(50); 
  servoY.setPeriodHertz(50);

  // Move servos to neutral
  servoX.write(SERVO_MIDPOS);
  servoY.write(SERVO_MIDPOS);
  delay(6000);
  servo_startup_round(2, 800);

  // ------------ 3) IMU Init & Full Calibration -------------
  if (!rocketIMU.begin()) {
    Serial.println("IMU not found! Stopping.");
    while (1) { delay(100); }
  }
  Serial.println("Performing full calibration...");
  rocketIMU.calibrate();
  Serial.println("Full calibration done.");

  // Set up for mini-cal
  lastCalibTime = millis();

  Serial.println("=== Setup Complete ===");
}




// ---------------- Loop Routine ----------------
void loop() {
  unsigned long now = millis();
  
  // -------------------- 1) IMU Update --------------------
  rocketIMU.update();
  rocketIMU.getYPRAccel(rollZ, pitchX, yawY, ax, ay, az);

  // (Optional) Debug printing
  
  Serial.print(" launchstate: ");   Serial.print(launchDetected);
  Serial.print(" time: ");   Serial.print(now);
  Serial.print(" RollZ:");  Serial.print(rollZ,1);
  Serial.print(" PitchX:"); Serial.print(pitchX,1);
  Serial.print(" YawY:");   Serial.print(yawY,1);
  Serial.print(" Ax:");     Serial.print(ax,1);
  Serial.print(" Ay:");     Serial.print(ay,1);
  Serial.print(" Az:");     Serial.print(az,1);
  Serial.print(" min:");     Serial.print(-40);
  Serial.print(" max:");     Serial.print(40);
  Serial.println();
  

  // -------------------- 2) Launch Detection Check --------------------
  if (!launchDetected) {
    float netAccel = sqrtf(ax*ax + ay*ay + az*az);

    if (netAccel > ACC_THRESHOLD) {
      consecutiveAccelCount++;
    } else {
      consecutiveAccelCount = 0;
    }

    if (consecutiveAccelCount >= CONSEC_LOOPS_REQUIRED) {
      // Launch detected
      launchDetected = true;
      flightActive   = true;
      launchTime     = now;
      Serial.println("=== LAUNCH DETECTED ===");

      // Start logging now that launch is confirmed
      logger.startLogging();
      Serial.println("Data logging STARTED.");

      // Write a single line of CSV headers at the top of the file
      File f = SPIFFS.open("/log.txt", FILE_APPEND);
      if (!f) {
        Serial.println("Failed to open /log.txt for CSV header!");
      } else {
        f.println("TimeStamp,PitchX,YawY,RollZ,Ax,Ay,Az,ActX,ActY");
        f.close();
        Serial.println("CSV header line written to /log.txt");
      }
    }
    else {
      // Not launched yet, do mini-cal if time
      if ((now - lastCalibTime) >= MINICALIB_INTERVAL) {
        Serial.println("Performing mini-calibration (pre-launch)...");
        rocketIMU.miniCalibrate();
        lastCalibTime = now;
      }
    }
  }

  // -------------------- 3) Flight Control (TVC) if Active --------------------
  if (flightActive) {
    // int gimbal actuation = PID(float reading, float target, float Ki, float Kd, float Kp)
    
    int cmdX =  -SERVO_IMU_GAIN_XZZ * computePID(rollZ, 0, dt, pitchConfig, pitchState) + SERVO_MIDPOS;
    int cmdY =  -SERVO_IMU_GAIN_Y   * computePID(yawY,  0, dt, yawConfig,     yawState) + SERVO_MIDPOS;
    // pitchX

    // clamp
    if (cmdX > SERVO_MAXPOS) cmdX = SERVO_MAXPOS;
    if (cmdX < SERVO_MINPOS) cmdX = SERVO_MINPOS;
    if (cmdY > SERVO_MAXPOS) cmdY = SERVO_MAXPOS;
    if (cmdY < SERVO_MINPOS) cmdY = SERVO_MINPOS;

    servoX.write(cmdX);
    servoY.write(cmdY);

    servoCmdX = cmdX;
    servoCmdY = cmdY;

    // If flight time has exceeded FLIGHT_DURATION, end flight
    if ((now - launchTime) >= FLIGHT_DURATION) {
      Serial.println("=== Flight duration elapsed. Stopping TVC and Data Logging. ===");
      flightActive = false;

      // Move servos to neutral
      servoX.write(SERVO_MIDPOS);
      servoY.write(SERVO_MIDPOS);

      // Stop logging
      logger.stopLogging();
      Serial.println("Data logging STOPPED.");
    }
  }
  else {
    // If flight isn't active, keep servos at mid
    servoX.write(SERVO_MIDPOS);
    servoY.write(SERVO_MIDPOS);
    servoCmdX = SERVO_MIDPOS;
    servoCmdY = SERVO_MIDPOS;
  }

  // -------------------- 4) Data Logging --------------------
  // only logs if logger is started
  if ((now - lastLogTime) >= LOG_INTERVAL) {
    lastLogTime = now;
    logger.logData(now, pitchX, yawY, rollZ, ax, ay, az, servoCmdX, servoCmdY);
  }

  delay(1);
}


void servo_startup_round(int rounds, int intervals){
  // mid position
  servoX.write(SERVO_MIDPOS);
  servoY.write(SERVO_MIDPOS);
  delay(500);

  for(int i=0; i<rounds; i++){
    servoX.write(SERVO_MAXPOS);
    delay(intervals);
    servoX.write(SERVO_MINPOS);
    delay(intervals);
  }

  // mid position
  servoX.write(SERVO_MIDPOS);
  servoY.write(SERVO_MIDPOS);
  delay(200);

  for(int j=0; j<rounds; j++){
    servoY.write(SERVO_MAXPOS);
    delay(intervals);
    servoY.write(SERVO_MINPOS);
    delay(intervals);
  }

  // mid position
  servoX.write(SERVO_MIDPOS);
  servoY.write(SERVO_MIDPOS);
  delay(500);
}
