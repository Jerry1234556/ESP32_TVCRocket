#ifndef ESP32_SPIFFS_DataLogger_h
#define ESP32_SPIFFS_DataLogger_h

#include "Arduino.h"
#include "FS.h"
#include "SPIFFS.h"

class ESP32_SPIFFS_DataLogger {
public:
    ESP32_SPIFFS_DataLogger();
    void begin();
    void startLogging();
    void stopLogging();
    void eraseData();
    void logData(unsigned long currentMillis,
                 float pitchX, float yawY, float rollZ,
                 float ax, float ay, float az,
                 int actX, int actY);
private:
    File _file;
    bool _logging = false;
};

#endif
