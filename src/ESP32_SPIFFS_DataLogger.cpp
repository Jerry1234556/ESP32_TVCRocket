#include "ESP32_SPIFFS_DataLogger.h"

ESP32_SPIFFS_DataLogger::ESP32_SPIFFS_DataLogger() {}

void ESP32_SPIFFS_DataLogger::begin() {
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS Mount Failed");
    }
}

void ESP32_SPIFFS_DataLogger::startLogging() {
    _file = SPIFFS.open("/log.txt", FILE_APPEND);
    if (!_file) {
        Serial.println("Failed to open file for writing");
    } else {
        _logging = true;
    }
}

void ESP32_SPIFFS_DataLogger::stopLogging() {
    if (_logging) {
        _file.close();
        _logging = false;
        Serial.println("Data logged successfully!");
    }
}

void ESP32_SPIFFS_DataLogger::eraseData() {
    if (SPIFFS.begin()) {
        Serial.println("Formatting SPIFFS...");
        SPIFFS.format();
        Serial.println("Done!");
    } else {
        Serial.println("SPIFFS Mount Failed");
    }
}

void ESP32_SPIFFS_DataLogger::logData(unsigned long currentMillis,
                                      float pitchX, float yawY, float rollZ,
                                      float ax, float ay, float az,
                                      int actX, int actY) {
    if (_logging) {
        _file.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d\n",
                     currentMillis, pitchX, yawY, rollZ, ax, ay, az, actX, actY);
    }
}
