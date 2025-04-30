#include "SPIFFS.h"

void setup() {
    Serial.begin(115200);
    if (SPIFFS.begin(true)) {
        Serial.println("Formatting SPIFFS...");
        SPIFFS.format(); // Clears all stored data
        Serial.println("Done!");
    } else {
        Serial.println("SPIFFS Mount Failed");
    }
}

void loop() {
  //Serial.println("Jerry: no data");
  //delay(2000);
  //Serial.println("no data");
  //delay(2000);
}
