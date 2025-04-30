#include "FS.h"
#include "SPIFFS.h"

//read test: by chatgpt; non exportable as csv i don't think

void setup() {
    Serial.begin(115200);

    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS Mount Failed");
        return;
    }

    File file = SPIFFS.open("/log.txt", FILE_READ);
    if (!file) {
        Serial.println("Faailed to open file for reading");
        return;
    }

    Serial.println("Reading Logged Data:");
    while (file.available()) {
        Serial.write(file.read());
    }
    file.close();
}

void loop() {
    // Nothing needed here for reading
}
