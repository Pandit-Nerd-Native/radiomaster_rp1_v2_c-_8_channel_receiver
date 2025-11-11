#include <Arduino.h>
#include "CRSFSerial.h"

#define RXD2 16  // RP1 TX → ESP32 RX
#define TXD2 17  // RP1 RX → ESP32 TX

CRSF crsf;  // Create CRSF instance

void setup() {
    Serial.begin(115200);
    Serial2.begin(420000, SERIAL_8N1, RXD2, TXD2);  // CRSF baud rate
    crsf.begin(&Serial2);
}

void loop() {
    crsf.readPacket();  // Read CRSF data

    // Display only active channels
    for (int ch = 0; ch < 8; ch++) {
        uint16_t value = crsf.getChannel(ch);
        if (value > 900) {  // Ignore invalid values
            Serial.print("CH");
            Serial.print(ch + 1);
            Serial.print(": ");
            Serial.print(value);
            Serial.print("  ");
        }
    }
    Serial.println();
    delay(100); // Update every 100ms
}
