#include <Arduino.h>
#include <utils.h>

char readSerial() {
    if (Serial.available() > 0) {
        return Serial.read();
    }
}