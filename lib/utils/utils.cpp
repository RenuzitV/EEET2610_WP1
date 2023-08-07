#include <Arduino.h>
#include <utils.h>

char readSerial() {
    while (true) {
        if (Serial.available() > 0) {
            return Serial.read();
        }
    }
}