#ifndef CURRENT_SENSOR_ACS712_H
#define CURRENT_SENSOR_ACS712_H

#include <Arduino.h>

#define CURRENT_DT_PIN 2
const float Ai = 87.869, Bi = 3145.8; // Analog(currentSensor) = A*I + B

void initializeCurrentSensor();
float getCurrentValue();

#endif
