#ifndef CURRENT_SENSOR_ACS712_H
#define CURRENT_SENSOR_ACS712_H

#include <Arduino.h>

#define CURRENT_DT_PIN 2
static float Ai = 87.869/100*85, Bi = 3145.8; // Analog(currentSensor) = A*I + B
static float currentOffset = 0;

void calibrateCurrentSensor();
void initializeCurrentSensor();
float getCurrentValue();

#endif
