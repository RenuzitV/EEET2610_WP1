#ifndef CURRENT_SENSOR_ACS712_H
#define CURRENT_SENSOR_ACS712_H

#include <Arduino.h>

#define CURRENT_DT_PIN 2
extern float Ai, Bi; // Analog(currentSensor) = A*I + B
extern float currentOffset;

void calibrateCurrentSensor();
void initializeCurrentSensor();
float getCurrentValue();

#endif
