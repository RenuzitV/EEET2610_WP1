#include <Arduino.h>
#include <current_sensor_acs712.h>

void calibrateCurrentSensor() {
  initializeCurrentSensor();

  float sum = 0, avg = 0;
  float sampleTimes = 100.0;
  for (int i = 0; i < sampleTimes; i++) {
      sum += (analogRead(CURRENT_DT_PIN));
  }
  avg = sum / sampleTimes;
  Bi = avg;
}

float getCurrentValue() {
  float sum = 0, avg = 0; 
  float sampleTimes = 30.0;
  for (int i = 0; i < sampleTimes; i++) {
    sum += (analogRead(CURRENT_DT_PIN));
  }
  avg = sum/sampleTimes;
  float I = (avg - Bi)/Ai;
  if (I < 0.05) I = 0;
  return I;
}

void initializeCurrentSensor() {
  pinMode(CURRENT_DT_PIN, INPUT);
}
