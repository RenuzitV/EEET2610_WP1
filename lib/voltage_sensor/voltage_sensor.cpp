#include <Arduino.h>
#include <voltage_sensor.h>

// Define analog input
#define VOLTAGE_DT_PIN 3

// Floats for ADC voltage & Input voltage
float adc_voltage = 0.0;
float in_voltage = 0.0;
float voltageOffset;

// Floats for resistor values in divider (in ohms)
float R1 = 30000.0;
float R2 = 7500.0; 

// Float for Reference Voltage
float ref_voltage = 3.335;

// Integer for ADC value
int adc_value = 0;

void initializeVoltageSensor() {
  pinMode(VOLTAGE_DT_PIN, INPUT);
}

void calibrateVoltageSensor() {
  initializeVoltageSensor();

  // Read the Analog Input
  int x = 300;
  float sum = 0, avgSum;
  for (int i = 0; i < x; i++) {
    adc_value = analogRead(VOLTAGE_DT_PIN);
    sum += adc_value;
  }

  avgSum = sum / x;
  voltageOffset = avgSum;
}

float getVoltageValue() {
  // Read the Analog Input
  int x = 20;
  float sum = 0, avgSum;
  for (int i = 0; i < x; i++) {
    adc_value = analogRead(VOLTAGE_DT_PIN);
    sum += adc_value;
  }

  adc_value = sum/x - voltageOffset;

  // Determine voltage at ADC input
  adc_voltage  = (adc_value * ref_voltage) / 4095.0; 

  // Calculate voltage at divider input
  in_voltage = adc_voltage / (R2/(R1+R2)) ; 
  return in_voltage;
}
