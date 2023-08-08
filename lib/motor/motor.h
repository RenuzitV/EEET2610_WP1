#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000

const int analogOutPin = A6;  // Analog output pin that the ESC signal is attached to

// set to 0 to do manual configuration, set to 1 to start the program with no manual configuration
#define MOTOR_CALIBRATED 1

// function to calibrate the motor
void setupMotor(Servo& m);
void calibrateMotor(Servo& m);

#endif