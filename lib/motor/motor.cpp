#include <Arduino.h>
#include <Servo.h>
#include <motor.h>

const int motorOutPin = A9;  // Analog output pin that the ESC signal is attached to

void setupMotor(Servo& motor) {

    // attach motor
    motor.attach(motorOutPin);

    // write 0 to motor
    motor.writeMicroseconds(MIN_SIGNAL);
    
    //done with setup

    //calibration time

    //return if we dont need to calibrate
    if (MOTOR_CALIBRATED) { 
        return;
    }

    while (Serial.available()) {
        Serial.read();
    }

    Serial.println("This program will start the ESC.");

    motor.attach(motorOutPin);

    motor.writeMicroseconds(MAX_SIGNAL);

    Serial.println("Turn off power source for 2 seconds, then turn power source on, and press any key to continue");

    while (!Serial.available());
    Serial.read();

    motor.writeMicroseconds(MIN_SIGNAL);
    delay(2000);

    Serial.println("The ESC is calibrated");
}