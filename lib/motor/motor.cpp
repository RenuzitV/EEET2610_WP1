#include <Arduino.h>
#include <Servo.h>
#include <motor.h>

void setupMotor(Servo& motor) {

    // attach motor
    motor.attach(A6);

    // ESC
    // turning the ESC off and on and write 0 to motor
    pinMode(A9, OUTPUT);
    analogWrite(A9, LOW);
    motor.writeMicroseconds(MIN_SIGNAL);
    // ESC
    
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

    motor.attach(A6);

    // ESC
    pinMode(A9, OUTPUT);
    analogWrite(A9, HIGH);
    delay(100);
    analogWrite(A9, LOW);

    motor.writeMicroseconds(MAX_SIGNAL);

    Serial.println("Turn off power source for 2 seconds, then turn power source on, and press any key to continue");

    while (!Serial.available());
    Serial.read();

    motor.writeMicroseconds(MIN_SIGNAL);
    delay(2000);

    Serial.println("The ESC is calibrated");
}