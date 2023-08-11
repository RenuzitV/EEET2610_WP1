#include <Arduino.h>
#include <HX711_ADC.h>
#include <PID_v1.h>
#include <Servo.h>
#include <loadcell.h>
#include <motor.h>
#include <utils.h>
#include <current_sensor_acs712.h>
#include <voltage_sensor.h>
#include <ACS712.h>

// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

int sensorValue = 0;  // value read from the potentialmeter
int outputValue = 0;  // value output to the PWM (motor output)

static double maxSetpoint = 250; //max Setpoint for potentialmeter

// motor class
// doesnt matter that it says servo
Servo motor;

// HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

static double zero;
ACS712 sensor(ACS712_20A, A2);

// Define Variables we'll be connecting to
double Setpoint, Input, Output;

// static variable that tells if the motor is allowed to run
// send 's' in the serial monitor console or s while graph is focused to start/stop the motor
// set to false to stop the motor on startup
static bool startMotor = false;

// Specify the links and initial tuning parameters

// intput, output, setpoint, Kp, Ki, Kd, Proportional on Measurement/Error, Direct/Reversed output (+input->+output or +input->-output)
// BIG BIG ASS PROPELLER
PID myPID(&Input, &Output, &Setpoint, 0.7, 2, 0.0, P_ON_E, DIRECT);  //P_ON_M specifies that Proportional on Measurement be used

// SMALL PROPELLER
// PID myPID(&Input, &Output, &Setpoint, 4.0, 16.0, 0.0, P_ON_E, DIRECT);  // P_ON_M specifies that Proportional on Measurement be used
                                                                        // P_ON_E (Proportional on Error) is the default behavior
//          (P term)    (I term)         (D term)
// output = error*Kp + sum(error)*Ki + derror/dt*Kd
// error = Setpoint - Input
// NOTE: Kd might have a negative term actually, since it is usually used to prevent systems from overshooting.
// basically, Kp tries to fix the error immediately, Ki tries to fix the error over time, and Kd 'predicts' the current 'error fixing rate' and prevents overshooting
// lets say we have a curved up input resulted from our motor spinning faster and faster:
// - error is getting smaller, so our P term also gets smaller.
// - our I term (at this very instance) is also getting smaller, but since it take the sum of ALL errors, it remembers Ki*pastError as pastKiSum and is now adding Ki*error to it. (this also accounts for the drop from the P term as P gets smaller, but we can simplify)
// - our D term looks at the trendline of our input growth, let it be say y = 3x + 5. it calculates the slope (3 in this case) and multiplies it with Kd and adds it to the output. this means that if we're going fast -> slope is high -> more slowdown and vice versa.

// imagine our PID terms looking like this, each dashed line means one unit of output, and we need 10 dashes to reach our Setpoint
//    (P Term)     (I Term)     (D Term)
//  [          ] [          ] [           ] at the start, none of the terms are active, and we have an error E = 10 dashed lines. (ignoring D part, explaination later)
//  [//////    ] [/         ] [           ] after computing, our P term shoots up immediately (Kd*(10 dashed lines) = 7 lines), our I term also shoots up similarly (Ki*(10 dashed lines) = 1 line). this Output is then sent to the motor.
//  [/////     ] [//        ] [           ] the motor spins resulting in less error, so our P term goes down (Kd*(3 lines)). the I term remains and also adds in the current error (1 line + Ki*(3 lines)).
//  [///       ] [//////    ] [           ] the same thing occures: P term goes down, I term remains but adds in even more error (pastKiSum + Ki * missing lines).
//  [//        ] [////////  ] [           ] (you get the point). You may be wondering, "why is the P term removing itself? why is I taking over?" this is totally normal for our propeller system. another example will shed light into this.
//  [          ] [//////////] [           ] this is it. its done. we've drove Input to reach the required Setpoint. the error is now 0 so our P term is no longer affecting Output. The I term also no longer gets added with more stuff (pastKiSum + Ki * 0 lines)

//"WHY IS IT NOT OVERSHOOTING" you may ask, and this is why:
// as the I term grows, the past Ki terms does not move our Input at all. it merely keeps our input to stay in place, while the new error*Ki terms makes the error smaller.
// imagine if we stop computing for new PID values altoghether, and the P, I, D terms stays. what happens to the motor output and input? does the input stay still? does it move?
// the answer is that the input does not move at all. telling the motor to spin at 50% of its power for 2 days does not generate more lift than telling it to spin for 2 seconds. telling it to spin faster does.
// this might be confusing, but imagine a PID system to drive a car. telling the car to move 50km/h for 2 days definitely moves us closer to the setpoint (distance), so we only need the Proportinal term to drive our car.
// however, in the case of our propeller, we need the intergral part to keep adding to our output

// for negative values (or when the setpoint is lower than the input), the opposite occures. it doesnt matter if the P I or D term is not zero, the system would just drive it back to what the setpoint is at.

void setup() {

    // initialize serial communications at 9600 bps:
    Serial.begin(9600);
    // wait for serial communication is ready
    // this requires serial communication to be open for the program to run
    while (!Serial);

    // start calibration procedure
    setupMotor(motor);
    loadcell_setup(LoadCell); 

    // set PID mode to automatic
    // we can set this back to manual to allow
    myPID.SetMode(AUTOMATIC);
    // set the output of our PID to match with the range of our esc's output range
    // this range is calibrated during manual configuration (LOADCELL_CALIBRATED = 0)
    myPID.SetOutputLimits(MIN_SIGNAL, MAX_SIGNAL);

    // confirmation to run the motor if on either manual configuration
    if (!LOADCELL_CALIBRATED || !MOTOR_CALIBRATED) {
        Serial.println("press r key to run motor"); 
		while (readSerial() != 'r');
    } else {
        // skip confirmation and run motor
        Serial.println("PROGRAM WILL START AFTER 2 SECONDS");
        delay(2000);
    }

    analogReadResolution(12);

    // Initialize current + voltage sensor
    Serial.println("disconnect sensor and press any key");
    // calibrateCurrentSensor();
    zero = sensor.calibrate();
    calibrateVoltageSensor();
    while (!readSerial());
}

// time variable for serial update interval
static int tt = 0;

// last Input value i.e. loadcell reading
// since we're setting Input and Output as 0 when we're stopping the motor, we need a copy of the loadcell reading to write back to the serial communication
static double copyInput = 0;

const int serialPrintInterval = 50;  // increase value to slow down serial print activity

//*************************************************** LOOP FUNCTION ***************************************************
void loop() {
    // check for serial input
    // works with both serial communication from ide and matlab
    char inByte = readSerial();

    // turn off/on motor
    if (inByte == 's')
        startMotor = !startMotor;
    // tare the scale again
    else if (inByte == 't') {
        motor.writeMicroseconds(MIN_SIGNAL);
        delay(2000);

        LoadCell.tareNoDelay();  // tare (non blocking)

        tareDone = false;
    }

	readData(LoadCell, Input, copyInput);

    // read the analog in value for potentialmeter
    sensorValue = analogRead(analogInPin);
    // map it to the range of the analog out
    // and set new setpoint for PID

    Setpoint = max(min(map(sensorValue, 150, 4095, 0, maxSetpoint), maxSetpoint), 0);

    // update tare flag status
    if (LoadCell.getTareStatus()) {
        tareDone = true;
    }

    //  //print data to serial
    if (millis() > (unsigned int)tt + serialPrintInterval) {
        // uncomment to force positive data
        //  i = max(i, 0);
        Serial.printf("Load_cell output val: %.2f\n", copyInput);

        // print the results to the Serial Monitor:
        Serial.printf("setpoint: %.2f\n", Setpoint);

        Serial.printf("motor output: %.2f%%\n", (Output - 1000) / 10);
        Serial.printf("time: %d\n", millis());
        
        // float cur = getCurrentValue();
        float vol = getVoltageValue();
        float cur = sensor.getCurrentDC();
        Serial.printf("current: %.2f amp\n", cur);
        Serial.printf("voltage: %.2f vol\n", vol);
        Serial.printf("power: %.2fW\r\n", cur*vol);

        tt = millis();
    }

    // only run the motor is tare operation is done and start motor is true
    if (!startMotor || !tareDone) {
        Input = 0;
        Output = MIN_SIGNAL;
        myPID.SetMode(MANUAL);
        motor.writeMicroseconds(1000);
    } else {
        myPID.SetMode(AUTOMATIC);
        myPID.Compute();
        motor.writeMicroseconds(Output);
    }

    // wait 10 milliseconds before the next loop for the analog-to-digital
    // converter to settle after the last reading:
    // we dont actually need this if we have intervals for every single operation
    // its just good practice and to make sure we dont overload the serial communication
    delay(10);
}