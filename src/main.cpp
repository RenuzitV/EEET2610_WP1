/*
  Analog input, analog output, serial output

  Reads an analog input pin, maps the result to a range from 0 to 255 and uses
  the result to set the pulse width modulation (PWM) of an output pin.
  Also prints the results to the Serial Monitor.

  The circuit:
  - potentiometer connected to analog pin 0.
    Center pin of the potentiometer goes to the analog pin.
    side pins of the potentiometer go to +5V and ground
  - LED connected from digital pin 9 to ground through 220 ohm resistor

  created 29 Dec. 2008
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogInOutSerial
*/
#include <Arduino.h>
#include <Servo.h>
#include <HX711_ADC.h>
#include <PID_v1.h>
// #include <SimpleKalmanFilter.h>


#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
// set to 0 to do manual configuration, set to 1 to start the program with no manual configuration
#define CONFIGURED 0
#define RESOLUTION 12

//pins:
const int HX711_dout = 7;  //mcu > HX711 dout pin
const int HX711_sck = 8;   //mcu > HX711 sck pin

// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A0;   // Analog input pin that the potentiometer is attached to
const int analogOutPin = A6;  // Analog output pin that the ESC signal is attached to

int sensorValue = 0;  // value read from the potentialmeter
int outputValue = 0;  // value output to the PWM (motor output)

// change the known calFactor here
// this value will be used if CONFIGURED is 0 or we choose 'e' during manual configuration
double calFactor = -463.31;

//motor class
//doesnt matter that it says servo
Servo motor;

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//static variable that tells if the motor is allowed to run
//send 's' in the serial monitor console or s while graph is focused to start/stop the motor
//set to false to stop the motor on startup
static bool startMotor = true;

//Specify the links and initial tuning parameters

//intput, output, setpoint, Kp, Ki, Kd, Proportional on Measurement/Error, Direct/Reversed output (+input->+output or +input->-output)
//BIG ASS PROPELLER
//PID myPID(&Input, &Output, &Setpoint, 2.0, 4.0, 1.0, P_ON_E, DIRECT);  //P_ON_M specifies that Proportional on Measurement be used

//SMALL PROPELLER
const float Kp = 3.2;
const float Ki = 14;
const float Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, DIRECT);  //P_ON_M specifies that Proportional on Measurement be used
                                                                                                                        //P_ON_E (Proportional on Error) is the default behavior
//          (P term)    (I term)         (D term)
// output = error*Kp + sum(error)*Ki + derror/dt*Kd 
// error = Setpoint - Input
//NOTE: Kd might have a negative term actually, since it is usually used to prevent systems from overshooting.
//basically, Kp tries to fix the error immediately, Ki tries to fix the error over time, and Kd 'predicts' the current 'error fixing rate' and prevents overshooting
//lets say we have a curved up input resulted from our motor spinning faster and faster:
// - error is getting smaller, so our P term also gets smaller.
// - our I term (at this very instance) is also getting smaller, but since it take the sum of ALL errors, it remembers Ki*pastError as pastKiSum and is now adding Ki*error to it. (this also accounts for the drop from the P term as P gets smaller, but we can simplify)
// - our D term looks at the trendline of our input growth, let it be say y = 3x + 5. it calculates the slope (3 in this case) and multiplies it with Kd and adds it to the output. this means that if we're going fast -> slope is high -> more slowdown and vice versa.

//imagine our PID terms looking like this, each dashed line means one unit of output, and we need 10 dashes to reach our Setpoint
//   (P Term)     (I Term)     (D Term)
// [          ] [          ] [           ] at the start, none of the terms are active, and we have an error E = 10 dashed lines. (ignoring D part, explaination later) 
// [//////    ] [/         ] [           ] after computing, our P term shoots up immediately (Kd*(10 dashed lines) = 7 lines), our I term also shoots up similarly (Ki*(10 dashed lines) = 1 line). this Output is then sent to the motor.
// [/////     ] [//        ] [           ] the motor spins resulting in less error, so our P term goes down (Kd*(3 lines)). the I term remains and also adds in the current error (1 line + Ki*(3 lines)). we will call "1 line" as pastKiSum from now on.
// [///       ] [//////    ] [           ] the same thing occures: P term goes down, I term remains but adds in even more error (pastKiSum + Ki * missing lines).
// [//        ] [////////  ] [           ] (you get the point). You may be wondering, "why is the P term removing itself? why is I taking over?" this is totally normal for our propeller system. another example will shed light into this.
// [          ] [//////////] [           ] this is it. its done. we've drove Input to reach the required Setpoint. the error is now 0 so our P term is no longer affecting Output. The I term also no longer gets added with more stuff (pastKiSum + Ki * 0 lines)

//"WHY IS IT NOT OVERSHOOTING" you may ask, and this is why:
// as the I term grows, the past Ki terms does not move our Input at all. it merely keeps our input to stay in place, while the new error*Ki terms makes the error smaller.
// imagine if we stop computing for new PID values altoghether, and the P, I, D terms stays. what happens to the motor output and input? does the input stay still? does it move?
// the answer is that the input does not move at all. telling the motor to spin at 50% of its power for 2 days does not generate more lift than telling it to spin for 2 seconds. telling it to spin faster does.
// this might be confusing, but imagine a PID system to drive a car. telling the car to move 50km/h for 2 days definitely moves us closer to the setpoint (distance), so we only need the Proportinal term to drive our car.
// however, in the case of our propeller, we need the intergral part to keep adding to our output

// for negative values (or when the setpoint is lower than the input), the opposite occures. it doesnt matter if the P I or D term is not zero, the system would just drive it back to what the setpoint needs.



#define VOLTAGE_DT_PIN 3
#define CURRENT_DT_PIN 2
/* Current sensor */
float currentValue, currentOffset;

/* Voltage sensor */
// Floats for ADC voltage & Input voltage
float adc_voltage = 0.0;
float in_voltage = 0.0;
float voltageOffset;
// Floats for resistor values in divider (in ohms)
float R1 = 30000.0;
float R2 = 7500.0; 
// Float for Reference Voltage
float ref_voltage = 3.3;
// Integer for ADC value
int adc_value = 0;


//function to calibrate the motor
void calibrateMotor() {
  while(Serial.available()){Serial.read();}

  Serial.println("This program will start the ESC.");

  motor.attach(A6);

  //ESC
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

//function to calibrate the load cell
void calibrate() {
  LoadCell.begin();
//  LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  
  unsigned long stabilizingtime = millis() + 2000;  // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                  //set this to false if you don't want tare to be performed in the next step
  
  LoadCell.start(stabilizingtime, _tare);

  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    //hx711 pin connection is faulty, restart
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while ((LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag())) {
      delay(1000);
      Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    }
  } else {
    LoadCell.setCalFactor(1.0);  // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }

  // make sure loadcell is ready to be read
  while (!LoadCell.update());

  // if we have already configured, we can tare to 0 offset and set the calibration factor, then exit the calibration
  if (CONFIGURED) {
    LoadCell.tare();
    LoadCell.setCalFactor(calFactor);
    return;
  }

  Serial.println("***");
  Serial.println("Start calibration:");
  Serial.println("Place the load cell an a level stable surface.");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("Send 't' from serial monitor to set the tare offset.");
  Serial.println("Send 'c' from serial monitor to calibrate motor.");
  Serial.println("Send 'e' from serial monitor to use known calFactor.");

  boolean _resume = false;
  boolean known = false;

  // loop for calibration options
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      // default option, taring the load cell
      if (inByte == 't') LoadCell.tareNoDelay();
      // using the known calFactor and quit the calibration process
      else if (inByte == 'e') {
        Serial.printf("using known calFactor: %.2f\n", calFactor);
        LoadCell.setCalFactor(calFactor);
        Serial.printf("using tare offset: %f\n", LoadCell.getTareOffset());
        known = true;
        return;
      }
      // calibrate motor if needed
      else if (inByte == 'c'){
        calibrateMotor();
      }
    }
    // we get out of the loop if inByte is t and we've finished taring 
    if (LoadCell.getTareStatus() == true) {
      Serial.println("Tare complete");
      Serial.printf("using tare offset: %f\n", LoadCell.getTareOffset()); // this line does not work and I do not know why
      _resume = true;
    }
  }
  

  if (known) return;
  Serial.println("Now, place your known mass on the loadcell.");
  Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

  // enter known weight
  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass != 0) {
        Serial.print("Known mass is: ");
        Serial.println(known_mass);
        _resume = true;
      }
    }
  }

  LoadCell.refreshDataSet();                                           //refresh the dataset to be sure that the known mass is measured correct
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass);  //get the new calibration value

  Serial.print("New calibration value has been set to: ");
  Serial.print(newCalibrationValue);
  Serial.println(", use this as calibration value (calFactor) in your project sketch.");
  Serial.println("End calibration");
}

void setup() {

  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  analogReadResolution(RESOLUTION);
  //wait for serial communication is ready
  //this requires serial communication to be open for the program to run
  while (!Serial);

  // Current sensor calibration
  pinMode(CURRENT_DT_PIN, INPUT);
  currentSensorCalibration();

  // Voltage sensor calibration
  pinMode(VOLTAGE_DT_PIN, INPUT);
  voltageSensorCalibration();

  //attach motor
  motor.attach(A6);

  //ESC
  //turning the ESC off and on and write 0 to motor
  pinMode(A9, OUTPUT);
  analogWrite(A9, LOW);
  motor.writeMicroseconds(MIN_SIGNAL);
  //ESC

  //set PID mode to automatic
  //we can set this back to manual to allow 
  myPID.SetMode(AUTOMATIC);
  //set the output of our PID to match with the range of our esc's output range
  //this range is calibrated during manual configuration (CONFIGURED = 0)
  myPID.SetOutputLimits(MIN_SIGNAL, MAX_SIGNAL);

  calibrate();  //start calibration procedure

  // confirmation to run the motor if on manual configuration
  if (!CONFIGURED){
    Serial.print("press r key to run motor");
    bool _resume = false;
    while (_resume == false) {
      LoadCell.update();
      if (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 'r') {
          _resume = true;
        }
      }
    }
  }
  else {
  // skip confirmation and run motor 
    Serial.print("PROGRAM WILL START AFTER 2 SECONDS");
    delay(2000);
  }
}

//time variable for loadcell interval
static unsigned long t = 0;
//time variable for serial update interval
static int tt = 0;

// last Input value i.e. loadcell reading
// since we're setting Input and Output as 0 when we're stopping the motor, we need a copy of the loadcell reading to write back to the serial communication
static float copyInput = 0;

// flag when loadcell data is ready
static boolean newDataReady = 0;

// flag when tare is done
static bool tareDone = true;
const int serialPrintInterval = 50;  //increase value to slow down serial print activity
const int loadcellUpdateInterval = 0;  //increase value to slow down loadcell update activity

//*************************************************** LOOP FUNCTION ***************************************************
void loop() {
  // Current sensor value
  calculateCurrentValue();

  // Voltage sensor value
  calculateVoltageValue();

  //check for serial input
  //works with both serial communication from ide and matlab
  if (Serial.available() > 0) {
    char inByte = Serial.read();

    //turn off/on motor
    if (inByte == 's') startMotor = !startMotor;
    //tare the scale again
    else if (inByte == 't') {
      motor.writeMicroseconds(MIN_SIGNAL);
      delay(2000);

      LoadCell.tareNoDelay();  //tare (non blocking)

      tareDone = false;
    }
    //calibrate the scale (or the motor)
    else if (inByte == 'c') calibrate();
  }


  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out
  //and set new setpoint for PID
  static double maxSetpoint = 100;

  Setpoint = max(min(map(sensorValue, 100, 4096, 0, maxSetpoint), maxSetpoint), 0);

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  //only 
  if (newDataReady && millis() > (unsigned int) t + loadcellUpdateInterval) {
    //prob dont need kalman filer
    // float i = simpleKalmanFilter.updateEstimate(LoadCell.getData());
    float i = LoadCell.getData();
    //get a copy of loadcell reading so we can display on the serial monitor
    Input = i;
    copyInput = i;
    t = millis();
    newDataReady = 0;
  }

  //update tare flag status
  if (LoadCell.getTareStatus()){
    tareDone = true;
  }

  //  //print data to serial
  if (millis() > (unsigned int) tt + serialPrintInterval){
    //uncomment to force positive data
    // i = max(i, 0);
    Serial.printf("Load_cell output val: %.2f\n", copyInput);

    // print the results to the Serial Monitor:
    Serial.printf("setpoint: %.2f\n", Setpoint);

    Serial.printf("motor output: %.2f%%\n", (Output - 1000) / 10);
    Serial.printf("time: %d\n", millis());

    Serial.printf("current: %.2f\n", currentValue);
    Serial.printf("voltage: %.2f\r\n", in_voltage);
    tt = millis();
  }

  // only run the motor is tare operation is done and start motor is true
  if (!startMotor || !tareDone) {
    Input = 0;
    Output = MIN_SIGNAL;
    myPID.SetMode(MANUAL);
    motor.writeMicroseconds(1000);
  }
  else {
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

void calculateCurrentValue() {
  unsigned int x=0;
  float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0;

  for (int x = 0; x < 150; x++) { //Get 150 samples
    AcsValue = analogRead(CURRENT_DT_PIN);     //Read current sensor values   
    Samples = Samples + AcsValue;  //Add samples together
    // delay (3); // let ADC settle before next sample 3ms
  }

  AvgAcs=Samples/150.0;//Taking Average of Samples

  //((AvgAcs * (5.0 / 4096.0)) is converitng the read voltage in 0-5 volts
  //2.5 is offset(I assumed that arduino is working on 5v so the viout at no current comes
  //out to be 2.5 which is out offset. If your arduino is working on different voltage than 
  //you must change the offset according to the input voltage)
  //0.100v(100mV) is rise in output voltage when 1A current flows at input
  AcsValueF = ((AvgAcs * (5 / 4096.0) - 2.5) )/0.100 - currentOffset;
  currentValue = AcsValueF;

  // Ignore value below 0.09
  if (currentValue <= 0.09) {
    currentValue = 0;
  }
}

void currentSensorCalibration() {
  unsigned int x=0;
  float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF;

  for (int x = 0; x < 150; x++) { //Get 150 samples
    AcsValue = analogRead(CURRENT_DT_PIN);     //Read current sensor values   
    Samples = Samples + AcsValue;  //Add samples together
    delay (2); // let ADC settle before next sample 3ms
  }
  AvgAcs=Samples/150.0;
  AcsValueF = ((AvgAcs * (5 / 4096.0) - 2.5) )/0.100;
  currentOffset = AcsValueF;

  Serial.println(currentOffset);
}

void calculateVoltageValue() {
  // Read the Analog Input
  int x = 150;
  float sum = 0, avgSum;
  for (int i = 0; i < x; i++) {
    adc_value = analogRead(VOLTAGE_DT_PIN);
    sum += adc_value;
  }

  adc_value = sum/x - voltageOffset;

  // Determine voltage at ADC input
  adc_voltage  = (adc_value * ref_voltage) / 4096.0; 
  
  // Calculate voltage at divider input
  in_voltage = adc_voltage / (R2/(R1+R2)) ;
}

void voltageSensorCalibration() {
  // Read the Analog Input
  int x = 150;
  float sum = 0, avgSum;
  for (int i = 0; i < x; i++) {
    adc_value = analogRead(VOLTAGE_DT_PIN);
    sum += adc_value;
  }

  avgSum = sum / x;
  voltageOffset = avgSum;
  Serial.println(voltageOffset);
}
