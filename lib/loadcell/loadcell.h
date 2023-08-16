<<<<<<< Updated upstream
#ifndef LOADCELL_H_
#define LOADCELL_H_

#include <Arduino.h>
#include <HX711_ADC.h>
#include <motor.h>

// set to 0 to do manual configuration, set to 1 to start the program with no manual configuration
#define LOADCELL_CALIBRATED 1

// pins:
extern const int HX711_dout;  // mcu > HX711 dout pin
extern const int HX711_sck;   // mcu > HX711 sck pin

// change the known calFactor here
// this value will be used if LOADCELL_CALIBRATED is 0 or we choose 'e' during manual configuration
extern const double calFactor;

extern const int loadcellUpdateInterval;  // increase value to slow down loadcell update activity

// flag when loadcell data is ready
extern boolean newDataReady;

// flag when tare is done
extern bool tareDone;

// time variable for loadcell read interval
extern unsigned long t;

void loadcell_setup(HX711_ADC& ls);
void readData(HX711_ADC& LoadCell, double& Input, double& copyInput);

=======
#ifndef LOADCELL_H_
#define LOADCELL_H_

#include <Arduino.h>
#include <HX711_ADC.h>
#include <motor.h>

// set to 0 to do manual configuration, set to 1 to start the program with no manual configuration
#define LOADCELL_CALIBRATED 1

// pins:
static const int HX711_dout = 7;  // mcu > HX711 dout pin
static const int HX711_sck = 8;   // mcu > HX711 sck pin

const int loadcellUpdateInterval = 0;  // increase value to slow down loadcell update activity

// change the known calFactor here
// this value will be used if LOADCELL_CALIBRATED is 0 or we choose 'e' during manual configuration
const double calFactor = 430.56;

// flag when loadcell data is ready
static boolean newDataReady = 0;

// flag when tare is done
static bool tareDone = true;

// time variable for loadcell read interval
static unsigned long t = 0;

void loadcell_setup(HX711_ADC& ls);
void readData(HX711_ADC& LoadCell, double& Input, double& copyInput);

>>>>>>> Stashed changes
#endif