<<<<<<< Updated upstream
#include <Arduino.h>
#include <HX711_ADC.h>
#include <loadcell.h>
#include <motor.h>

void loadcell_setup(HX711_ADC* lc) {
    HX711_ADC LoadCell = *lc;

    LoadCell.begin();
    //  LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive

    unsigned long stabilizingtime = millis() + 2000;  // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
    boolean _tare = true;                             // set this to false if you don't want tare to be performed in the next step

    LoadCell.start(stabilizingtime, _tare);

    if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
        // hx711 pin connection is faulty, restart
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
    if (LOADCELL_CALIBRATED) {
        LoadCell.tare();
        LoadCell.setCalFactor(calFactor);
        return;
    }

    Serial.println("***");
    Serial.println("Start calibration:");
    Serial.println("Place the load cell an a level stable surface.");
    Serial.println("Remove any load applied to the load cell.");
    Serial.println("Send 't' from serial monitor to set the tare offset.");
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
        }
        // we get out of the loop if inByte is t and we've finished taring
        if (LoadCell.getTareStatus() == true) {
            Serial.println("Tare complete");
            Serial.printf("using tare offset: %f\n", LoadCell.getTareOffset());  // this line does not work and I do not know why
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

    LoadCell.refreshDataSet();                                           // refresh the dataset to be sure that the known mass is measured correct
    float newCalibrationValue = LoadCell.getNewCalibration(known_mass);  // get the new calibration value

    Serial.print("New calibration value has been set to: ");
    Serial.print(newCalibrationValue);
    Serial.println(", use this as calibration value (calFactor) in your project sketch.");
    Serial.println("End calibration");
}

float readData(HX711_ADC* lc) {
    HX711_ADC LoadCell = *lc;
    // check for new data/start next conversion:
    if (LoadCell.update()){
        newDataReady = true;
    }

    // get smoothed value from the dataset
    if (newDataReady && millis() > (unsigned int)t + loadcellUpdateInterval) {
        // prob dont need kalman filer
        //  float i = simpleKalmanFilter.updateEstimate(LoadCell.getData());
        float i = LoadCell.getData();
        // get a copy of loadcell reading so we can display on the serial monitor
        t = millis();
        newDataReady = 0;
        return i;
    }
=======
#include <Arduino.h>
#include <HX711_ADC.h>
#include <loadcell.h>
#include <motor.h>

void loadcell_setup(HX711_ADC& LoadCell) {

    LoadCell.begin();
     LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive

    unsigned long stabilizingtime = millis() + 2000;  // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
    boolean _tare = true;                             // set this to false if you don't want tare to be performed in the next step

    LoadCell.start(stabilizingtime, _tare);

    if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
        // hx711 pin connection is faulty, restart
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
    if (LOADCELL_CALIBRATED) {
        LoadCell.tare();
        LoadCell.setCalFactor(calFactor);
        return;
    }

    Serial.println("***");
    Serial.println("Start calibration:");
    Serial.println("Place the load cell an a level stable surface.");
    Serial.println("Remove any load applied to the load cell.");
    Serial.println("Send 't' from serial monitor to set the tare offset.");
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
        }
        // we get out of the loop if inByte is t and we've finished taring
        if (LoadCell.getTareStatus() == true) {
            Serial.println("Tare complete");
            Serial.printf("using tare offset: %f\n", LoadCell.getTareOffset());  // this line does not work and I do not know why
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

    LoadCell.refreshDataSet();                                           // refresh the dataset to be sure that the known mass is measured correct
    float newCalibrationValue = LoadCell.getNewCalibration(known_mass);  // get the new calibration value

    Serial.print("New calibration value has been set to: ");
    Serial.print(newCalibrationValue);
    Serial.println(", use this as calibration value (calFactor) in your project sketch.");
    Serial.println("End calibration");
}

void readData(HX711_ADC& LoadCell, double& Input, double& copyInput) {
    // check for new data/start next conversion:
    if (LoadCell.update()) newDataReady = true;

    // get smoothed value from the dataset:
    if (newDataReady && millis() > (unsigned int)t + loadcellUpdateInterval) {
        // prob dont need kalman filer
        //  float i = simpleKalmanFilter.updateEstimate(LoadCell.getData());
        float i = LoadCell.getData();
        // get a copy of loadcell reading so we can display on the serial monitor
        Input = i;
        copyInput = i;
        t = millis();
        newDataReady = 0;
    }
>>>>>>> Stashed changes
}