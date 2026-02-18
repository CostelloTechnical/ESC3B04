/*
This example pulls together elements from the other examples into
a simple controller.
*/

#include "esc3b04.h"

esc3b04 plc;

void setup() {
    plc.init(); // Uses the default communication parameters.
}

void loop() {
    plc.engine(); // Operates the analog averaging, button and RS485 communication
    if(plc.getAnalogInput(Vi1) > 4.5){
        plc.setOutput(CH1, true);
    }
    else {
        plc.setOutput(CH1, false);
    }

    if(plc.getDigitalInput(Vi2) == true){
        plc.setOutput(CH2, true);
    } 
    else {
        plc.setOutput(CH2, false);
    }

    if(plc.getButtonPressed()){
        plc.setOutput(CH3, !plc.getOutput(CH3));
    }

    if(plc.getDataReady() == true){
        if(strcmp(plc.getReceivedCharacters(),"CH4_ON")==0){
            plc.setOutput(CH4, true);
        }
        if(strcmp(plc.getReceivedCharacters(),"CH4_OFF")==0){
            plc.setOutput(CH4, false);
        }
    }
}