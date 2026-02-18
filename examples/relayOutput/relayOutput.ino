#include "esc3b04.h"

esc3b04 plc;
uint8_t outputSelector = 0;

void setup() {
    plc.init();
}

void loop() {
    plc.engine();
    if(plc.getButtonPressed()){
        switch (outputSelector){
            case 0:
                plc.setOutput(CH1, HIGH);
                break;
            case 1:
                plc.setOutput(CH2, HIGH);
                break;
            case 2:
                plc.setOutput(CH3, HIGH);
                break;
            case 3:
                plc.setOutput(CH4, HIGH);
                break;
            case 4:
                plc.setOutputs(0);
                break;
        }
        outputSelector = (outputSelector < 4) ? outputSelector + 1 : 0; 
    }
}