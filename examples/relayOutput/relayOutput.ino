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
            outputSelector++;
            case 1:
                plc.setOutput(Vi1, HIGH);
                break;
            case 2:
                plc.setOutput(Vi2, HIGH);
                break;
            case 3:
                plc.setOutput(Vi3, HIGH);
                break;
            case 4:
                plc.setOutput(Vi4, HIGH);
                break;
            case 5:
                plc.setOutputs(0);
                outputSelector = 0;
                break;
        }
    }
}