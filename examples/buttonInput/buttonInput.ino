#include "esc3b04.h"

esc3b04 plc;
uint8_t outputSelector = 0;

void setup() {
    plc.init();
    plc.setButtonDebounce(250);
}

void loop() {
    plc.engine();
    if(plc.getButtonPressed()){
        outputSelector = (outputSelector < 15) ? outputSelector + 1 : 0;
        plc.printf("Button pressed. Output = %d.\r\n", outputSelector);
        plc.setOutputs(outputSelector);
    }
}