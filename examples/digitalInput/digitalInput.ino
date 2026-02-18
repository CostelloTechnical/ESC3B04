#include "esc3b04.h"

esc3b04 plc;
uint8_t oldInputsValue = 0;
uint8_t newInputsValue = 0;

void setup() {
    plc.init();
}

void loop() {
    plc.engine(); // Not needed for the digital inputs, but we'll include it anyway.
    newInputsValue = plc.getDigitalInputs();
    if(newInputsValue != oldInputsValue){
        plc.print("Vi1 = ");
        plc.println(plc.getDigitalInput(Vi1));

        plc.printf("Vi2 = %d\n\r", plc.getDigitalInput(Vi2));

        plc.print("Vi3 = ");
        plc.println(bitRead(newInputsValue, 2));

        plc.printf("Vi4 = %d\n\r", bitRead(newInputsValue, 3));

        plc.println("");
    }
}