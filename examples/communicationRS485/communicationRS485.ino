#include "esc3b04.h"

esc3b04 plc;
uint32_t messageCounter = 0;

void setup() {
    plc.init(); // The default.
    //plc.init(115200, '\n', 3000); // This is the same as plc.init();
    //plc.init(9600,'<', '>', 1000);
    //plc.init(115200, '$', '\r');
    //plc.init(230400);
}

void loop() {
    plc.engine();
    if(plc.getDataReady()){
        plc.printf("Message received: %s\n\r", plc.getReceivedCharacters());
        plc.print("Message number: ");
        plc.println(messageCounter);
    }
}