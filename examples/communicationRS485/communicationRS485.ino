#include "esc3b04.h"

esc3b04 plc;

void setup() {
    plc.init();
}

void loop() {
    plc.engine();
}