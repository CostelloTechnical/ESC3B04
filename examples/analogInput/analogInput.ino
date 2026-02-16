#include "esc3b04.h"

esc3b04 plc;
uint32_t timer_ms;

void setup() {
  plc.init();
  if(plc.setAnalogParameters(Vi1, 10.0, 1.0, TIME_MS, 2000) == -1){
    plc.println("Vi1 init error.");
  }
  plc.setAnalogParameters(Vi2, 20.0, 1.0, TIME_MS, 5000);
  plc.setAnalogParameters(Vi3, 100.0, 1.0, READINGS, 10000);
  plc.setAnalogParameters(Vi4, 200.0, 1.0, READINGS, 5000);
  plc.println("");
  timer_ms = millis();
}

void loop() {
  plc.engine();
  if (millis() - timer_ms > 1000) {
    plc.printf("Vi1: %0.02fV\n\r",plc.getAnalogAverage(Vi1));
    //plc.println(plc.getAnalogAverage(Vi1));
    plc.print("Vi2: ");
    plc.println(plc.getAnalogAverage(Vi2));
    plc.print("Vi3: ");
    plc.println(plc.getAnalogAverage(Vi3));
    plc.print("Vi4: ");
    plc.println(plc.getAnalogAverage(Vi4));
    plc.println("");
    timer_ms = millis();
  }
}