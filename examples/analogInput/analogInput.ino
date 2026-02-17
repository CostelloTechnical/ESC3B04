#include "esc3b04.h"

esc3b04 plc;
uint32_t printTimer_ms;

void setup() {
  plc.init();
  plc.setAnalogParameters(Vi1, 1.0, 0.0, AVG_DIABLED, 2000);
  plc.setAnalogParameters(Vi2, 1.0, 0.0, TIME_MS, 5000);
  plc.setAnalogParameters(Vi3, 1.0, 0.0, READINGS, 10000);
  plc.setAnalogParameters(Vi4, 1.0, 0.0, READINGS, 5000);
  plc.println("");
  printTimer_ms = millis();
}

void loop() {
  plc.engine();
  if (millis() - printTimer_ms > 1000) {
    plc.printf("Vi1: %0.02fV\n\r", plc.getAnalogAverage(Vi1));
    plc.printf("Vi2: %0.02fV\n\r", plc.getAnalogAverage(Vi2));
    plc.printf("Vi3: %0.02fV\n\r", plc.getAnalogAverage(Vi3));
    plc.printf("Vi4: %0.02fV\n\r", plc.getAnalogAverage(Vi4));
    printTimer_ms = millis();
  }
}