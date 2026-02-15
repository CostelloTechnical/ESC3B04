#include "esc3b04.h"

esc3b04 plc;

void setup() {
  plc.init();
  plc.setAnalogCalibration(Vi4, 10.0, 1.0, TIME_MS, 100);
}

void loop() {
  plc.engine();
  if (plc.getDataReady()) {
    if(strcmp(plc.getReceivedCharacters(), "Vi1") == 0) plc.println(plc.getAnalogInput(Vi1));
    if(strcmp(plc.getReceivedCharacters(), "Vi2") == 0) plc.println(plc.getAnalogInput(Vi2));
    if(strcmp(plc.getReceivedCharacters(), "Vi3") == 0) plc.println(plc.getAnalogInput(Vi3));
    if(strcmp(plc.getReceivedCharacters(), "Vi4") == 0) plc.println(plc.getAnalogInput(Vi4));
    if(strcmp(plc.getReceivedCharacters(), "Vi5") == 0) plc.println(plc.getAnalogInput(5));
  }
}