/*
  Simple PLC

  Combines analogue input, digital input, button, and RS-485 communication
  into a minimal working controller. Each of the four digital outputs (CH1–CH4)
  is driven by a different input source to demonstrate how all features work
  together in a single sketch.

  Control logic:
  * CH1 - ON when Vi1 analogue voltage exceeds 4.5 V, OFF otherwise
  * CH2 - Follows Vi2 treated as a digital input (HIGH above ~3.0 V)
  * CH3 - Toggles on each button press
  * CH4 - Controlled by RS-485 commands: send "CH4ON" or "CH4OFF"

  Created 20 Feb 2026
  By James Costello

  https://github.com/CostelloTechnical/ESC3B04
*/

#include "esc3b04.h"

esc3b04 plc;  // Create an instance of the ESC3B04 library

void setup() {
  // Initialise serial at default settings (115200 baud, '\r' end character)
  plc.init();
}

void loop() {
  // Run the library engine — must be called every loop iteration.
  // Handles button debounce, analogue averaging, and serial communication.
  plc.engine();

  // CH1: analogue threshold control.
  // getAnalogInput() returns the instantaneous voltage on Vi1.
  // CH1 turns ON above 4.5 V and OFF at or below it.
  if (plc.getAnalogInput(Vi1) > 4.5) {
    plc.setOutput(CH1, true);
  } else {
    plc.setOutput(CH1, false);
  }

  // CH2: digital input mirror.
  // getDigitalInput() returns 1 (HIGH) when Vi2 exceeds ~1600 mV (~3.0 V scaled).
  // CH2 state directly tracks the Vi2 digital state.
  if (plc.getDigitalInput(Vi2) == true) {
    plc.setOutput(CH2, true);
  } else {
    plc.setOutput(CH2, false);
  }

  // CH3: button toggle.
  // getButtonPressed() returns true once per confirmed button press (auto-clears).
  // getOutput() reads back the current CH3 state so it can be inverted.
  if (plc.getButtonPressed()) {
    plc.setOutput(CH3, !plc.getOutput(CH3));
  }

  // CH4: RS-485 command control.
  // getDataReady() returns true once a complete message has been received (auto-clears).
  // strcmp() returns 0 when the strings match — CH4 is set or cleared by name.
  if (plc.getDataReady() == true) {
    if (strcmp(plc.getReceivedCharacters(), "CH4ON") == 0) {
      plc.setOutput(CH4, true);   // Command "CH4ON"  received — turn CH4 on
    }
    if (strcmp(plc.getReceivedCharacters(), "CH4OFF") == 0) {
      plc.setOutput(CH4, false);  // Command "CH4OFF" received — turn CH4 off
    }
  }
}
