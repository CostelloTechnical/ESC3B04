/*
  Relay Output

  Demonstrates how to control the four digital outputs (CH1–CH4) on the
  ESC3B04. Each press of the built-in button advances through a 5-step
  sequence that switches one output on at a time, then turns all outputs
  off on the final step before wrapping back to the start.

  Created 20 Feb 2026
  By James Costello

  https://github.com/CostelloTechnical/ESC3B04
*/

#include "esc3b04.h"

esc3b04 plc;                  // Create an instance of the ESC3B04 library
uint8_t outputSelector = 0;   // Tracks the current step in the output sequence (0–4)

void setup() {
  // Initialise serial at default settings (115200 baud, '\r' end character)
  plc.init();
}

void loop() {
  // Run the library engine — must be called every loop iteration.
  // Handles button debounce, analogue averaging, and serial communication.
  plc.engine();

  // Check if the button has been pressed since the last loop iteration
  if (plc.getButtonPressed()) {

    // Step through the output sequence one press at a time.
    // Each case turns on a single output; the outputs are not cleared between steps,
    // so by step 4 all four outputs will be ON. Case 4 clears them all at once.
    switch (outputSelector) {
      case 0:
        plc.setOutput(CH1, HIGH);  // Turn CH1 on
        break;
      case 1:
        plc.setOutput(CH2, HIGH);  // Turn CH2 on
        break;
      case 2:
        plc.setOutput(CH3, HIGH);  // Turn CH3 on
        break;
      case 3:
        plc.setOutput(CH4, HIGH);  // Turn CH4 on
        break;
      case 4:
        // setOutputs(0) writes a bitmask of 0 to all four outputs simultaneously.
        // Bit 0 = CH1, Bit 1 = CH2, Bit 2 = CH3, Bit 3 = CH4 — all cleared at once.
        plc.setOutputs(0);
        break;
    }

    // Advance to the next step; wrap back to 0 after step 4 (all-off)
    outputSelector = (outputSelector < 4) ? outputSelector + 1 : 0;
  }
}
