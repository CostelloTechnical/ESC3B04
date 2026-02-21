/*
  Button Input

  Demonstrates how to use the built-in push button on the ESC3B04 module.
  Each time the button is pressed, a counter increments from 0 to 15 and
  then wraps back to 0. The current counter value is written to all four
  digital outputs (CH1–CH4) as a binary nibble and reported over RS-485.

  Created 20 Feb 2026
  By James Costello

  https://github.com/CostelloTechnical/ESC3B04
*/

#include "esc3b04.h"

esc3b04 plc;             // Create an instance of the ESC3B04 library
uint8_t outputSelector = 0;  // Tracks the current output state (0–15)

void setup() {
  // Initialise serial at default settings (115200 baud, '\r' end character)
  plc.init();

  // Set button debounce window to 50 ms to filter contact bounce
  plc.setButtonDebounce(50);
}

void loop() {
  // Run the library engine — must be called every loop iteration.
  // Handles button debounce, analogue averaging, and serial communication.
  plc.engine();

  // Check if the button has been pressed since the last loop iteration
  if (plc.getButtonPressed()) {

    // Increment the counter; wrap back to 0 after reaching 15 (0b1111)
    outputSelector = (outputSelector < 15) ? outputSelector + 1 : 0;

    // Report the new output value over RS-485
    plc.printf("Button pressed. Output = %d.\r\n", outputSelector);

    // Write the counter value to all four outputs simultaneously as a bitmask.
    // Bit 0 = CH1, Bit 1 = CH2, Bit 2 = CH3, Bit 3 = CH4.
    plc.setOutputs(outputSelector);
  }
}
