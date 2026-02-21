/*
  Digital Input

  Demonstrates how to read the four analogue inputs (Vi1–Vi4) as digital
  signals on the ESC3B04. An input is treated as HIGH when its ADC reading
  exceeds 1600 mV (~3.0 V on the 0–10 V input range), and LOW otherwise.

  Two equivalent reading methods are shown side-by-side:
  * getDigitalInput(ViX)       : reads a single channel directly
  * bitRead(getDigitalInputs()) : extracts a channel from the packed nibble bitmask

  Created 20 Feb 2026
  By James Costello

  https://github.com/CostelloTechnical/ESC3B04
*/

#include "esc3b04.h"

esc3b04 plc;                   // Create an instance of the ESC3B04 library
uint8_t oldInputsValue = 0;    // Bitmask of input states from the previous loop iteration
uint8_t newInputsValue = 0;    // Bitmask of input states from the current loop iteration

void setup() {
  // Initialise serial at default settings (115200 baud, '\r' end character)
  plc.init();
}

void loop() {
  // engine() is not required for digital input reads, but is included here as
  // good practice — it is needed if button, averaging, or serial features are added later.
  plc.engine();

  // Read all four digital input states packed into a nibble bitmask.
  // Bit 0 = Vi1, Bit 1 = Vi2, Bit 2 = Vi3, Bit 3 = Vi4.
  // Each bit is 1 (HIGH) if the channel voltage exceeds 1600 mV, 0 (LOW) otherwise.
  newInputsValue = plc.getDigitalInputs();

  // Only print when at least one input has changed state since the last loop iteration
  if (newInputsValue != oldInputsValue) {

    // Vi1 and Vi2: read individually using getDigitalInput().
    // Returns 1 (HIGH) or 0 (LOW) for the specified channel.
    plc.print("Vi1 = ");
    plc.println(plc.getDigitalInput(Vi1));

    plc.printf("Vi2 = %d\n\r", plc.getDigitalInput(Vi2));

    // Vi3 and Vi4: extract from the already-read bitmask using bitRead().
    // This avoids a second ADC read and is equivalent to calling getDigitalInput().
    // bitRead(value, bit): bit 2 = Vi3, bit 3 = Vi4.
    plc.print("Vi3 = ");
    plc.println(bitRead(newInputsValue, 2));

    plc.printf("Vi4 = %d\n\r", bitRead(newInputsValue, 3));

    plc.println("");  // Blank line to separate each printed group

    // Save the current state so the next iteration can detect further changes
    oldInputsValue = newInputsValue;
  }
}
