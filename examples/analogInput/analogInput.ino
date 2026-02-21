/*
  Analogue Input

  Demonstrates the three analogue averaging modes available on the ESC3B04.
  Each of the four voltage input channels (Vi1–Vi4) is configured differently
  to show how live readings, time-windowed averages, and sample-count averages
  compare in practice. All four channel voltages are printed over RS-485 once
  per second using a non-blocking timer.

  Created 20 Feb 2026
  By James Costello

  https://github.com/CostelloTechnical/ESC3B04
*/

#include "esc3b04.h"

esc3b04 plc;               // Create an instance of the ESC3B04 library
uint32_t printTimer_ms;    // Timestamp (ms) of the last serial print, used for non-blocking 1 s interval

void setup() {
  // Initialise serial at default settings (115200 baud, '\r' end character)
  plc.init();

  // Configure each analogue channel with gain=1.0 and offset=0.0 (no scaling applied).
  // The final argument sets the window size — units depend on the averaging mode:
  //   TIME_MS  : milliseconds
  //   READINGS : number of ADC samples
  //   AVG_DISABLED : argument is ignored
  plc.setAnalogParameters(Vi1, 1.0, 0.0, AVG_DISABLED, 0);      // Vi1: no averaging — getAnalogInput() returns live readings
  plc.setAnalogParameters(Vi2, 1.0, 0.0, TIME_MS,      5000);   // Vi2: average recalculated every 5 seconds
  plc.setAnalogParameters(Vi3, 1.0, 0.0, READINGS,     10000);  // Vi3: average recalculated every 10000 samples
  plc.setAnalogParameters(Vi4, 1.0, 0.0, READINGS,     5000);   // Vi4: average recalculated every 5000 samples

  plc.println("");           // Send a blank line to clear the terminal on connect
  printTimer_ms = millis();  // Seed the print timer
}

void loop() {
  // Run the library engine — must be called every loop iteration.
  // Handles button debounce, analogue averaging, and serial communication.
  plc.engine();

  // Print all four channel voltages once per second (non-blocking)
  if (millis() - printTimer_ms > 1000) {

    // Vi1 has averaging disabled — getAnalogInput() returns the instantaneous voltage
    plc.printf("Vi1: %0.02fV\n\r", plc.getAnalogInput(Vi1));

    // Vi2–Vi4 use averaging — getAnalogAverage() returns the last completed window average.
    // The value only updates when a full window (time or sample count) has elapsed.
    plc.printf("Vi2: %0.02fV\n\r", plc.getAnalogAverage(Vi2));  // 5-second time average
    plc.printf("Vi3: %0.02fV\n\r", plc.getAnalogAverage(Vi3));  // 10000-sample average
    plc.printf("Vi4: %0.02fV\n\r", plc.getAnalogAverage(Vi4));  // 5000-sample average

    plc.println("");           // Blank line to separate each printed group
    printTimer_ms = millis();  // Reset the print timer for the next 1-second interval
  }
}
