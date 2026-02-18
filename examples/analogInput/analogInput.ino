#include "esc3b04.h"

esc3b04 plc;               // Instantiate the PLC object
uint32_t printTimer_ms;    // Tracks when the last serial print occurred

void setup() {
    plc.init();  // Initialise serial at 115200 baud with '\r' as end character

    // Configure each analogue channel with gain=1.0, offset=0.0, and different averaging modes:
    plc.setAnalogParameters(Vi1, 1.0, 0.0, AVG_DISABLED, 2000); // Vi1: no averaging (live reading)
    plc.setAnalogParameters(Vi2, 1.0, 0.0, TIME_MS, 5000);      // Vi2: average over 5-second window
    plc.setAnalogParameters(Vi3, 1.0, 0.0, READINGS, 10000);    // Vi3: average over 10000 samples
    plc.setAnalogParameters(Vi4, 1.0, 0.0, READINGS, 5000);     // Vi4: average over 5000 samples

    plc.println("");         // Send a blank line to clear the terminal
    printTimer_ms = millis();
}

void loop() {
    plc.engine();  // Run all background engines (button, averaging, serial comms)

    // Print all four channel voltages once per second
    if (millis() - printTimer_ms > 1000) {
        plc.printf("Vi1: %0.02fV\n\r", plc.getAnalogInput(Vi1));    // Instantaneous voltage
        plc.printf("Vi2: %0.02fV\n\r", plc.getAnalogAverage(Vi2));  // 5-second time-averaged voltage
        plc.printf("Vi3: %0.02fV\n\r", plc.getAnalogAverage(Vi3));  // 10000-sample average
        plc.printf("Vi4: %0.02fV\n\r", plc.getAnalogAverage(Vi4));  // 5000-sample average
        plc.println("");
        printTimer_ms = millis();  // Reset the print timer
    }
}
