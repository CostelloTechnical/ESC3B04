/*
  RS-485 Communication

  Demonstrates how to receive framed messages over RS-485 using the ESC3B04.
  The library buffers incoming bytes until a configured end character arrives,
  then signals that a complete message is ready. Each received message and a
  running message counter are echoed back over RS-485.

  Created 20 Feb 2026
  By James Costello

  https://github.com/CostelloTechnical/ESC3B04
*/

#include "esc3b04.h"

esc3b04 plc;                  // Create an instance of the ESC3B04 library
uint32_t messageCounter = 0;  // Counts the number of complete messages received

void setup() {
  // Initialise RS-485 serial with default settings:
  // 115200 baud, '\r' as the end character, 3000 ms timeout.
  plc.init();

  // The following are equivalent or alternative init() configurations:
  //plc.init(115200, '\r', 3000);       // Identical to the default above
  //plc.init(9600, '<', '>', 1000);     // 9600 baud; message framed between '<' and '>'
  //plc.init(115200, '$', '\n');        // 115200 baud; message starts with '$', ends with '\n'
  //plc.init(230400);                   // 230400 baud; all other settings at default
}

void loop() {
  // Run the library engine — must be called every loop iteration.
  // Handles button debounce, analogue averaging, and serial communication.
  plc.engine();

  // Check if a complete framed message has been received.
  // getDataReady() auto-clears after the first true return, so each message is reported only once.
  // Use getTimedOut() to detect cases where the end character never arrived within the timeout window.
  if (plc.getDataReady()) {

    // Echo the received message back over RS-485.
    // getReceivedCharacters() returns a pointer to the null-terminated internal receive buffer.
    plc.printf("Message received: %s\n\r", plc.getReceivedCharacters());

    // Print the running message count on two separate lines for readability
    plc.print("Message counter: ");
    plc.println(messageCounter);

    messageCounter++;  // Increment counter for the next received message
  }
}
