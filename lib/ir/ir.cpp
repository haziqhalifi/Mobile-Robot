/*********************************************
 * 1. Designed for the Raspberry Pi Pico.
 * 2. Timing values assume a 125 MHz clock.
 * 3. Date: 2021-12-03
 * 4. Programmer: Jieliang Mo
 * 5. https://github.com/earlephilhower/arduino-pico/
 * 6. https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
 *********************************************/

#include "ir.h"

/////////////////////////////////////////////////////////
// Create an IR receiver that reads its signal from pin p.
IR::IR(int p) {
  pin_ = p;
  pinMode(pin_, INPUT);
}

/////////////////////////////////////////////////////////
// Check for the NEC IR protocol's leading pulse.
// A normal NEC frame begins with about 9 ms LOW followed by 4.5 ms HIGH.
boolean IR::IRStart(void) {
  int t = 0;

  // Measure the duration of the initial LOW pulse in 50 us steps.
  if (digitalRead(pin_) == LOW) {
    while (digitalRead(pin_) == LOW && t < 190) {
      delayMicroseconds(50);
      t++;
    }
  }

  // 180 counts x 50 us is approximately 9 ms.
  // Allow a small timing tolerance when recognizing the start pulse.
  if (t > 170 && t < 190) {
    return true;
  }

  return false;
}

/////////////////////////////////////////////////////////
// Read one 8-bit value from the 32-bit NEC message.
// The full message contains: address, inverse address, command,
// and inverse command.
int IR::getByte(void) {
  int Byte = 0;

  for (char i = 0; i < 8; i++) {
    int t = 0;

    // Every data bit starts with a LOW pulse. Wait for it to finish.
    while (digitalRead(pin_) == LOW);

    // Measure the following HIGH pulse in 50 us steps.
    // A longer HIGH pulse (about 1.69 ms) represents binary 1.
    if (digitalRead(pin_) == HIGH) {
      while (digitalRead(pin_) == HIGH && t < 38) {
        delayMicroseconds(50);
        t++;
      }
    }

    // If the HIGH pulse is long enough, set bit i.
    // NEC sends the least-significant bit first.
    if (t > 20 && t < 38) {
      Byte |= 1 << i;
    }
  }

  return Byte;
}

/////////////////////////////////////////////////////////
// Read and validate a complete NEC message.
// Return the command/key value, or -1 if no valid message is received.
int IR::getKey(void) {
  int key[4] = {0, 0, 0, 0};

  // Reject the signal if the 9 ms leading pulse is missing.
  if (IRStart() == false) {
    // A complete NEC message frame lasts approximately 108 ms.
    return -1;
  }

  // Wait until the 4.5 ms HIGH part of the start signal ends.
  while (digitalRead(pin_) == HIGH);

  // Read address, inverse address, command, and inverse command.
  for (char i = 0; i < 4; i++) {
    key[i] = getByte();
  }

  // Each normal byte and its inverse must add up to 0xFF.
  // This verifies that the received data is not corrupted.
  if (key[0] + key[1] == 0xff && key[2] + key[3] == 0xff) {
    return key[2];
  }

  return -1;
}
