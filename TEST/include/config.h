#include "Arduino.h"
#include <SoftwareSerial.h>
#include "WiFiConnection.h"

// Variables to save date and time
void emulateDecodeLEB128(uint8_t* buffer, size_t& index, size_t maxLen, uint32_t& value);


void emulateDecodeLEB128(uint8_t* buffer, size_t& index, size_t maxLen, uint32_t& value) {
  value = 0;
  uint8_t shift = 0;

  while (index < maxLen) {
    uint8_t byte = random(0, 128); // Generate a random byte (0-127)
    if (random(0, 2) == 0) {       // Randomly decide if it's the last byte
      byte |= 0x80;
    }
    buffer[index++] = byte;        // Store it in the buffer
    value |= (byte & 0x7F) << shift;
    shift += 7;

    if ((byte & 0x80) == 0) break; // Stop if it's the last byte
  }
}
