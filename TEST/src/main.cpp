#include "config.h"
#include "TimeStamp.h"
#include "mqttConnect.h"


#define RX_PIN D2        // GPIO4, connect to module's SDA (TX)
#define SYNC_BYTE 0xAA   // Sync byte
#define MAX_BYTES 64     // Maximum bytes to read

SoftwareSerial softSerial(RX_PIN, -1);

// Variables to save date and time
void emulateDecodeLEB128(uint8_t* buffer, size_t& index, size_t maxLen, uint32_t& value);


// Function to decode LEB128 values
uint32_t decodeLEB128(uint8_t* buffer, size_t& index, size_t maxLen) {
  uint32_t value = 0;
  uint8_t shift = 0;

  // Decode LEB128-encoded value
  while (index < maxLen) {
    uint8_t byte = buffer[index++];
    value |= (byte & 0x7F) << shift;
    if ((byte & 0x80) == 0) break; // Last byte of the current LEB128 value
    shift += 7;
  }
  return value;
}


void setup() {
  Serial.begin(115200);         // Initialize debug Serial Monitor
  softSerial.begin(115200);     // Initialize Software Serial at 115200 baud rate

  connectToWiFi();              // Connect to Wi-Fi
  connectToMQTT();
    // Initialize NTP client
  
  Serial.println("Listening for software UART data...");
}

void loop() {
  
  String DateTimes = timeStamps();

  if (softSerial.available()) {
    uint8_t buffer[MAX_BYTES];
    size_t len = softSerial.readBytes(buffer, MAX_BYTES);

    // Process received data
    for (size_t i = 0; i < len; i++) {
      if (buffer[i] == SYNC_BYTE) { // Check for the sync byte
        size_t index = i + 1;       // Start after sync byte

        if (index >= len) break;

        uint8_t length = buffer[index++]; // Number of LEB128 variables
        
        Serial.print("Decoded LEB128 Data: ");
        Serial.printf("[%s] ", DateTimes.c_str()); // Print the timestamp first
        for (uint8_t j = 0; j < length; j++) {
          if (index >= len) break;

          uint32_t value = decodeLEB128(buffer, index, len);
          // Ensure at least 5 digits
          if (j != 10) {               // 5 digits for all except 11th variable
            Serial.printf("%05u ", value);
          } else {                      // No leading zeros for 11th variable
            Serial.printf("%u ", value);
          }
        }
        Serial.println();
        break; // Process one packet at a time
      }
    }
  }
  

 

  const size_t bufferSize = 64;
  uint8_t buffer[bufferSize];
  size_t index = 0;

  Serial.print("Decoded LEB128 Data: ");
  Serial.printf("%s ", DateTimes.c_str()); // Print the timestamp first
  for (int i = 0; i < 12; i++) {
    uint32_t value = 0;
    emulateDecodeLEB128(buffer, index, bufferSize, value);
    Serial.printf("%05u ", value); // Print each emulated 32-bit integer
  }
  Serial.println();


  String payload =   DateTimes + " Emulated LEB128 Decoded Values: ";
  for (int i = 0; i < 12; i++) {
    uint32_t value = 0;
    emulateDecodeLEB128(buffer, index, bufferSize, value);
    payload += String(value) + " ";


  }
 payload += "}"; // Close the JSON object
  Serial.println(payload);         // Print data to Serial Monitor
  publishToMQTT(payload);          // Publish data to MQTT broker

  delay(2000); // Wait for 2 seconds before running again

}


 
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



