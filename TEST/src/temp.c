#include "config.h"
#include "TimeStamp.h"
#include "mqttConnect.h"

#define RX_PIN D2        // GPIO4, connect to module's SDA (TX)
#define SYNC_BYTE 0xAA   // Sync byte
#define MAX_BYTES 64     // Maximum bytes to read

SoftwareSerial softSerial(RX_PIN, -1);

// Function to decode LEB128 values
uint32_t decodeLEB128(uint8_t* buffer, size_t& index, size_t maxLen) {
  uint32_t value = 0;
  uint8_t shift = 0;

  while (index < maxLen) {
    uint8_t byte = buffer[index++];
    value |= (byte & 0x7F) << shift;
    if ((byte & 0x80) == 0) break; 
    shift += 7;
  }
  return value;
}

// Function to compute checksum (Example: XOR of all values)
uint32_t computeChecksum(uint32_t values[], size_t count) {
  uint32_t checksum = 0;
  for (size_t i = 0; i < count; i++) {
    checksum ^= values[i];  // Simple XOR checksum
  }
  return checksum;
}

void setup() {
  Serial.begin(115200);
  softSerial.begin(115200);

  connectToWiFi();
  connectToMQTT();
  
  Serial.println("Listening for software UART data...");
}

void loop() {
  String DateTimes = timeStamps();
  const char* valueNames[] = {"Temperature", "Humidity", "Pressure", "Light", 
                              "CO2", "VOC", "Sound", "Motion", "Battery", 
                              "Signal", "Debounce", "CheckSum"};
  const size_t numNames = sizeof(valueNames) / sizeof(valueNames[0]);

  if (softSerial.available()) {
    uint8_t buffer[MAX_BYTES];
    size_t len = softSerial.readBytes(buffer, MAX_BYTES);

    for (size_t i = 0; i < len; i++) {
      if (buffer[i] == SYNC_BYTE) {
        size_t index = i + 1;
        if (index >= len) break;

        uint8_t length = buffer[index++];
        if (length > numNames) break; // Ensure valid length

        uint32_t values[numNames] = {0};

        Serial.printf("[%s] Decoded LEB128 Data:\n", DateTimes.c_str());
        Serial.println("-------------------------------------------------");

        for (uint8_t j = 0; j < length; j++) {
          if (index >= len) break;
          values[j] = decodeLEB128(buffer, index, len);

          if (j != 10) {  
            Serial.printf("%05u ", values[j]); 
          } else {  
            Serial.printf("%u ", values[j]); 
          }
        }

        // Extract received checksum (last value)
        uint32_t receivedChecksum = values[length - 1];
        
        // Compute checksum from received data (excluding the last value)
        uint32_t computedChecksum = computeChecksum(values, length - 1);

        Serial.printf("\nComputed Checksum: %u, Received Checksum: %u\n", computedChecksum, receivedChecksum);

        if (computedChecksum == receivedChecksum) {
          Serial.println("Checksum Verified ✅ - Publishing Data...");
          for (uint8_t j = 0; j < length - 1; j++) { // Exclude checksum in publishing
            String subtopic = "sensor/data/" + String(valueNames[j]);
            String subtopic1 = "value" + String(j + 1);
            publishToMQTT(subtopic, String(values[j]));
            Serial.printf("%s: %u\n", subtopic1.c_str(), values[j]);
          }
        } else {
          Serial.println("Checksum Mismatch ❌ - Data Rejected.");
        }

        Serial.println("-------------------------------------------------");
        break;
      }
    }
  }
  delay(200);
}
