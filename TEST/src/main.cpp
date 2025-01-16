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
  
  String DateTimes = timeStamps();  // Get the timestamp
    // Define an array of names for the values
  const char* valueNames[] = {"Temperature", "Humidity", "Pressure", "Light", 
                              "CO2", "VOC", "Sound", "Motion", "Battery", 
                              "Signal", "Debounce", "CheckSum"};
  const size_t numNames = sizeof(valueNames) / sizeof(valueNames[0]);
  if (softSerial.available()) {
    uint8_t buffer[MAX_BYTES];
    size_t len = softSerial.readBytes(buffer, MAX_BYTES);

    // Process received data
    for (size_t i = 0; i < len; i++) {
      if (buffer[i] == SYNC_BYTE) {  // Check for the sync byte
        size_t index = i + 1;       // Start after sync byte

        if (index >= len) break;

        uint8_t length = buffer[index++]; // Number of LEB128 variables
        
        Serial.print("Decoded LEB128 Data: ");
        Serial.printf("[%s] ", DateTimes.c_str()); // Print the timestamp first
        Serial.println();
        Serial.println("----------------------------------------------------------------");

        for (uint8_t j = 0; j < length && j < numNames; j++) {
          if (index >= len) break;

          uint32_t value = decodeLEB128(buffer, index, len);
          // Ensure at least 5 digits
          // if (j != 10) {  // 5 digits for all except 11th variable
          //   Serial.printf("%05u ", value);
          // } else {  // No leading zeros for 11th variable
          //   Serial.printf("%u ", value);
          // }
           // Use the corresponding name for the value
          String name = valueNames[j];

          // Construct the subtopic for each named value
          String subtopic = "sensor/data/" + name;
          String subtopic1 = "value" + String(j + 1);

          // Publish the value to the MQTT broker under the subtopic
          publishToMQTT(subtopic, String(value));

          // Debugging information for the terminal
          Serial.printf("%s: %u\n", subtopic1.c_str(), value);
        }
        break;  // Process one packet at a time
      }
    }
  }
  
  delay(200); // Wait for 1 second before running again
}