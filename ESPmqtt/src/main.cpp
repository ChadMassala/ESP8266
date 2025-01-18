#include <Arduino.h>
#include "main.h"

//+++++++++++++++++++++++++++++++++++++++++++++++
const char *WIFI_SSID = ".NET";
const char *WIFI_PASSWORD = "Massala@31";

const char *MQTT_HOST = "192.168.8.105";
const int MQTT_PORT = 1883;
const char *mqtt_user = "Chad";
const char *mqtt_password = "massala";
const char *mqtt_client_id = "cmteqESP";
const char *TOPIC = "CMTEQ/BATT/";
//+++++++++++++++++++++++++++++++++++++++++++++++++
void connectWiFi();
void mqttConnect();
void callback(char *topic, byte *payload, unsigned int length);


char un1[10], un2[10], un3[10];
float power = 0.0;

int val = 0;
char msg1[50];

bool flagEnableRelay1 = false;
static unsigned long lastBlinkTime = 0;
static bool ledState = false;

WiFiClient client;
PubSubClient mqttClient(client);

//========================== 
//
//   S E T U P 
//
//========================== 
void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT); 
  connectWiFi();
  ArduinoOTA.begin();
}


//========================== 
//
//   L O O P
//
//========================== 
void loop() {
  // Ensure Wi-Fi is connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi connection lost, reconnecting...");
    connectWiFi();
  }

  // Ensure MQTT is connected
  if (!mqttClient.connected()) {
    Serial.println("MQTT connection lost, reconnecting...");
    mqttConnect();
  }

  // Keep the MQTT connection alive
  ArduinoOTA.handle();
  mqttClient.loop();

  if (Serial.available() > 0) {
    char uno[50];
    Serial.readBytesUntil('\n', uno, sizeof(uno));
    uno[sizeof(uno)-1] = '\0'; //Ensure null termination

   // Parse the input string
    if (sscanf(uno, "%9[^,],%9[^,],%9s", un1, un2, un3) == 3) {
      float voltage = atof(un1);
      float current = atof(un2);
      power = voltage * current;

      if(voltage > 1){
        char powerStr[10];
        snprintf(powerStr, sizeof(powerStr), "%.2f", power);
        mqttClient.publish("CMTEQ/BATT/Voltage(V)", un1);
        mqttClient.publish("CMTEQ/BATT/Current(A)", un2);
        mqttClient.publish("CMTEQ/BATT/Power(W)", powerStr);
        mqttClient.publish("CMTEQ/BATT/State", un3);
        delay(50);
      }
    }
    Serial.println("Publishing to CMTEQ/BATT/....");
  }

  if (millis() - lastBlinkTime > 500) { // Toggle every 500ms
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlinkTime = millis();
  }
}

//======================================================
//
//   W I F I   C O N N E C T
//
//======================================================
void connectWiFi(){
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
      if (millis() - startAttemptTime > 10000) { // Timeout after 10 seconds
      Serial.println("Failed to connect to Wi-Fi");
      return;
    }
  }
  Serial.println("Connected to Wi-Fi");
  //mqttConnect();
} 

//======================================================
//
//   M Q T T   C O N N E C T
//
//======================================================
void mqttConnect(){
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(callback);
  int retryCount = 0;

  while (!client.connected() && retryCount < 5) { // Retry 5 times
    Serial.println("Attempting to connect to MQTT broker...");
    if (mqttClient.connect(mqtt_client_id)) {
      Serial.println("Connected to MQTT broker");
      mqttClient.subscribe(TOPIC);
      return;
    } else {
      Serial.println("Failed with state ");
      Serial.println(mqttClient.state());
      retryCount++;
      delay(2000);
    }
  }
  if (!mqttClient.connected()) {
      Serial.println("MQTT connection failed after 5 attempts.");
  }
}

//======================================================
//
//   C A L L  B A C K
//
//======================================================
void callback(char *topic, byte *payload, unsigned int length) {
  if (length >= 50) { // Check for oversized payload
      Serial.println("Payload too large, ignoring");
      return;
  }
  payload[length] = '\0'; // Null-terminate payload
  String strTopic = String(topic);
  String value = String((char *)payload);

  if(value.equals("true")) {
    flagEnableRelay1 = true;
  }else if(value == "false") {
    flagEnableRelay1 = false;
  }
  Serial.println(strTopic + ": " + value);
}