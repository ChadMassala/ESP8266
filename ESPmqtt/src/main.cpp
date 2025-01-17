#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include "TaskScheduler.h"

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


String un1, un2, un3;
String readString;
String POW;
int val = 0;
char msg1[50];

#define relay 4
#define LED 2
bool flagEnableRelay1 = false;
//int value = 0;

WiFiClient client;
PubSubClient mqttClient(client);

//========================== 
//
//   S E T U P 
//
//========================== 
void setup() {
  Serial.begin(115200);
  pinMode(relay, OUTPUT);
  pinMode(LED, OUTPUT);
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
    String uno = Serial.readStringUntil('\n');
    int comma1 = uno.indexOf(',');    //12.82,-0.42,1, comma is at 6 and 
    int comma2 = uno.indexOf(',', comma1+1);    //12.82,-0.42,1, comma is at 6 and 
    un1 = uno.substring(0, comma1);  // 
    un2 = uno.substring(comma1 + 1, comma2);  //7 means we stop at 2. 7 means we start at -
    un3 = uno.substring(comma2 + 1); // comma1 is at 6, 12 means we stop after 1, 12 means we start after 0.42,
    POW = un1.toFloat()*un2.toFloat();

    if (un1.toFloat() > 1) {
      
      mqttClient.publish("CMTEQ/BATT/Voltage(V) ", un1.c_str());
      mqttClient.publish("CMTEQ/BATT/Current(A) ", un2.c_str());
      mqttClient.publish("CMTEQ/BATT/Power(W) ", &POW[0]);
      mqttClient.publish("CMTEQ/BATT/State ", un3.c_str());
      delay(50);
    }
    Serial.println("Publishing to CMTEQ/BATT/....");
  }

  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);
}

//======================================================
//
//   W I F I   C O N N E C T
//
//======================================================
void connectWiFi(){
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("Connected to Wi-Fi");
  mqttConnect();
} 

//======================================================
//
//   M Q T T   C O N N E C T
//
//======================================================
void mqttConnect(){
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(callback);

  while (!client.connected()) {
    if (mqttClient.connect(mqtt_client_id)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.println("Failed with state ");
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }
  ESP.wdtFeed(); // "Feed" the watchdog to prevent reset
  mqttClient.subscribe(TOPIC);
}

//======================================================
//
//   C A L L  B A C K
//
//======================================================
void callback(char *topic, byte *payload, unsigned int length) {
  payload[length] = '\0';
  String strTopic = String(topic);
  int lastForwardSlash = strTopic.lastIndexOf('/');
  String strSubTopic = strTopic.substring(lastForwardSlash + 1, strTopic.length());
  String value = String((char *)payload);

  if (value.equals("true")) {
    flagEnableRelay1 = true;
    //value = 1;
    Serial.println(strSubTopic + ":" + value);
  }

  if (value == "false") {
    flagEnableRelay1 = false;
    //value = 0;
    Serial.println(strSubTopic + ":" + value);
  }
  //Serial.println(topic);
}