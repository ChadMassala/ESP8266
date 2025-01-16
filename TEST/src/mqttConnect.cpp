

 #include "mqttConnect.h"
 #include <PubSubClient.h> // Include MQTT library
 #include <ESP8266WiFi.h> // Include the Wi-Fi library



// MQTT Configuration
const char *MQTT_HOST = "192.168.8.105";
const int MQTT_PORT = 1883;
const char *mqtt_user = "Chad";
const char *mqtt_password = "massala";
const char *mqtt_client_id = "cmteqESP";
const char *topic = "Serial/node1/";

WiFiClient client;
PubSubClient mqttClient(client);


// Connect to MQTT broker
void connectToMQTT() {
     mqttClient.loop(); // Handle MQTT client
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  while (!client.connected()) {
    if (mqttClient.connect(mqtt_client_id)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.println("Failed with state ");
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }
}


// Publish data to MQTT
void publishToMQTT(String topic, String data) {


  if (!mqttClient.connected()) {
    connectToMQTT();
  }
  mqttClient.publish(topic.c_str(), data.c_str());
}




