#ifndef MAIN_H
#define MAIN_H

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include "TaskScheduler.h"

#define RELAY_PIN 4
#define LED_PIN 2
#define MQTT_VOLTAGE_TOPIC "CMTEQ/BATT/Voltage(V)"
#define MQTT_CURRENT_TOPIC "CMTEQ/BATT/Current(A)"
#define MQTT_POWER_TOPIC "CMTEQ/BATT/Power(W)"
#define MQTT_STATE_TOPIC "CMTEQ/BATT/State"


#endif
