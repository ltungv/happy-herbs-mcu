#include <Arduino.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <PubSubClient.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "constants.h"
#include "mqtt.h"
#include "wifi.h"

WiFiClientSecure wifiClient;

PubSubClient mqttClient;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(SERIAL_BAUD_RATE);

  if (!SPIFFS.begin()) {
    Serial.println("Could not initialize file system!");
    return;
  }

  if (!wifiConnect(WIFI_CREDS.c_str())) {
    Serial.println("Could not connect to wifi!");
    return;
  }

  if (!wifiSetupClientSecure(wifiClient, AWS_ROOTCA_CERT.c_str(),
                                AWS_DEVICE_CERT.c_str(),
                                AWS_DEVICE_KEY.c_str())) {
    Serial.println("Could not configurate client authentication!");
    return;
  }

  if (!mqttSetupClientSecure(mqttClient, wifiClient,
                                AWS_IOT_ENDPOINT.c_str())) {
    Serial.println("Could not configurate MQTT client!");
    return;
  }
}

void loop() {
  if (!mqttClient.connected()) {
    mqttReconnect(mqttClient);
  }
  mqttClient.loop();
}
