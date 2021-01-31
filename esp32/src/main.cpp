#include <Arduino.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <PubSubClient.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "constants.h"
#include "happy_herbs.h"
#include "ioutils.h"

WiFiClientSecure wifiClient;
PubSubClient pubsubClient(wifiClient);

HappyHerbsState hhState;
HappyHerbsService hhService(pubsubClient, hhState);

char* awsEndpoint;
char* awsRootCACert;
char* awsClientCert;
char* awsClientKey;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  if (!SPIFFS.begin()) {
    return;
  }

  awsEndpoint = loadFile(AWS_IOT_ENDPOINT.c_str());
  if (!awsEndpoint) {
    return;
  }
  awsRootCACert = loadFile(AWS_ROOTCA_CERT.c_str());
  if (!awsRootCACert) {
    return;
  }
  awsClientCert = loadFile(AWS_CLIENT_CERT.c_str());
  if (!awsClientCert) {
    return;
  }
  awsClientKey = loadFile(AWS_CLIENT_KEY.c_str());
  if (!awsClientKey) {
    return;
  }

  // ================ CONNECT TO WIFI NETWORK ================
  char* wifiCreds = loadFile(WIFI_CREDS.c_str());
  StaticJsonDocument<256> wifiCredsJson;
  deserializeJson(wifiCredsJson, wifiCreds);
  free(wifiCreds);

  const String ssid = wifiCredsJson["ssid"];
  const String password = wifiCredsJson["password"];
  Serial.print("Connecting to wifi");
  WiFi.begin(ssid.c_str(), password.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("connected!");

  // ================ SETUP MQTT CLIENT ================
  wifiClient.setCACert(awsRootCACert);
  wifiClient.setCertificate(awsClientCert);
  wifiClient.setPrivateKey(awsClientKey);

  pubsubClient.setServer(awsEndpoint, 8883);
  pubsubClient.setCallback([](char* topic, byte* payload, unsigned int length) {
    Serial.print("RECV [");
    Serial.print(topic);
    Serial.print("]");
    Serial.print(" : ");
    Serial.println((char*)payload);
    hhService.handleCallback(topic, payload, length);
  });

  pinMode(LED_BUILTIN, OUTPUT);
  hhState.setLampPinID(LED_BUILTIN);
  hhState.setLampState(false);
}

void loop() {
  if (!hhService.connected()) {
    hhService.reconnect();
  }
  hhService.loop();
  delay(1000);
}
