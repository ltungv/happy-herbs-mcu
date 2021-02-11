#include <Arduino.h>
#include <ArduinoJson.h>
#include <BH1750.h>
#include <FS.h>
#include <PubSubClient.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>

#include "constants.h"
#include "happy_herbs.h"
#include "ioutils.h"

const int HH_GPIO_LAMP = LED_BUILTIN;
const int HH_GPIO_BH1750_SDA = 8;
const int HH_GPIO_BH1750_SCL = 9;
const int HH_I2C_BH1750_ADDR = 0x23;

char* awsEndpoint;
char* awsRootCACert;
char* awsClientCert;
char* awsClientKey;

// Create a wifi client that uses SSL client authentication
WiFiClientSecure wifiClient;

// Create a wifi client that communicates with AWS
PubSubClient pubsubClient(wifiClient);

// Create an object to interact with the light sensor driver
BH1750 lightSensorBH1750(HH_I2C_BH1750_ADDR);

HappyHerbsState hhState(lightSensorBH1750, HH_GPIO_LAMP);
HappyHerbsService hhService(pubsubClient, hhState);

void setup() {
  pinMode(HH_GPIO_LAMP, OUTPUT);
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial)
    ;
  Wire.begin(HH_GPIO_BH1750_SDA, HH_GPIO_BH1750_SCL);

  if (!lightSensorBH1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE_2)) {
    Serial.println("Could not begin BH1750 light sensor");
  }

  // ================ START FILE SYSTEM AND LOAD CONFIGURATIONS ================
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

  // ================ SET THE INITIAL STATE ================
  hhState.writeLampPinID(false);
}

void loop() {
  if (!hhService.connected()) {
    hhService.reconnect();
  }
  hhService.loop();
  hhService.publishShadowUpdate();
  delay(10000);
}
