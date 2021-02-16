#include <Arduino.h>
#include <ArduinoJson.h>
#include <BH1750.h>
#include <FS.h>
#include <PubSubClient.h>
#include <SPIFFS.h>
#include <TaskScheduler.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>

#include "constants.h"
#include "happy_herbs.h"
#include "ioutils.h"
#include "time.h"

char* awsEndpoint;
char* awsRootCACert;
char* awsClientCert;
char* awsClientKey;

// Create an object to interact with the light sensor driver
BH1750 lightSensorBH1750(HH_I2C_BH1750_ADDR);

// Create a wifi client that uses SSL client authentication
WiFiClientSecure wifiClient;
// Create a wifi client that communicates with AWS
PubSubClient pubsubClient(wifiClient);

// State manager and hardware controller
HappyHerbsState hhState(lightSensorBH1750, HH_GPIO_LAMP);
// Service for managing statea and communication with server
HappyHerbsService *hhService;

Scheduler taskManager;

Task tReconnectAWSIoT(TASK_SECOND, TASK_FOREVER, []() {
  if (!hhService->connected()) {
    hhService->reconnect();
  }
  hhService->loop();
});

Task tPublishCurrentSensorsMeasurements(10 * 60 * 1000, TASK_FOREVER, []() {
  hhService->publishCurrentSensorsMeasurements();
});

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

  // ================ CONNECT TO WIFI AND SETUP LOCAL TIME ================
  char* miscCreds = loadFile(MISC_CREDS.c_str());
  StaticJsonDocument<512> miscCredsJson;
  deserializeJson(miscCredsJson, miscCreds);
  free(miscCreds);

  Serial.print("Connecting to wifi");
  const String ssid = miscCredsJson["wifiSSID"];
  const String password = miscCredsJson["wifiPass"];
  WiFi.begin(ssid.c_str(), password.c_str());
  Serial.println(ssid.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("connected!");

  int ntpTimezoneOffset = miscCredsJson["ntpTimezoneOffset"];
  int ntpDaylightOffset = miscCredsJson["ntpDaylightOffset"];
  configTime(ntpTimezoneOffset, ntpDaylightOffset, NTP_SERVER.c_str());

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
    hhService->handleCallback(topic, payload, length);
  });

  String awsThingName = loadFile(AWS_THING_NAME.c_str());
  if (!awsThingName) {
    return;
  }
  hhState.writeLampPinID(false);
  hhService = new HappyHerbsService(awsThingName, pubsubClient, hhState);

  // ================ SETUP SCHEDULER ================
  taskManager.init();
  taskManager.addTask(tReconnectAWSIoT);
  taskManager.addTask(tPublishCurrentSensorsMeasurements);

  tReconnectAWSIoT.enable();
  tPublishCurrentSensorsMeasurements.enable();
}

void loop() { taskManager.execute(); }
