#include <Arduino.h>
#include <ArduinoJson.h>
#include <BH1750.h>
#include <DHT.h>
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
DHT tempMoisSensorDHT(HH_GPIO_DHT, DHT11);
// Create a wifi client that uses SSL client authentication
WiFiClientSecure wifiClient;
// Create a wifi client that communicates with AWS
PubSubClient pubsubClient(wifiClient);

// State manager and hardware controller
HappyHerbsState hhState(lightSensorBH1750, tempMoisSensorDHT, HH_GPIO_LAMP,
                        HH_GPIO_PUMP, HH_GPIO_MOIS, DEFAULT_LIGHT_THRES,
                        DEFAULT_MOIS_THRES);
// Service for managing statea and communication with server
HappyHerbsService* hhService;

// ================ SETUP SCHEDULER ================

Scheduler taskManager;

Task tReconnectAWSIoT(
    TASK_SECOND, TASK_FOREVER,
    []() {
      if (!hhService->connected()) {
        hhService->reconnect();
      }
      hhService->loop();
    },
    &taskManager, true);

Task tPublishCurrentSensorsMeasurements(
    10 * TASK_MINUTE, TASK_FOREVER,
    []() { hhService->publishCurrentSensorsMeasurements(); }, &taskManager,
    true);

Task tPump(
    3 * TASK_SECOND, TASK_ONCE, NULL, &taskManager, false,
    []() {
      Serial.print("Watering for 3 seconds... ");
      hhState.writePumpPinID(true);
      return true;
    },
    []() {
      Serial.println("\tPump Off");
      hhState.writePumpPinID(false);
    });

Task tPumpInterval(
    15 * TASK_MINUTE, TASK_FOREVER,
    []() {
      if (hhState.readMoisSensor() < hhState.getMoisThreshHold())
        tPump.restartDelayed();
    },
    &taskManager, true);

Task tLampInterval(
    TASK_HOUR, TASK_FOREVER,
    []() {
      hhState.writeLampPinID(false);
      if (hhState.readLightSensorBH1750() < hhState.getLightThreshHold())
        hhState.writeLampPinID(true);
    },
    &taskManager, true);

void setup() {
  pinMode(HH_GPIO_LAMP, OUTPUT);
  pinMode(HH_GPIO_PUMP, OUTPUT);
  pinMode(HH_GPIO_MOIS, INPUT);
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
  hhState.writePumpPinID(false);
  hhService = new HappyHerbsService(awsThingName, pubsubClient, hhState);
}

void loop() { taskManager.execute(); }
