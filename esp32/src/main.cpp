#include <Arduino.h>
#include <ArduinoJson.h>
#include <BH1750.h>
#include <DHT.h>
#include <FS.h>
#include <PubSubClient.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>

#define _TASK_STD_FUNCTION
#include <TaskScheduler.h>

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
DHT tempHumidSensorDHT(HH_GPIO_DHT, DHT11);

// Create a wifi client that uses SSL client authentication
WiFiClientSecure wifiClient;
// Create a wifi client that communicates with AWS
PubSubClient pubsubClient(wifiClient);

Scheduler scheduler;

// State manager and hardware controller
HappyHerbsState hhState(lightSensorBH1750, tempHumidSensorDHT, HH_GPIO_LAMP,
                        HH_GPIO_PUMP, HH_GPIO_MOISTURE);
// Service for managing statea and communication with server
HappyHerbsService hhService(hhState, pubsubClient, scheduler);

Task tHappyHerbsServiceLoop(
    TASK_IMMEDIATE, TASK_FOREVER,
    []() {
      if (hhService.connected()) {
        hhService.loop();
        return;
      }
      if (hhService.connect()) {
        hhService.publishShadowUpdate();
      }
    },
    &scheduler, true);

Task tPeriodicSensorsMeasurementsPublish(
    10 * TASK_MINUTE, TASK_FOREVER,
    []() { hhService.publishSensorsMeasurements(); }, &scheduler, true);

Task tPeriodicShadowGetPublish(
    1 * TASK_MINUTE, TASK_FOREVER, []() { hhService.publishShadowGet(); },
    &scheduler, true);

/**
 * This task periodically take measurement on the moisture sensor and compare
 * the result with the user's threshold, if the moisture is not high enough,
 * restart the task that starts the pump
 */
Task taskStartWateringBaseOnMoisture(
    15 * TASK_MINUTE,  // Task's interval (ms)
    TASK_FOREVER,      // Tasks's iterations
    []() {             // Task's callback
      float moisture = hhState.readMoistureSensor();
      if (moisture < hhState.getMoistureThreshold()) {
        Serial.printf("MOISTURE IS LOW %f.2 < %f.2\n", moisture,
                      hhState.getMoistureThreshold());
        hhService.getTaskPlantWatering().restartDelayed();
      }
    },
    &scheduler,  // Tasks scheduler
    false);      // Is task enabled?

/**
 * This task periodically take measurement on the light sensor and compare
 * the result with the user's threshold, if the light level is not high enough,
 * turn on the connected lamp
 */
Task taskTurnOnLampBaseOnLightMeter(
    30 * TASK_MINUTE,  // Task's interval (ms)
    TASK_FOREVER,      // Tasks's iterations
    []() {             // Task's callback
      hhService.writeLampPinID(false);
      float lightLevel = hhState.readLightSensorBH1750();
      if (lightLevel < hhState.getLightThreshold()) {
        Serial.printf("LIGHT LEVEL IS LOW %f.2 < %f.2\n", lightLevel,
                      hhState.getLightThreshold());
        Serial.println("TURN ON LAMP");
        hhService.writeLampPinID(true);
      }
    },
    &scheduler,  // Tasks scheduler
    false);      // Is task enabled?

void setup() {
  pinMode(HH_GPIO_LAMP, OUTPUT);
  pinMode(HH_GPIO_PUMP, OUTPUT);
  pinMode(HH_GPIO_MOISTURE, INPUT);
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial)
    ;
  Wire.begin(I2C_SDA0, I2C_SCL0);

  if (!SPIFFS.begin()) {
    Serial.println("Could not start file system");
    return;
  }

  if (!lightSensorBH1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE_2)) {
    Serial.println("Could not begin BH1750 light sensor");
  }

  // ================ CONNECT TO WIFI AND SETUP LOCAL TIME ================
  char* miscCreds = loadFile(MISC_CREDS.c_str());
  StaticJsonDocument<MQTT_MESSAGE_BUFFER_SIZE> miscCredsJson;
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
  awsEndpoint = loadFile(AWS_IOT_ENDPOINT.c_str());
  if (!awsEndpoint) {
    Serial.println("Could not read AWS endpoint from file");
    return;
  }
  awsRootCACert = loadFile(AWS_ROOTCA_CERT.c_str());
  if (!awsRootCACert) {
    Serial.println("Could not read root CA certificate from file");
    return;
  }
  awsClientCert = loadFile(AWS_CLIENT_CERT.c_str());
  if (!awsClientCert) {
    Serial.println("Could not read client certificate from file");
    return;
  }
  awsClientKey = loadFile(AWS_CLIENT_KEY.c_str());
  if (!awsClientKey) {
    Serial.println("Could not read client key from file");
    return;
  }

  wifiClient.setCACert(awsRootCACert);
  wifiClient.setCertificate(awsClientCert);
  wifiClient.setPrivateKey(awsClientKey);

  pubsubClient.setServer(awsEndpoint, 8883);
  pubsubClient.setBufferSize(MQTT_MESSAGE_BUFFER_SIZE);
  pubsubClient.setCallback([](char* topic, byte* payload, unsigned int length) {
    hhService.handleCallback(topic, payload, length);
  });

  // ================ SETUP STATE AND SERVICE ================
  String awsThingName = loadFile(AWS_THING_NAME.c_str());
  if (!awsThingName) {
    Serial.println("Could not read AWS thing's name from file");
  }
  hhService.setThingName(awsThingName);

  hhState.writeLampPinID(false);
  hhState.writePumpPinID(false);
  hhState.setLightThreshold(DEFAULT_LIGHT_THRESHOLD);
  hhState.setMoistureThreshold(DEFAULT_MOISTURE_THRESHOLD);

  taskTurnOnLampBaseOnLightMeter.enable();
  taskStartWateringBaseOnMoisture.enable();
}

void loop() { scheduler.execute(); }