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
DHT tempHumidSensorDHT(HH_GPIO_DHT, DHT11);

// Create a wifi client that uses SSL client authentication
WiFiClientSecure wifiClient;
// Create a wifi client that communicates with AWS
PubSubClient pubsubClient(wifiClient);

// State manager and hardware controller
HappyHerbsState hhState(lightSensorBH1750, tempHumidSensorDHT, HH_GPIO_LAMP,
                        HH_GPIO_PUMP, HH_GPIO_MOISTURE);
// Service for managing statea and communication with server
HappyHerbsService hhService(hhState, pubsubClient);

Scheduler taskManager;

/**
 * This task immediately publishes the current state of the system to update AWS
 * thing's shadow
 */
Task tPublishShadowUpdate(
    TASK_IMMEDIATE,                             // Task's interval (ms)
    TASK_ONCE,                                  // Tasks's iterations
    []() { hhService.publishShadowUpdate(); },  // Task's callback
    &taskManager,                               // Tasks scheduler
    false);                                     // Is task enabled?

/**
 * This task immediately take measurement from every sensor and publishes the
 * data to AWS
 */
Task tPublishSensorsMeasurements(
    TASK_IMMEDIATE,                                    // Task's interval (ms)
    TASK_ONCE,                                         // Tasks's iterations
    []() { hhService.publishSensorsMeasurements(); },  // Task's callback
    &taskManager,                                      // Tasks scheduler
    false);                                            // Is task enabled?

/**
 * This task turns on the water pump when started, then after its designated
 * interval has passed, the task stops and turns off the water pump
 */
Task tTurnOnWaterPump(
    5 * TASK_SECOND,  // Task's interval (ms)
    TASK_FOREVER,     // Tasks's iterations
    NULL,             // Task's callback
    &taskManager,     // Tasks scheduler
    false,            // Is task enabled?
    []() {
      Serial.println("Watering for 3 seconds... ");
      hhState.writePumpPinID(true);
      tPublishShadowUpdate.restart();
      return true;
    },  // Function to call when task is enabled
    []() {
      Serial.println("Stop watering");
      hhState.writePumpPinID(false);
      tPublishShadowUpdate.restart();
    });  // Function to call when task is disabled

/**
 * This task tries to reconnect to AWS MQTT broker if the service is
 * disconnected, upon reconnection, publish a message to report the system
 * current state
 */
Task tHappyHerbsServiceReconnect(
    5 * TASK_SECOND,                  // Task's interval (ms)
    TASK_FOREVER,                     // Tasks's iterations
    []() { hhService.reconnect(); },  // Task's callback
    &taskManager,                     // Tasks scheduler
    true,                             // Is task enabled?
    []() {
      return !hhService.connected();
    },  // Function to call when task is enabled
    []() {
      tPublishShadowUpdate.restartDelayed(TASK_SECOND);
    });  // Function to call when task is disabled

/**
 * Call the loop() function on HappyHerbsService so that MQTT messages can be
 * processed
 */
Task tHappyHerbsServiceLoop(
    TASK_IMMEDIATE,              // Task's interval (ms)
    TASK_FOREVER,                // Tasks's iterations
    []() { hhService.loop(); },  // Task's callback
    &taskManager,                // Tasks scheduler
    true);                       // Is task enabled?

/**
 * This task periodically restart the task for updating AWS thing's shadow
 */
Task tPeriodicSensorsMeasurementsPublish(
    10 * TASK_MINUTE,                                 // Task's interval (ms)
    TASK_FOREVER,                                     // Tasks's iterations
    []() { tPublishSensorsMeasurements.restart(); },  // Task's callback
    &taskManager,                                     // Tasks scheduler
    true);                                            // Is task enabled?

/**
 * This task periodically take measurement on the moisture sensor and compare
 * the result with the user's threshold, if the moisture is not high enough,
 * restart the task that starts the pump
 */
Task tTurnOnWaterPumpOnMoisture(
    15 * TASK_MINUTE,  // Task's interval (ms)
    TASK_FOREVER,      // Tasks's iterations
    []() {             // Task's callback
      if (hhState.readMoistureSensor() < hhState.getMoistureThreshold()) {
        tTurnOnWaterPump.restart();
      }
    },
    &taskManager,  // Tasks scheduler
    true);         // Is task enabled?

/**
 * This task periodically take measurement on the light sensor and compare
 * the result with the user's threshold, if the light level is not high enough,
 * turn on the connected lamp
 */
Task tTurnOnLampBaseOnLightMeter(
    30 * TASK_MINUTE,  // Task's interval (ms)
    TASK_FOREVER,      // Tasks's iterations
    []() {             // Task's callback
      hhState.writeLampPinID(false);
      if (hhState.readLightSensorBH1750() < hhState.getLightThreshold()) {
        hhState.writeLampPinID(true);
      }
      tPublishShadowUpdate.restart();
    },
    &taskManager,  // Tasks scheduler
    true);         // Is task enabled?

void setup() {
  pinMode(HH_GPIO_LAMP, OUTPUT);
  pinMode(HH_GPIO_PUMP, OUTPUT);
  pinMode(HH_GPIO_MOISTURE, INPUT);
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial)
    ;
  Wire.begin(I2C_SDA0, I2C_SCL0);

  if (!SPIFFS.begin()) {
    return;
  }

  if (!lightSensorBH1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE_2)) {
    Serial.println("Could not begin BH1750 light sensor");
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

  // ================ SETUP STATE AND SERVICE ================
  String awsThingName = loadFile(AWS_THING_NAME.c_str());
  if (!awsThingName) {
    return;
  }
  hhService.setThingName(awsThingName);

  hhState.writeLampPinID(false);
  hhState.writePumpPinID(false);
  hhState.setLightThreshold(DEFAULT_LIGHT_THRESHOLD);
  hhState.setMoistureThreshold(DEFAULT_MOISTURE_THRESHOLD);
}

void loop() { taskManager.execute(); }