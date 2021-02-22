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

// ================ SETUP SCHEDULER ================
Scheduler taskManager;

// ================ SETUP SCHEDULER: ONE TIME TASKS ================
void tPublishShadowUpdateCallback();
Task tPublishShadowUpdate(TASK_IMMEDIATE, TASK_ONCE,
                          &tPublishShadowUpdateCallback, &taskManager, false);

void tPublishSensorsMeasurementsCallback();
Task tPublishSensorsMeasurements(TASK_IMMEDIATE, TASK_ONCE,
                                 &tPublishSensorsMeasurementsCallback,
                                 &taskManager, false);

bool tTurnOnWaterPumpOnEnable();
void tTurnOnWaterPumpOnDisable();
Task tTurnOnWaterPump(3 * TASK_SECOND, TASK_ONCE, NULL, &taskManager, false,
                      &tTurnOnWaterPumpOnEnable, &tTurnOnWaterPumpOnDisable);

// ================ SETUP SCHEDULER: PEDIODIC TASKS ================
void tHappyHerbsServiceReconnectCallback();
bool tHappyHerbsServiceReconnectOnEnable();
void tHappyHerbsServiceReconnectOnDisable();
Task tHappyHerbsServiceReconnect(5 * TASK_SECOND, TASK_FOREVER,
                                 &tHappyHerbsServiceReconnectCallback,
                                 &taskManager, true,
                                 &tHappyHerbsServiceReconnectOnEnable,
                                 &tHappyHerbsServiceReconnectOnDisable);

void tHappyHerbsServiceLoopCallback();
Task tHappyHerbsServiceLoop(TASK_IMMEDIATE, TASK_FOREVER,
                            &tHappyHerbsServiceLoopCallback, &taskManager,
                            true);

void tPeriodicSensorsMeasurementsPublishCallback();
Task tPeriodicSensorsMeasurementsPublish(
    10 * TASK_MINUTE, TASK_FOREVER,
    &tPeriodicSensorsMeasurementsPublishCallback, &taskManager, true);

void tTurnOnWaterPumpBaseOnMoistureCallback();
Task tTurnOnWaterPumpOnMoisture(15 * TASK_MINUTE, TASK_FOREVER,
                                &tTurnOnWaterPumpBaseOnMoistureCallback,
                                &taskManager, true);

void tTurnOnLampBaseOnLightMeterCallback();
Task tTurnOnLampBaseOnLightMeter(30 * TASK_MINUTE, TASK_FOREVER,
                                 &tTurnOnLampBaseOnLightMeterCallback,
                                 &taskManager, true);

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

// ================ PERIODIC TASKS ================
void tHappyHerbsServiceReconnectCallback() { hhService.reconnect(); }

bool tHappyHerbsServiceReconnectOnEnable() { return !hhService.connected(); }

void tHappyHerbsServiceReconnectOnDisable() {
  tPublishShadowUpdate.restartDelayed(TASK_SECOND);
};

void tHappyHerbsServiceLoopCallback() { hhService.loop(); };

void tPeriodicSensorsMeasurementsPublishCallback() {
  tPublishSensorsMeasurements.restart();
};

void tTurnOnWaterPumpBaseOnMoistureCallback() {
  if (hhState.readMoistureSensor() < hhState.getMoistureThreshold())
    tTurnOnWaterPump.restart();
};

void tTurnOnLampBaseOnLightMeterCallback() {
  hhState.writeLampPinID(false);
  if (hhState.readLightSensorBH1750() < hhState.getLightThreshold())
    hhState.writeLampPinID(true);

  tPublishShadowUpdate.restart();
};

// ================ ONE TIME TASKS ================
void tPublishShadowUpdateCallback() { hhService.publishShadowUpdate(); };

void tPublishSensorsMeasurementsCallback() {
  hhService.publishSensorsMeasurements();
};

bool tTurnOnWaterPumpOnEnable() {
  Serial.print("Watering for 3 seconds... ");
  hhState.writePumpPinID(true);
  tPublishShadowUpdate.restart();
  return true;
};

void tTurnOnWaterPumpOnDisable() {
  Serial.println("\tPump Off");
  hhState.writePumpPinID(false);
  tPublishShadowUpdate.restart();
};
