#include "happy_herbs.h"

#include <Arduino.h>

#include "constants.h"
#include "time.h"

HappyHerbsState::HappyHerbsState(BH1750 &lightSensorBH17150,
                                 DHT &tempHumidSensorDHT, int lampPinID,
                                 int pumpPinID, int moistureSensorPinId) {
  this->lightSensorBH1750 = &lightSensorBH17150;
  this->tempHumidSensorDHT = &tempHumidSensorDHT;
  this->lampPinID = lampPinID;
  this->pumpPinID = pumpPinID;
  this->moistureSensorPinID = moistureSensorPinId;
}

/**
 * Performs digital read on the pin
 */
bool HappyHerbsState::readLampPinID() {
  return digitalRead(this->lampPinID) == HIGH;
}

/**
 * Performs digital write on the pin
 */
void HappyHerbsState::writeLampPinID(bool lampState) {
  digitalWrite(this->lampPinID, lampState);
}

bool HappyHerbsState::readPumpPinID() {
  return digitalRead(this->pumpPinID) == HIGH;
}

void HappyHerbsState::writePumpPinID(bool pumpState) {
  digitalWrite(this->pumpPinID, pumpState);
}

/**
 * Reads the value given from the sensor
 */
float HappyHerbsState::readLightSensorBH1750() {
  return this->lightSensorBH1750->readLightLevel();
}

/**
 * NOTE: Due to an issue related to analogRead() causing WiFi to disconnect,
 * this function does not return the real analog readings of the pin.
 *
 * TODO: Implement the function when the issue on github is resolved
 * (https://github.com/espressif/arduino-esp32/issues/4844)
 */
float HappyHerbsState::readMoistureSensor() {
  // return analogRead(this->moistureSensorPinID);
  return 0;
}

float HappyHerbsState::readTemperatureSensor() {
  return this->tempHumidSensorDHT->readTemperature();
}

float HappyHerbsState::readHumiditySensor() {
  return this->tempHumidSensorDHT->readHumidity();
}

void HappyHerbsState::setLightThreshold(float lightThreshold) {
  this->lightThreshold = lightThreshold;
}

void HappyHerbsState::setMoistureThreshold(float moistureThreshold) {
  this->moistureThreshold = moistureThreshold;
}

float HappyHerbsState::getLightThreshold() { return this->lightThreshold; }

float HappyHerbsState::getMoistureThreshold() {
  return this->moistureThreshold;
}

HappyHerbsService::HappyHerbsService(HappyHerbsState &hhState,
                                     PubSubClient &pubsub,
                                     Scheduler &taskScheduler) {
  this->hhState = &hhState;
  this->pubsub = &pubsub;
}

void HappyHerbsService::setThingName(String thingName) {
  this->thingName = thingName;

  String shadowPrefix = "$aws/things/" + thingName + "/shadow";

  this->topicShadowGet = shadowPrefix + "/get";
  this->topicShadowGetAccepted = this->topicShadowGet + "/accepted";
  this->topicShadowGetRejected = this->topicShadowGet + "/rejected";

  this->topicShadowUpdate = shadowPrefix + "/update";
  this->topicShadowUpdateAccepted = this->topicShadowUpdate + "/accepted";
  this->topicShadowUpdateRejected = this->topicShadowUpdate + "/rejected";
  this->topicShadowUpdateDelta = this->topicShadowUpdate + "/delta";
}

/**
 * Calls the underlying PubSubClient loop method
 */
void HappyHerbsService::loop() { this->pubsub->loop(); }

/**
 * Try to recconect to AWS IoT
 */
bool HappyHerbsService::connect() {
  Serial.print("Connecting to AWS IoT @");
  Serial.println(this->thingName);

  bool isConnected = this->pubsub->connect(this->thingName.c_str());
  if (isConnected) {
    Serial.println("-- connected!");

    this->subscribe(this->topicShadowUpdateAccepted.c_str(), 1);
    this->subscribe(this->topicShadowUpdateRejected.c_str(), 1);
    this->subscribe(this->topicShadowUpdateDelta.c_str(), 1);

    this->subscribe(this->topicShadowGetAccepted.c_str(), 1);
    this->subscribe(this->topicShadowGetRejected.c_str(), 1);
  } else {
    Serial.println("-- failed!");
  }

  return isConnected;
}

/**
 * Check if the client is still connected
 */
bool HappyHerbsService::connected() { return this->pubsub->connected(); }

bool HappyHerbsService::publish(const char *topic, const char *payload) {
  bool isSent = this->pubsub->publish(topic, payload);
  if (isSent) {
    Serial.print("SENT [");
    Serial.print(topic);
    Serial.print("]");
    Serial.print(" : ");
    Serial.printf("%s\n", payload);
  }
  return isSent;
}

bool HappyHerbsService::subscribe(const char *topic, unsigned int qos) {
  bool isSubscribed = this->pubsub->subscribe(topic, qos);
  if (isSubscribed) {
    Serial.print("SUBSCRIBED ");
    Serial.println(topic);
  }
  return isSubscribed;
}

/**
 * Receives messages from all topics; this method acts as a controller that
 * routes messages to other handlers that can process the message
 */
void HappyHerbsService::handleCallback(const char *topic, byte *payload,
                                       unsigned int length) {
  Serial.print("RECV [");
  Serial.print(topic);
  Serial.print("]");
  Serial.print(" : ");
  Serial.printf("%s\n", payload);

  if (strcmp(topic, this->topicShadowUpdateDelta.c_str()) == 0) {
    StaticJsonDocument<MQTT_MESSAGE_BUFFER_SIZE> shadowUpdateDeltaJson;
    deserializeJson(shadowUpdateDeltaJson, payload, length);
    this->handleShadowUpdateDelta(shadowUpdateDeltaJson);
  }

  if (strcmp(topic, this->topicShadowUpdateAccepted.c_str()) == 0) {
    StaticJsonDocument<MQTT_MESSAGE_BUFFER_SIZE> shadowUpdateAcceptedJson;
    deserializeJson(shadowUpdateAcceptedJson, payload, length);
    this->handleShadowUpdateAccepted(shadowUpdateAcceptedJson);
  }

  if (strcmp(topic, this->topicShadowUpdateRejected.c_str()) == 0) {
    StaticJsonDocument<MQTT_MESSAGE_BUFFER_SIZE> shadowUpdateRejectedJson;
    deserializeJson(shadowUpdateRejectedJson, payload, length);
    this->handleShadowUpdateRejected(shadowUpdateRejectedJson);
  }
};

/**
 * Publishes a message to the topic "$aws/things/{thing_name}/shadow/update" to
 * announces to client current state
 */
void HappyHerbsService::publishShadowUpdate() {
  StaticJsonDocument<MQTT_MESSAGE_BUFFER_SIZE> shadowUpdateJson;
  JsonObject stateObj = shadowUpdateJson.createNestedObject("state");
  JsonObject reportedObj = stateObj.createNestedObject("reported");
  reportedObj["lampState"] = this->hhState->readLampPinID();
  reportedObj["pumpState"] = this->hhState->readPumpPinID();
  reportedObj["lightThreshold"] = this->hhState->getLightThreshold();
  reportedObj["moistureThreshold"] = this->hhState->getMoistureThreshold();

  char shadowUpdateBuf[MQTT_MESSAGE_BUFFER_SIZE];
  serializeJson(shadowUpdateJson, shadowUpdateBuf);
  this->publish(this->topicShadowUpdate.c_str(), shadowUpdateBuf);
}

/**
 * Handle messages from the topic
 * "$aws/things/{thing_name}/shadow/update/delta". Upon a new message, the
 * client will update it state as given in the message
 */
void HappyHerbsService::handleShadowUpdateDelta(const JsonDocument &deltaDoc) {
  int tsLampState = deltaDoc["metadata"]["lampState"]["timestamp"];
  if (tsLampState > this->tsLampState) {
    bool lampState = deltaDoc["state"]["lampState"];
    this->hhState->writeLampPinID(lampState);
  }

  int tsPumpState = deltaDoc["metadata"]["pumpState"]["timestamp"];
  if (tsPumpState > this->tsPumpState) {
    /**
     * TODO: Turn water on and off after a set interval
     * */
    bool pumpState = deltaDoc["state"]["pumpState"];
    this->hhState->writePumpPinID(pumpState);
  }

  int tsLightThreshold = deltaDoc["metadata"]["lightThreshold"]["timestamp"];
  if (tsLightThreshold > this->tsLightThreshold) {
    float lightThreshold = deltaDoc["state"]["lightThreshold"];
    this->hhState->setLightThreshold(lightThreshold);
  }

  int tsMoistureThreshold =
      deltaDoc["metadata"]["moistureThreshold"]["timestamp"];
  if (tsMoistureThreshold > this->tsMoistureThreshold) {
    float moistureThreshold = deltaDoc["state"]["moistureThreshold"];
    this->hhState->setMoistureThreshold(moistureThreshold);
  }

  this->publishShadowUpdate();
}

void HappyHerbsService::handleShadowUpdateAccepted(
    const JsonDocument &acceptedDoc) {
  if (!acceptedDoc["state"].containsKey("reported")) {
    return;
  }

  bool lampState = acceptedDoc["state"]["reported"]["lampState"];
  bool pumpState = acceptedDoc["state"]["reported"]["pumpState"];
  float lightThreshold = acceptedDoc["state"]["reported"]["lightThreshold"];
  float moistureThreshold =
      acceptedDoc["state"]["reported"]["moistureThreshold"];

  if (lampState != this->hhState->readLampPinID() ||
      pumpState != this->hhState->readPumpPinID() ||
      lightThreshold != this->hhState->getLightThreshold() ||
      moistureThreshold != this->hhState->getMoistureThreshold()) {
    this->publishShadowUpdate();
  }
}

void HappyHerbsService::handleShadowUpdateRejected(
    const JsonDocument &errorDoc) {
  int errCode = errorDoc["code"];
  String errMsg = errorDoc["message"];
  if (errCode == 500) {
    this->publishShadowUpdate();
  }
  Serial.printf("ERR%d : %s", errCode, errMsg);
}

void HappyHerbsService::publishSensorsMeasurements() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return;
  }
  time(&now);

  StaticJsonDocument<MQTT_MESSAGE_BUFFER_SIZE> sensorsJson;
  sensorsJson["timestamp"] = now;
  sensorsJson["thingsName"] = this->thingName;
  sensorsJson["luxBH1750"] = this->hhState->readLightSensorBH1750();
  sensorsJson["moisture"] = this->hhState->readMoistureSensor();
  sensorsJson["temperature"] = this->hhState->readTemperatureSensor();
  sensorsJson["humidity"] = this->hhState->readHumiditySensor();

  char sensorsBuf[MQTT_MESSAGE_BUFFER_SIZE];
  serializeJson(sensorsJson, sensorsBuf);
  this->publish(TOPIC_SENSORS_PUBLISH.c_str(), sensorsBuf);
}