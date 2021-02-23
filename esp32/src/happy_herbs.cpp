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
                                     PubSubClient &pubsub) {
  this->hhState = &hhState;
  this->pubsub = &pubsub;
}

/**
 * Calls the underlying PubSubClient loop method
 */
void HappyHerbsService::loop() { this->pubsub->loop(); }

/**
 * Check if the client is still connected
 */
bool HappyHerbsService::connected() { return this->pubsub->connected(); }

/**
 * Try to recconect to AWS IoT
 */
void HappyHerbsService::reconnect() {
  Serial.print("Connecting to AWS IoT @");
  Serial.println(this->thingName);
  if (this->pubsub->connect(this->thingName.c_str())) {
    Serial.println("-- connected!");
    if (this->pubsub->subscribe(this->topicShadowUpdateDelta.c_str(), 1)) {
      Serial.print("SUBSCRIBED ");
      Serial.println(this->topicShadowUpdateDelta);
    };
  } else {
    Serial.println("-- failed!");
  }
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
  Serial.println((char *)payload);

  if (strcmp(topic, this->topicShadowUpdateDelta.c_str()) == 0) {
    StaticJsonDocument<512> shadowUpdateDeltaJson;
    deserializeJson(shadowUpdateDeltaJson, payload, length);
    this->handleShadowUpdateDelta(shadowUpdateDeltaJson);
  }
};

void HappyHerbsService::setThingName(String thingName) {
  this->thingName = thingName;

  this->topicShadowUpdate = "$aws/things/" + thingName + "/shadow/update";
  this->topicShadowUpdateAccepted =
      "$aws/things/" + thingName + "/shadow/update/accepted";
  this->topicShadowUpdateDelta =
      "$aws/things/" + thingName + "/shadow/update/delta";

  this->topicShadowGet = "$aws/things/" + thingName + "/shadow/get";
  this->topicShadowGetAccepted =
      "$aws/things/" + thingName + "/shadow/get/accepted";
}

/**
 * Handle messages from the topic
 * "$aws/things/{thing_name}/shadow/update/delta". Upon a new message, the
 * client will update it state as given in the message
 */
void HappyHerbsService::handleShadowUpdateDelta(const JsonDocument &delta) {
  int tsLampState = delta["metadata"]["lampState"]["timestamp"];
  if (tsLampState > this->tsLampState) {
    bool lampState = delta["state"]["lampState"];
    this->hhState->writeLampPinID(lampState);
  }

  int tsPumpState = delta["metadata"]["pumpState"]["timestamp"];
  if (tsPumpState > this->tsPumpState) {
    /**
     * TODO: Turn water on and off after a set interval
     * */
    bool pumpState = delta["state"]["pumpState"];
    this->hhState->writePumpPinID(pumpState);
  }

  int tsLightThreshold = delta["metadata"]["lightThreshold"]["timestamp"];
  if (tsLightThreshold > this->tsLightThreshold) {
    float lightThreshold = delta["state"]["lightThreshold"];
    this->hhState->setLightThreshold(lightThreshold);
  }

  int tsMoistureThreshold = delta["metadata"]["moistureThreshold"]["timestamp"];
  if (tsMoistureThreshold > this->tsMoistureThreshold) {
    float moistureThreshold = delta["state"]["moistureThreshold"];
    this->hhState->setMoistureThreshold(moistureThreshold);
  }

  this->publishShadowUpdate();
}

/**
 * Publishes a message to the topic "$aws/things/{thing_name}/shadow/update" to
 * announces to client current state
 */
void HappyHerbsService::publishShadowUpdate() {
  StaticJsonDocument<512> shadowUpdateJson;
  JsonObject stateObj = shadowUpdateJson.createNestedObject("state");
  JsonObject reportedObj = stateObj.createNestedObject("reported");
  reportedObj["lampState"] = this->hhState->readLampPinID();
  reportedObj["pumpState"] = this->hhState->readPumpPinID();
  reportedObj["lightThreshold"] = this->hhState->getLightThreshold();
  reportedObj["moistureThreshold"] = this->hhState->getMoistureThreshold();

  char shadowUpdateBuf[512];
  serializeJson(shadowUpdateJson, shadowUpdateBuf);
  this->pubsub->publish(this->topicShadowUpdate.c_str(), shadowUpdateBuf);

  Serial.print("SEND [");
  Serial.print(this->topicShadowUpdate);
  Serial.print("]");
  Serial.print(" : ");
  Serial.println(shadowUpdateBuf);
}

void HappyHerbsService::publishSensorsMeasurements() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return;
  }
  time(&now);

  StaticJsonDocument<512> sensorsJson;
  sensorsJson["timestamp"] = now;
  sensorsJson["thingsName"] = this->thingName;
  sensorsJson["luxBH1750"] = this->hhState->readLightSensorBH1750();
  sensorsJson["moisture"] = this->hhState->readMoistureSensor();
  sensorsJson["temperature"] = this->hhState->readTemperatureSensor();
  sensorsJson["humidity"] = this->hhState->readHumiditySensor();

  char sensorsBuf[512];
  serializeJson(sensorsJson, sensorsBuf);
  this->pubsub->publish(TOPIC_SENSORS_PUBLISH.c_str(), sensorsBuf);

  Serial.print("SEND [");
  Serial.print(TOPIC_SENSORS_PUBLISH);
  Serial.print("]");
  Serial.print(" : ");
  Serial.println(sensorsBuf);
}