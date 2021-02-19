#include "happy_herbs.h"

#include <Arduino.h>

#include "constants.h"
#include "time.h"

HappyHerbsState::HappyHerbsState(BH1750 &lightSensorBH17150,
                                 DHT &tempHumidSensorDHT, int lampPinID,
                                 int pumpPinID, int moistureSensorPinId,
                                float lightThreshold, float moistureThreshold) {
  this->lightSensorBH1750 = &lightSensorBH17150;
  this->tempHumidSensorDHT = &tempHumidSensorDHT;
  this->lampPinID = lampPinID;
  this->pumpPinID = pumpPinID;
  this->moistureSensorPinID = moistureSensorPinId;
  this->lightThreshold = lightThreshold;
  this->moistureThreshold = moistureThreshold;
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

float HappyHerbsState::readMoistureSensor() {
  return analogRead(this->moistureSensorPinID);
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

float HappyHerbsState::getMoistureThreshold() { return this->moistureThreshold; }

HappyHerbsService::HappyHerbsService(String &thingName, PubSubClient &pubsub,
                                     HappyHerbsState &hhState) {
  this->thingName = thingName;

  this->topicShadowUpdate = "$aws/things/" + thingName + "/shadow/update";
  this->topicShadowUpdateDelta =
      "$aws/things/" + thingName + "/shadow/update/delta";

  this->pubsub = &pubsub;
  this->hhState = &hhState;
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
  Serial.println("Connecting to AWS IoT");
  Serial.println(this->thingName);
  while (!this->pubsub->connected()) {
    if (this->pubsub->connect(this->thingName.c_str())) {
      Serial.println("-- connected!");
      if (this->pubsub->subscribe(this->topicShadowUpdateDelta.c_str(), 1)) {
        Serial.print("SUBSCRIBED ");
        Serial.println(this->topicShadowUpdateDelta);
      };
    } else {
      Serial.println("-- failed!");
      delay(5000);
    }
  }

  delay(500);
  this->publishShadowUpdate();
}

/**
 * Receives messages from all topics; this method acts as a controller that
 * routes messages to other handlers that can process the message
 */
void HappyHerbsService::handleCallback(char *topic, byte *payload,
                                       unsigned int length) {
  if (strcmp(topic, this->topicShadowUpdateDelta.c_str()) == 0) {
    StaticJsonDocument<512> shadowUpdateDeltaJson;
    deserializeJson(shadowUpdateDeltaJson, payload, length);
    this->handleShadowUpdateDelta(shadowUpdateDeltaJson);
  }
};

/**
 * Handle messages from the topic
 * "$aws/things/{thing_name}/shadow/update/delta". Upon a new message, the
 * client will update it state as given in the message
 */
void HappyHerbsService::handleShadowUpdateDelta(const JsonDocument &delta) {
  int ts = delta["timestamp"];
  if (ts > this->lastUpdated) {
    bool lampState = delta["state"]["lampState"];
    bool pumpState = delta["state"]["pumpState"];
    int lightThreshold = delta["state"]["lightThreshold"];
    int pumpThreshold = delta["state"]["pumpThreshold"];

    this->hhState->writePumpPinID(pumpState);
    this->hhState->writeLampPinID(lampState);
    this->hhState->setLightThreshold(lightThreshold);
    this->hhState->setMoistureThreshold(pumpThreshold);
    this->lastUpdated = ts;
    this->publishShadowUpdate();
  }
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

void HappyHerbsService::publishCurrentSensorsMeasurements() {
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
  // sensorsJson["moisture"] = this->hhState->readMoistureSensor();
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