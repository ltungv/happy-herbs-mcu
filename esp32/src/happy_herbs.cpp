#include "happy_herbs.h"

#include <Arduino.h>

#include "constants.h"
#include "time.h"

HappyHerbsState::HappyHerbsState(BH1750 &lightSensorBH17150, int lampPinID) {
  this->lightSensorBH1750 = &lightSensorBH17150;
  this->lampPinID = lampPinID;
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

/**
 * Reads the value given from the sensor
 */
float HappyHerbsState::readLightSensorBH1750() {
  return this->lightSensorBH1750->readLightLevel();
}

HappyHerbsService::HappyHerbsService(PubSubClient &pubsub,
                                     HappyHerbsState &hhState) {
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
  while (!this->pubsub->connected()) {
    if (this->pubsub->connect(AWS_THING_NAME.c_str())) {
      Serial.println("-- connected!");
      if (this->pubsub->subscribe(TOPIC_SHADOW_UPDATE_DELTA.c_str(), 1)) {
        Serial.print("SUBSCRIBED ");
        Serial.println(TOPIC_SHADOW_UPDATE_DELTA);
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
  if (strcmp(topic, TOPIC_SHADOW_UPDATE_DELTA.c_str()) == 0) {
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
    this->hhState->writeLampPinID(lampState);
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

  char shadowUpdateBuf[512];
  serializeJson(shadowUpdateJson, shadowUpdateBuf);
  this->pubsub->publish(TOPIC_SHADOW_UPDATE.c_str(), shadowUpdateBuf);

  Serial.print("SEND [");
  Serial.print(TOPIC_SHADOW_UPDATE);
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
  sensorsJson["thingsName"] = AWS_THING_NAME;
  sensorsJson["luxBH1750"] = this->hhState->readLightSensorBH1750();

  char sensorsBuf[512];
  serializeJson(sensorsJson, sensorsBuf);
  this->pubsub->publish(TOPIC_SENSORS_PUBLISH.c_str(), sensorsBuf);

  Serial.print("SEND [");
  Serial.print(TOPIC_SENSORS_PUBLISH);
  Serial.print("]");
  Serial.print(" : ");
  Serial.println(sensorsBuf);
}