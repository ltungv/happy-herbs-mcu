
#include "happy_herbs.h"

#include <Arduino.h>
#include <SPIFFS.h>

#include "constants.h"

bool HappyHerbsState::getLampState() {
  return digitalRead(this->lampPinID) == HIGH;
}

void HappyHerbsState::setLampState(bool lampState) {
  digitalWrite(this->lampPinID, lampState);
}

void HappyHerbsState::setLampPinID(int lampPinID) {
  this->lampPinID = lampPinID;
}

HappyHerbsService::HappyHerbsService(PubSubClient &pubsub,
                                     HappyHerbsState &hhState) {
  this->pubsub = &pubsub;
  this->hhState = &hhState;
}

void HappyHerbsService::loop() { this->pubsub->loop(); }

bool HappyHerbsService::connected() { return this->pubsub->connected(); }

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

void HappyHerbsService::handleCallback(char *topic, byte *payload,
                                       unsigned int length) {
  if (strcmp(topic, TOPIC_SHADOW_UPDATE_DELTA.c_str()) == 0) {
    StaticJsonDocument<512> shadowUpdateDeltaJson;
    deserializeJson(shadowUpdateDeltaJson, payload, length);
    this->handleShadowUpdateDelta(shadowUpdateDeltaJson);
  }
};

void HappyHerbsService::handleShadowUpdateDelta(const JsonDocument &delta) {
  int ts = delta["timestamp"];
  if (ts > this->lastUpdated) {
    bool lampState = delta["state"]["lampState"];
    this->hhState->setLampState(lampState);
    this->lastUpdated = ts;
    this->publishShadowUpdate();
  }
}

void HappyHerbsService::publishShadowUpdate() {
  StaticJsonDocument<512> shadowUpdateJson;
  JsonObject stateObj = shadowUpdateJson.createNestedObject("state");
  JsonObject reportedObj = stateObj.createNestedObject("reported");
  reportedObj["lampState"] = this->hhState->getLampState();

  char shadowUpdateBuf[512];
  serializeJson(shadowUpdateJson, shadowUpdateBuf);
  this->pubsub->publish(TOPIC_SHADOW_UPDATE.c_str(), shadowUpdateBuf);

  Serial.print("SEND [");
  Serial.print(TOPIC_SHADOW_UPDATE);
  Serial.print("]");
  Serial.print(" : ");
  Serial.println(shadowUpdateBuf);
}