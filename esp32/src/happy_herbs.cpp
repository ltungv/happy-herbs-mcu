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
                                     Scheduler &scheduler) {
  this->hhState = &hhState;
  this->pubsub = &pubsub;

  this->taskPlantWatering.setInterval(5 * TASK_SECOND);
  this->taskPlantWatering.setIterations(TASK_ONCE);
  this->taskPlantWatering.setOnEnable([&]() {
    Serial.println("START WATERING");
    this->writePumpPinID(true);
    return true;
  });
  this->taskPlantWatering.setOnDisable([&]() {
    Serial.println("STOP WATERING");
    this->writePumpPinID(false);
  });

  scheduler.addTask(this->taskPlantWatering);
}

Task &HappyHerbsService::getTaskPlantWatering() {
  return this->taskPlantWatering;
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

void HappyHerbsService::writeLampPinID(bool state) {
  this->hhState->writeLampPinID(state);
  StaticJsonDocument<256> shadowUpdateJson;
  JsonObject stateObj = shadowUpdateJson.createNestedObject("state");
  JsonObject reportedObj = stateObj.createNestedObject("reported");
  reportedObj["lampState"] = state;
  JsonObject desiredObj = stateObj.createNestedObject("desired");
  desiredObj["lampState"] = state;
  this->publishJson(this->topicShadowUpdate.c_str(), shadowUpdateJson);
}

void HappyHerbsService::writePumpPinID(bool state) {
  this->hhState->writePumpPinID(state);
  StaticJsonDocument<256> shadowUpdateJson;
  JsonObject stateObj = shadowUpdateJson.createNestedObject("state");
  JsonObject reportedObj = stateObj.createNestedObject("reported");
  reportedObj["pumpState"] = state;
  JsonObject desiredObj = stateObj.createNestedObject("desired");
  desiredObj["pumpState"] = state;
  this->publishJson(this->topicShadowUpdate.c_str(), shadowUpdateJson);
}

void HappyHerbsService::setLightThreshold(float threshold) {
  this->hhState->setLightThreshold(threshold);
  StaticJsonDocument<256> shadowUpdateJson;
  JsonObject stateObj = shadowUpdateJson.createNestedObject("state");
  JsonObject reportedObj = stateObj.createNestedObject("reported");
  reportedObj["lightThreshold"] = threshold;
  JsonObject desiredObj = stateObj.createNestedObject("desired");
  desiredObj["lightThreshold"] = threshold;
  this->publishJson(this->topicShadowUpdate.c_str(), shadowUpdateJson);
}

void HappyHerbsService::setMoistureThreshold(float threshold) {
  this->hhState->setMoistureThreshold(threshold);
  StaticJsonDocument<256> shadowUpdateJson;
  JsonObject stateObj = shadowUpdateJson.createNestedObject("state");
  JsonObject reportedObj = stateObj.createNestedObject("reported");
  reportedObj["moistureThreshold"] = threshold;
  JsonObject desiredObj = stateObj.createNestedObject("desired");
  desiredObj["moistureThreshold"] = threshold;
  this->publishJson(this->topicShadowUpdate.c_str(), shadowUpdateJson);
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
    Serial.printf("%s\n\n", payload);
  }
  return isSent;
}

void HappyHerbsService::publishJson(const char *topic,
                                    const JsonDocument &doc) {
  char buf[MQTT_MESSAGE_BUFFER_SIZE];
  serializeJson(doc, buf);
  this->publish(topic, buf);
}

void HappyHerbsService::publishShadowGet() {
  this->publish(this->topicShadowGet.c_str(), "");
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
  this->publishJson(this->topicShadowUpdate.c_str(), shadowUpdateJson);
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
  this->publishJson(TOPIC_SENSORS_PUBLISH.c_str(), sensorsJson);
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
  Serial.printf("%s\n\n", payload);

  StaticJsonDocument<MQTT_MESSAGE_BUFFER_SIZE> jsonDoc;

  if (strcmp(topic, this->topicShadowGetAccepted.c_str()) == 0) {
    deserializeJson(jsonDoc, payload, length);
    this->handleShadowGetAccepted(jsonDoc);
  }
  if (strcmp(topic, this->topicShadowGetRejected.c_str()) == 0) {
    deserializeJson(jsonDoc, payload, length);
    this->handleShadowGetRejected(jsonDoc);
  }
  if (strcmp(topic, this->topicShadowUpdateAccepted.c_str()) == 0) {
    deserializeJson(jsonDoc, payload, length);
    this->handleShadowUpdateAccepted(jsonDoc);
  }
  if (strcmp(topic, this->topicShadowUpdateRejected.c_str()) == 0) {
    deserializeJson(jsonDoc, payload, length);
    this->handleShadowUpdateRejected(jsonDoc);
  }
  if (strcmp(topic, this->topicShadowUpdateDelta.c_str()) == 0) {
    deserializeJson(jsonDoc, payload, length);
    this->handleShadowUpdateDelta(jsonDoc);
  }
};

void HappyHerbsService::handleShadowGetAccepted(
    const JsonDocument &acceptedDoc) {
  int ts = acceptedDoc["timestamp"];
  if (ts < this->tsShadowGetResponse) {
    return;
  }
  this->tsShadowGetResponse = ts;

  if (!acceptedDoc["state"].containsKey("delta")) {
    return;
  }

  if (acceptedDoc["state"]["delta"].containsKey("lampState")) {
    int tsLampState =
        acceptedDoc["metadata"]["delta"]["lampState"]["timestamp"];
    if (tsLampState > this->tsLampState) {
      bool lampState = acceptedDoc["state"]["delta"]["lampState"];
      this->writeLampPinID(lampState);
      this->tsLampState = tsLampState;
    }
  }
  if (acceptedDoc["state"]["delta"].containsKey("pumpState")) {
    int tsPumpState =
        acceptedDoc["metadata"]["delta"]["pumpState"]["timestamp"];
    if (tsPumpState > this->tsPumpState) {
      bool pumpState = acceptedDoc["state"]["delta"]["pumpState"];
      this->writePumpPinID(pumpState);
      this->tsPumpState = tsPumpState;
    }
  }
  if (acceptedDoc["state"]["delta"].containsKey("lightThreshold")) {
    int tsLightThreshold =
        acceptedDoc["metadata"]["delta"]["lightThreshold"]["timestamp"];
    if (tsLightThreshold > this->tsLightThreshold) {
      float lightThreshold = acceptedDoc["state"]["delta"]["lightThreshold"];
      this->setLightThreshold(lightThreshold);
      this->tsLightThreshold = tsLightThreshold;
    }
  }

  if (acceptedDoc["state"].containsKey("moistureThreshold")) {
    int tsMoistureThreshold =
        acceptedDoc["metadata"]["delta"]["moistureThreshold"]["timestamp"];
    if (tsMoistureThreshold > this->tsMoistureThreshold) {
      float moistureThreshold =
          acceptedDoc["state"]["delta"]["moistureThreshold"];
      this->setMoistureThreshold(moistureThreshold);
      this->tsMoistureThreshold = tsMoistureThreshold;
    }
  }
}

void HappyHerbsService::handleShadowGetRejected(const JsonDocument &errorDoc) {
  int ts = errorDoc["timestamp"];
  if (ts < this->tsShadowGetResponse) {
    return;
  }
  this->tsShadowGetResponse = ts;

  int errCode = errorDoc["code"];
  String errMsg = errorDoc["message"];
  if (errCode == 500) {
    this->publishShadowGet();
  }
  Serial.printf("ERR%d : %s", errCode, errMsg);
}

void HappyHerbsService::handleShadowUpdateAccepted(
    const JsonDocument &acceptedDoc) {
  int ts = acceptedDoc["timestamp"];
  if (ts < this->tsShadowUpdateResponse) {
    return;
  }
  this->tsShadowUpdateResponse = ts;

  bool isMismatched = false;
  if (!acceptedDoc["state"].containsKey("reported")) {
    return;
  }
  if (acceptedDoc["state"]["reported"].containsKey("lampState")) {
    bool lampState = acceptedDoc["state"]["reported"]["lampState"];
    isMismatched = lampState != this->hhState->readLampPinID();
  }
  if (acceptedDoc["state"]["reported"].containsKey("lampState")) {
    bool pumpState = acceptedDoc["state"]["reported"]["pumpState"];
    isMismatched = pumpState != this->hhState->readPumpPinID();
  }
  if (acceptedDoc["state"]["reported"].containsKey("lampState")) {
    float lightThreshold = acceptedDoc["state"]["reported"]["lightThreshold"];
    isMismatched = lightThreshold != this->hhState->getLightThreshold();
  }
  if (acceptedDoc["state"]["reported"].containsKey("lampState")) {
    float moistureThreshold =
        acceptedDoc["state"]["reported"]["moistureThreshold"];
    isMismatched = moistureThreshold != this->hhState->getMoistureThreshold();
  }

  if (isMismatched) {
    this->publishShadowUpdate();
  }
}

void HappyHerbsService::handleShadowUpdateRejected(
    const JsonDocument &errorDoc) {
  int ts = errorDoc["timestamp"];
  if (ts < this->tsShadowUpdateResponse) {
    return;
  }
  this->tsShadowUpdateResponse = ts;

  int errCode = errorDoc["code"];
  String errMsg = errorDoc["message"];
  if (errCode == 500) {
    this->publishShadowUpdate();
  }
  Serial.printf("ERR%d : %s", errCode, errMsg);
}

/**
 * Handle messages from the topic
 * "$aws/things/{thing_name}/shadow/update/delta". Upon a new message, the
 * client will update it state as given in the message
 */
void HappyHerbsService::handleShadowUpdateDelta(const JsonDocument &deltaDoc) {
  int ts = deltaDoc["timestamp"];
  if (ts < this->tsShadowUpdateDelta) {
    return;
  }
  this->tsShadowUpdateDelta = ts;

  if (deltaDoc["state"].containsKey("lampState")) {
    int tsLampState = deltaDoc["metadata"]["lampState"]["timestamp"];
    if (tsLampState > this->tsLampState) {
      bool lampState = deltaDoc["state"]["lampState"];
      this->writeLampPinID(lampState);
      this->tsLampState = tsLampState;
    }
  }

  if (deltaDoc["state"].containsKey("pumpState")) {
    int tsPumpState = deltaDoc["metadata"]["pumpState"]["timestamp"];
    if (tsPumpState > this->tsPumpState) {
      bool pumpState = deltaDoc["state"]["pumpState"];
      if (pumpState) {
        this->taskPlantWatering.restartDelayed();
      } else {
        this->writePumpPinID(false);
      }
      this->tsPumpState = tsPumpState;
    }
  }

  if (deltaDoc["state"].containsKey("lightThreshold")) {
    int tsLightThreshold = deltaDoc["metadata"]["lightThreshold"]["timestamp"];
    if (tsLightThreshold > this->tsLightThreshold) {
      float lightThreshold = deltaDoc["state"]["lightThreshold"];
      this->setLightThreshold(lightThreshold);
      this->tsLightThreshold = tsLightThreshold;
    }
  }

  if (deltaDoc["state"].containsKey("moistureThreshold")) {
    int tsMoistureThreshold =
        deltaDoc["metadata"]["moistureThreshold"]["timestamp"];
    if (tsMoistureThreshold > this->tsMoistureThreshold) {
      float moistureThreshold = deltaDoc["state"]["moistureThreshold"];
      this->setMoistureThreshold(moistureThreshold);
      this->tsMoistureThreshold = tsMoistureThreshold;
    }
  }
}