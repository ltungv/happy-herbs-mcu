#include "happy_herbs.h"

#include <Arduino.h>

#include "constants.h"
#include "ioutils.h"
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

bool HappyHerbsState::readLampPinID() {
  return digitalRead(this->lampPinID) == HIGH;
}

void HappyHerbsState::writeLampPinID(bool lampState) {
  digitalWrite(this->lampPinID, lampState);
}

bool HappyHerbsState::readPumpPinID() {
  return digitalRead(this->pumpPinID) == HIGH;
}

void HappyHerbsState::writePumpPinID(bool pumpState) {
  digitalWrite(this->pumpPinID, pumpState);
}

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

/**
 * Definition and usages of MQTT payload, and how to interact with the broker is
 * documented by AWS at
 * https://docs.aws.amazon.com/iot/latest/developerguide/iot-device-shadows.html
 */
HappyHerbsService::HappyHerbsService(HappyHerbsState &hhState,
                                     PubSubClient &pubsub) {
  this->hhState = &hhState;
  this->pubsub = &pubsub;
}

/**
 * Set up a task that start the plant watering routine, the pump will be turn
 * on for `wateringDuration` milliseconds when this task starts and finishes.
 * The task is disabled by default
 *
 * NOTE: The task is owned by this class so it can be used in some internal MQTT
 * messages handlers
 *
 * @param scheduler The scheduler that will run this task
 * @param wateringDuration Number of milliseconds that the pump is turned on
 */
void HappyHerbsService::setupTaskPlantWatering(Scheduler &scheduler,
                                               long wateringDuration) {
  this->taskPlantWatering.setInterval(wateringDuration);
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

/**
 * Get the task that control the plant watering routine
 *
 * @return The task for plant watering
 */
Task &HappyHerbsService::getTaskPlantWatering() {
  return this->taskPlantWatering;
}

/**
 * Set the name of this MCU designated by AWS, then setup of the MQTT topics
 * that are used to communicate with AWS
 *
 * @param thingName The assigned name
 */
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
 * Set the lamp's state using the underlying state object and send a message to
 * indicate state changes to AWS
 *
 * @param state Desired lamp's state
 */
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

/**
 * Set the pump's state using the underlying state object and send a message to
 * indicate state changes to AWS
 *
 * @param state Desired pump's state
 */
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

/**
 * Set the light threshold using the underlying state object and send a message
 * to indicate state changes to AWS
 *
 * @param threshold Desired light threshold
 */
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

/**
 * Set the moisture threshold using the underlying state object and send a
 * message to indicate state changes to AWS
 *
 * @param threshold Desired moisture threshold
 */
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
 * Try to connect to AWS IoT. If a connection is successfully initiated, the
 * system will subscribe to all the necessary MQTT topics
 *
 * @return True if a connection is made
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
 *
 * @return True if a connection is still maintained
 */
bool HappyHerbsService::connected() { return this->pubsub->connected(); }

/**
 * Publish a given payload to the given topic, this is a proxy to the underlying
 * MQTT client and provides serial logging for debug.
 *
 * NOTE: The payload size must not exceeds MQTT_MESSAGE_BUFFER_SIZE
 *
 * @param topic MQTT topic
 * @param payload Data to be sent
 * @return True if published successfully
 */
bool HappyHerbsService::publish(const char *topic, const char *payload) {
  bool isSent = this->pubsub->publish(topic, payload);
  if (isSent) {
    Serial.print("SENT [");
    Serial.print(topic);
    Serial.print("]");
    Serial.print(" : ");
    Serial.printf("%s\n\n", payload);
    ledBlink(LED_BUILTIN, 100, 100, 1);
  }
  return isSent;
}

/**
 * Serialize the JSON document and publish the serialized data to the given
 * topic
 *
 * NOTE: The payload size must not exceeds MQTT_MESSAGE_BUFFER_SIZE
 *
 * @param topic MQTT topic
 * @param doc JSON document to be sent
 * @return True if published successfully
 */
void HappyHerbsService::publishJson(const char *topic,
                                    const JsonDocument &doc) {
  char buf[MQTT_MESSAGE_BUFFER_SIZE];
  serializeJson(doc, buf);
  this->publish(topic, buf);
}

/**
 * Publish an empty message to "$aws/things/{thing_name}/shadow/get" to query
 * the shadow.
 */
void HappyHerbsService::publishShadowGet() {
  this->publish(this->topicShadowGet.c_str(), "");
}

/**
 * Publishes a message to the topic "$aws/things/{thing_name}/shadow/update" to
 * announce the client current state. Every existing state will be included in
 * the message
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

/**
 * Take measurements for every sensor and publish them to AWS, the data will be
 * stored inside a DynamoDB table with each corresponds with a table column
 */
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
  this->publishJson(TOPIC_SENSORS_MEASUREMENTS.c_str(), sensorsJson);
}

/**
 * Take measurements for every sensor and publish them to AWS, the data will be
 * stored inside a DynamoDB table with each corresponds with a table column
 */
void HappyHerbsService::publishStateSnapshot() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return;
  }
  time(&now);

  StaticJsonDocument<512> stateJson;
  stateJson["timestamp"] = now;
  stateJson["thingsName"] = this->thingName;
  JsonObject sensorsObj = stateJson.createNestedObject("sensors");
  sensorsObj["luxBH1750"] = this->hhState->readLightSensorBH1750();
  sensorsObj["moisture"] = this->hhState->readMoistureSensor();
  sensorsObj["temperature"] = this->hhState->readTemperatureSensor();
  sensorsObj["humidity"] = this->hhState->readHumiditySensor();
  JsonObject shadowObj = stateJson.createNestedObject("shadow");
  shadowObj["lampState"] = this->hhState->readLampPinID();
  shadowObj["pumpState"] = this->hhState->readPumpPinID();
  shadowObj["lightThreshold"] = this->hhState->getLightThreshold();
  shadowObj["moistureThreshold"] = this->hhState->getMoistureThreshold();
  this->publishJson(TOPIC_STATE_SNAPSHOT.c_str(), stateJson);
}

/**
 * Subscribe to the given topic with the specified QoS, this is a proxy to the
 * underlying MQTT client and provides serial logging for debug.
 *
 * @param topic MQTT topic
 * @param qos Quality of service, AWS only support level 0 and level 1
 */
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
 *
 * @param topic The topic on which the payload is published
 * @param payload The published data
 * @param length The size of the payload
 */
void HappyHerbsService::handleCallback(const char *topic, byte *payload,
                                       unsigned int length) {
  Serial.print("RECV [");
  Serial.print(topic);
  Serial.print("]");
  Serial.print(" : ");
  Serial.printf("%s\n\n", payload);
  ledBlink(LED_BUILTIN, 100, 100, 1);

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

/**
 * Handle shadow get accepted document. If the document contains delta state,
 * update the system state to match the delta
 *
 * @param acceptedDoc The shadow get accepted document
 */
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

/**
 * Handle shadow get rejected document. Resend a get message if receive error
 * code 500
 *
 * @param errorDoc The error document
 */
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

/**
 * Handle shadow update accepted document. Check if the returned "reported
 * state" matches with the system, if not, resend an update message
 *
 * @param accepted The accepted update document
 */
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

/**
 * Handle shadow update rejected document. Resend a get message if receive error
 * code 500
 *
 * @param errorDoc Error document
 */
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
 * Handle shadow update delta document. Update the system state to match the
 * delta document
 *
 * @param deltaDoc Delta document
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