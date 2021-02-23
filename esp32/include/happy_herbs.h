#ifndef HAPPY_HERBS_H_
#define HAPPY_HERBS_H_

#include <Arduino.h>
#include <ArduinoJson.h>
#include <BH1750.h>
#include <DHT.h>
#include <PubSubClient.h>

/**
 * This class manages the entire state of the planting system, any changes to
 * an object of this class will be reflected in the connected hardware
 */
class HappyHerbsState {
 private:
  float lightThreshold = 0.0;
  float moistureThreshold = 0.0;
  int lampPinID;
  int pumpPinID;
  int moistureSensorPinID;
  DHT *tempHumidSensorDHT;
  BH1750 *lightSensorBH1750;

 public:
  HappyHerbsState(BH1750 &, DHT &, int, int, int);

  void writeLampPinID(const bool);
  bool readLampPinID();
  void writePumpPinID(const bool);
  bool readPumpPinID();

  float readLightSensorBH1750();
  float readMoistureSensor();
  float readTemperatureSensor();
  float readHumiditySensor();

  void setLightThreshold(float);
  void setMoistureThreshold(float);
  float getLightThreshold();
  float getMoistureThreshold();
};

/**
 * This classes manages communications with AWS and synchronizes the local state
 * and the remote state
 */
class HappyHerbsService {
 private:
  int lastUpdated = 0;
  String thingName = "";

  String topicShadowUpdate = "";
  String topicShadowUpdateAccepted = "";
  String topicShadowUpdateDelta = "";

  String topicShadowGet = "";
  String topicShadowGetAccepted = "";

  HappyHerbsState *hhState;
  PubSubClient *pubsub;

 public:
  HappyHerbsService(HappyHerbsState &, PubSubClient &);

  void setThingName(String);

  void loop();
  bool connected();
  void reconnect();

  void handleCallback(char *, byte *, unsigned int);
  void handleShadowGetAccepted(const JsonDocument &);
  void handleShadowUpdateAccepted(const JsonDocument &);
  void handleShadowUpdateDelta(const JsonDocument &);

  void publishShadowGet();
  void publishShadowUpdate();
  void publishSensorsMeasurements();
};

#endif  // HAPPY_HERBS_H_