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
  float lightThreshold, moistureThreshold;
  int lampPinID;
  int pumpPinID;
  int moistureSensorPinID;
  DHT *tempHumidSensorDHT;
  BH1750 *lightSensorBH1750;

 public:
  HappyHerbsState(BH1750 &, DHT &, int, int, int, float, float);

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
  String thingName;
  String topicShadowUpdate;
  String topicShadowUpdateDelta;

  HappyHerbsState *hhState;
  PubSubClient *pubsub;
  int lastUpdated = 0;

 public:
  HappyHerbsService(HappyHerbsState &, PubSubClient &);

  void loop();
  bool connected();
  void reconnect();
  void handleCallback(char *, byte *, unsigned int);

  void setThingName(String);

  void handleShadowUpdateDelta(const JsonDocument &);
  void publishShadowUpdate();
  void publishCurrentSensorsMeasurements();
};

#endif  // HAPPY_HERBS_H_