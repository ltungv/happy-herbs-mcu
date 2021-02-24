#ifndef HAPPY_HERBS_H_
#define HAPPY_HERBS_H_

#include <Arduino.h>
#include <ArduinoJson.h>
#include <BH1750.h>
#include <DHT.h>
#include <PubSubClient.h>

#include <optional>

#define _TASK_STD_FUNCTION
#include <TaskSchedulerDeclarations.h>

class IHappyHerbsStateController {
  virtual void writeLampPinID(bool) = 0;
  virtual void writePumpPinID(bool) = 0;
  virtual void setLightThreshold(float) = 0;
  virtual void setMoistureThreshold(float) = 0;
};

/**
 * This class manages the entire state of the planting system, any changes to
 * an object of this class will be reflected in the connected hardware
 */
class HappyHerbsState : public IHappyHerbsStateController {
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

  float readLightSensorBH1750();
  float readMoistureSensor();
  float readTemperatureSensor();
  float readHumiditySensor();

  void writeLampPinID(bool) override;
  void writePumpPinID(bool) override;
  bool readLampPinID();
  bool readPumpPinID();

  void setLightThreshold(float) override;
  void setMoistureThreshold(float) override;
  float getLightThreshold();
  float getMoistureThreshold();
};

/**
 * This classes manages communications with AWS and synchronizes the local state
 * and the remote state
 */
class HappyHerbsService : IHappyHerbsStateController {
 private:
  HappyHerbsState *hhState;
  PubSubClient *pubsub;
  String thingName = "";

  String topicShadowGet = "";
  String topicShadowGetAccepted = "";
  String topicShadowGetRejected = "";

  String topicShadowUpdate = "";
  String topicShadowUpdateAccepted = "";
  String topicShadowUpdateRejected = "";
  String topicShadowUpdateDelta = "";

  Task taskPlantWatering;
  Task taskPublishShadowUpdate;
  Task taskPublishSensorsMeasurements;

  int tsLampState = 0;
  int tsPumpState = 0;
  int tsLightThreshold = 0;
  int tsMoistureThreshold = 0;

 public:
  HappyHerbsService(HappyHerbsState &, PubSubClient &, Scheduler &);
  void setThingName(String);

  void writeLampPinID(bool) override;
  void writePumpPinID(bool) override;
  void setLightThreshold(float) override;
  void setMoistureThreshold(float) override;

  void loop();
  bool connect();
  bool connected();
  bool publish(const char *, const char *);
  void publishJson(const char *, const JsonDocument &);
  bool subscribe(const char *, unsigned int = 0);
  void handleCallback(const char *, byte *, unsigned int);

  void publishShadowGet();
  void publishShadowUpdate();
  void publishSensorsMeasurements();

  void handleShadowGetAccepted(const JsonDocument &);
  void handleShadowUpdateAccepted(const JsonDocument &);
  void handleShadowUpdateRejected(const JsonDocument &);
  void handleShadowUpdateDelta(const JsonDocument &);

  Task &getTaskPlantWatering();
  Task &getTaskPublishShadowUpdate();
  Task &getTaskPublishSensorsMeasurements();
};

#endif  // HAPPY_HERBS_H_