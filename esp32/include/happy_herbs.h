#ifndef HAPPY_HERBS_H_
#define HAPPY_HERBS_H_

#include <Arduino.h>
#include <ArduinoJson.h>
#include <BH1750.h>
#include <PubSubClient.h>

/**
 * This class manages the entire state of the planting system, any changes to
 * an object of this class will be reflected in the connected hardware
 */
class HappyHerbsState {
 private:
  int lampPinID;
  int pumpPinID;
  BH1750 *lightSensorBH1750;

 public:
  HappyHerbsState(BH1750 &, int, int);

  void writeLampPinID(const bool);
  bool readLampPinID();
  void writePumpPinID(const bool);
  bool readPumpPinID();

  float readLightSensorBH1750();
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

  int lastUpdated = 0;
  PubSubClient *pubsub;
  HappyHerbsState *hhState;

 public:
  HappyHerbsService(String &, PubSubClient &, HappyHerbsState &);

  void loop();
  bool connected();
  void reconnect();
  void handleCallback(char *, byte *, unsigned int);

  void handleShadowUpdateDelta(const JsonDocument &);
  void publishShadowUpdate();
  void publishCurrentSensorsMeasurements();
};

#endif  // HAPPY_HERBS_H_