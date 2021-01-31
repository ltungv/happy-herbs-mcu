#ifndef HAPPY_HERBS_H_
#define HAPPY_HERBS_H_

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

class HappyHerbsState {
 private:
  int lampPinID;
  bool lampState;

 public:
  void setLampPinID(const int);
  /*
  void setPumpPinID(const int);
  void setSensorBrightness();
  void setSensorTemparature();
  void setSensorMoisture();
  void senSensorHumidity();
  */

  bool getLampState();
  void setLampState(const bool);
};

class HappyHerbsService {
 private:
  int lastUpdated = 0;
  String endpoint;
  PubSubClient *pubsub;
  HappyHerbsState *hhState;

 public:
  HappyHerbsService(PubSubClient &, HappyHerbsState &);

  void loop();
  bool connected();
  void reconnect();
  void handleCallback(char *, byte *, unsigned int);

  void handleShadowUpdateDelta(const JsonDocument &);
  void publishShadowUpdate();
};

#endif  // HAPPY_HERBS_H_