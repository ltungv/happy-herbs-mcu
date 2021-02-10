#ifndef HAPPY_HERBS_H_
#define HAPPY_HERBS_H_

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

/**
 * This class manages the entire state of the planting system, any changes to
 * the singleton object of this class will be reflected in the connected
 * hardware
 */
class HappyHerbsState {
 private:
  int lampPinID;

 public:
  /*
    void setPumpPinID(const int);
    void setSensorBrightness();
    void setSensorTemparature();
    void setSensorMoisture();
    void senSensorHumidity();
  */

  /**
   * Set the pin id of the MCU that controls the lamp
   */
  void setLampPinID(const int);

  /**
   * Get the state of the pin that controls the lamp
   */
  bool getLampState();

  /**
   * Set the state of the pin that controls the lamp
   */
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

  /**
   * Calls `loop` on the underlying PubSubClient
   */
  void loop();

  /**
   * Calls `connected` on the underlying PubSubClient
   */
  bool connected();

  /**
   * Attempting to reconnect to AWS, this will block until the client is able to
   * connect to AWS' server
   */
  void reconnect();

  /**
   * Handle messages from all subscribed topics, this function will act as a
   * controller and route messages to other handlers
   */
  void handleCallback(char *, byte *, unsigned int);

  /**
   * Handle messages from the topics
   * `$aws/things/{thing_name}/shadow/update/delta`. Upon a new message, the
   * handler will set the underlying HappyHerbsState according to the change
   * that was requested by the server.
   */
  void handleShadowUpdateDelta(const JsonDocument &);

  /**
   * Publish the current underlying HappyHerbsState to the topic
   * `$aws/things/{thing_name}/shadow/update/delta`.
   */
  void publishShadowUpdate();
};

#endif  // HAPPY_HERBS_H_