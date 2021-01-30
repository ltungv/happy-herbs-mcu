#ifndef MQTT_H
#define MQTT_H

#include <PubSubClient.h>
#include <WiFiClientSecure.h>

void mqttCallback(char *, byte *, unsigned int);

void mqttReconnect(PubSubClient &);

bool mqttSetupClientSecure(PubSubClient &, WiFiClientSecure &, const char *);

#endif  // MQTT_H