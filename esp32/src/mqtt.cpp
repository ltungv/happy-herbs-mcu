#include "mqtt.h"

#include <SPIFFS.h>

#include "constants.h"

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  Serial.print("[");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void mqttReconnect(PubSubClient &mqtt) {
  // Loop until we're reconnected
  while (!mqtt.connected()) {
    Serial.println("Connecting to AWS IoT...");
    if (mqtt.connect(AWS_THING_NAME.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

bool mqttSetupClientSecure(PubSubClient &mqtt, WiFiClientSecure &wifiSecure,
                              const char *awsEndpointPath) {
  // AWS endpoint from file
  File awsEndpointFile = SPIFFS.open(awsEndpointPath);
  if (!awsEndpointFile) {
    Serial.println("Could not read wifi credentials!");
    return false;
  }
  String awsEndpoint = awsEndpointFile.readString();
  awsEndpointFile.close();
  Serial.println("======== AWS IoT ENDPOINT ========");
  Serial.println(awsEndpoint);

  // Setup mqtt client
  mqtt.setClient(wifiSecure);
  mqtt.setServer(awsEndpoint.c_str(), 8883);
  mqtt.setCallback(mqttCallback);
  return true;
}
