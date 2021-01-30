#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

const int SERIAL_BAUD_RATE = 115200;

const String AWS_THING_NAME = "test-plant-module";

const String WIFI_CREDS = "/creds/wifi.json";

const String AWS_IOT_ENDPOINT = "/creds/aws/iot-endpoint.txt";

const String AWS_ROOTCA_CERT = "/creds/aws/rootca-cert.crt";

const String AWS_DEVICE_CERT = "/creds/aws/device-cert.crt";

const String AWS_DEVICE_KEY = "/creds/aws/device-key.key";

const String TOPIC_SUB_SHADOW_GET =
    "$aws/things/" + AWS_THING_NAME + "/shadow/get/accepted";

const String TOPIC_PUB_SHADOW_GET =
    "$aws/things/" + AWS_THING_NAME + "/shadow/get";

const String TOPIC_PUB_SHADOW_UPDATE =
    "$aws/things/" + AWS_THING_NAME + "/shadow/update";

#endif  // CONSTANTS_H