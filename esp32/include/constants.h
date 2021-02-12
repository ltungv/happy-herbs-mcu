#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

const int HH_GPIO_LAMP = LED_BUILTIN;
const int HH_GPIO_BH1750_SDA = 8;
const int HH_GPIO_BH1750_SCL = 9;
const int HH_I2C_BH1750_ADDR = 0x23;

const int SERIAL_BAUD_RATE = 115200;
const String NTP_SERVER = "pool.ntp.org";
const String MISC_CREDS = "/creds/misc.json";

const String AWS_THING_NAME = "test-plant-module";
const String AWS_IOT_ENDPOINT = "/creds/aws/iot-endpoint.txt";
const String AWS_ROOTCA_CERT = "/creds/aws/rootca-cert.crt";
const String AWS_CLIENT_CERT = "/creds/aws/device-cert.crt";
const String AWS_CLIENT_KEY = "/creds/aws/device-key.key";

const String SHADOW_TOPIC_PREFIX = "$aws/things/" + AWS_THING_NAME + "/shadow";
const String TOPIC_SHADOW_UPDATE = SHADOW_TOPIC_PREFIX + "/update";
const String TOPIC_SHADOW_UPDATE_DELTA = TOPIC_SHADOW_UPDATE + "/delta";

const String TOPIC_SENSORS_PUBLISH = "sensorsMeasurements";

#endif  // CONSTANTS_H