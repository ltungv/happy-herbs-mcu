#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

const int HH_GPIO_LAMP = LED_BUILTIN;
const int HH_GPIO_PUMP = 0;
const int HH_GPIO_DHT = 4;
const int HH_GPIO_MOISTURE = 10;
const int HH_GPIO_BH1750_SDA = 8;
const int HH_GPIO_BH1750_SCL = 9;
const int HH_I2C_BH1750_ADDR = 0x23;

const float DEFAULT_LIGHT_THRESHOLD = 100;
const float DEFAULT_MOISTURE_THRESHOLD = 30;

const int SERIAL_BAUD_RATE = 115200;
const String NTP_SERVER = "pool.ntp.org";
const String MISC_CREDS = "/creds/misc.json";

const String AWS_THING_NAME = "/creds/aws/iot-thingname.txt";
const String AWS_IOT_ENDPOINT = "/creds/aws/iot-endpoint.txt";
const String AWS_ROOTCA_CERT = "/creds/aws/rootca-cert.pem";
const String AWS_CLIENT_CERT = "/creds/aws/device-cert.crt";
const String AWS_CLIENT_KEY = "/creds/aws/device-key.key";

const String TOPIC_SENSORS_PUBLISH = "sensorsMeasurements";

#endif  // CONSTANTS_H
