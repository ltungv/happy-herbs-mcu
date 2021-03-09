#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

const String NTP_SERVER = "pool.ntp.org";
const String MISC_CREDS = "/creds/misc.json";

const String AWS_THING_NAME = "/creds/aws/iot-thingname.txt";
const String AWS_IOT_ENDPOINT = "/creds/aws/iot-endpoint.txt";
const String AWS_ROOTCA_CERT = "/creds/aws/rootca-cert.pem";
const String AWS_CLIENT_CERT = "/creds/aws/device-cert.crt";
const String AWS_CLIENT_KEY = "/creds/aws/device-key.key";

const String TOPIC_STATE_SNAPSHOT = "stateSnapshot";
const String TOPIC_SENSORS_MEASUREMENTS = "sensorsMeasurements";

const float DEFAULT_LIGHT_THRESHOLD = 100.0;
const float DEFAULT_MOISTURE_THRESHOLD = 30.0;
const int MQTT_MESSAGE_BUFFER_SIZE = 2048;

#ifdef __HAPPY_HERBS_ESP32S2

const int SERIAL_BAUD_RATE = 115200;
const int I2C_SDA0 = 8;
const int I2C_SCL0 = 9;

const int HH_GPIO_LAMP = 21;
const int HH_GPIO_PUMP = 0;
const int HH_GPIO_DHT = 4;
const int HH_GPIO_MOISTURE = 10;
const int HH_I2C_BH1750_ADDR = 0x23;

#else

const int SERIAL_BAUD_RATE = 115200;
const int I2C_SDA0 = 8;
const int I2C_SCL0 = 9;

const int HH_GPIO_LAMP = 21;
const int HH_GPIO_PUMP = 0;
const int HH_GPIO_DHT = 4;
const int HH_GPIO_MOISTURE = 10;
const int HH_I2C_BH1750_ADDR = 0x23;

#endif

#endif  // CONSTANTS_H