#include "wifi.h"

#include <ArduinoJson.h>
#include <SPIFFS.h>

bool wifiSetupClientSecure(WiFiClientSecure &wifiSecure,
                              const char *awsRootCACertPath,
                              const char *awsCertPath, const char *awsKeyPath) {
  // Root CA's certificate
  File awsRootCACertFile = SPIFFS.open(awsRootCACertPath);
  if (!awsRootCACertFile) {
    Serial.println("Could not read root CA certificate!");
    return false;
  }
  wifiSecure.loadCACert(awsRootCACertFile, awsRootCACertFile.size());
  awsRootCACertFile.close();

  // Client's certificate
  File awsCertFile = SPIFFS.open(awsCertPath);
  if (!awsCertFile) {
    Serial.println("Could not read client certificate!");
    return false;
  }
  wifiSecure.loadCertificate(awsCertFile, awsCertFile.size());
  awsCertFile.close();

  // Client's private key
  File awsKeyFile = SPIFFS.open(awsKeyPath);
  if (!awsKeyFile) {
    Serial.println("Could not read client key!");
    return false;
  }
  wifiSecure.loadPrivateKey(awsKeyFile, awsKeyFile.size());
  awsKeyFile.close();

  return true;
}

bool wifiConnect(const char *fpath) {
  File wifiCredsFile = SPIFFS.open(fpath);
  if (!wifiCredsFile) {
    Serial.println("Could not read wifi credentials!");
    return false;
  }
  String wifiCredsContent = wifiCredsFile.readString();
  wifiCredsFile.close();

  StaticJsonDocument<512> wifiCredsJson;
  deserializeJson(wifiCredsJson, wifiCredsContent);

  String ssid = wifiCredsJson["ssid"];
  String password = wifiCredsJson["password"];

  Serial.println("Connecting to wifi");
  WiFi.begin(ssid.c_str(), password.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  return true;
}