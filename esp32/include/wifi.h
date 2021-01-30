#ifndef WIFI_H
#define WIFI_H

#include <WiFiClientSecure.h>

bool wifiSetupClientSecure(WiFiClientSecure &, const char *, const char *,
                              const char *);

bool wifiConnect(const char *fpath);

#endif  // WIFI_H