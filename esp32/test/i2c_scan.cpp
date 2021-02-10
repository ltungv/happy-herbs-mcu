#include <Arduino.h>
#include <Wire.h>
#include <unity.h>

void scanI2C() {
  Serial.println("Scanning...");
  int nDevices = 0;
  for (byte address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("!");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void setup() {
  Wire.begin(8, 9);
  Serial.begin(115200);  // The baudrate of Serial monitor is set in 9600
  while (!Serial)
    ;  // Waiting for Serial Monitor
  UNITY_BEGIN();
}

bool finished = false;
void loop() {
  if (!finished) {
    RUN_TEST(scanI2C);
    finished = true;
  } else {
    UNITY_END();
  }
}
