#include "FS.h"
#include "SPIFFS.h"
#include "ioutils.h"

/*
 * Load n bytes from the stream into memory and return the pointer the the first
 * byte.
 *
 * WILL RETURN A NULL POINTER IF ERROR OCCUR.
 */
char* loadStream(Stream& stream, int n) {
  char* dest = (char*)malloc(n + 1);
  if (!dest) {
    return nullptr;
  }
  if (n != stream.readBytes(dest, n)) {
    free(dest);
    dest = nullptr;
    return nullptr;
  }
  dest[n] = '\0';
  return dest;
}

/*
 * Open a file and load the entire file as bytes array into memory. Then return
 * the pointer to the first byte.
 *
 * WILL RETURN A NULL POINTER IF ERROR OCCUR.
 */
char* loadFile(const char* path) {
  File f = SPIFFS.open(path);
  if (!f) {
    return nullptr;
  }

  char* data = loadStream(f, f.size());
  f.close();
  return data;
}

void ledBlink(int pin, int millisOn, int millisOff, int n) {
  while (n--) {
    digitalWrite(pin, HIGH);
    delay(millisOn);
    digitalWrite(pin, LOW);
    delay(millisOff);
  }
}