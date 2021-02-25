#include "FS.h"
#include "SPIFFS.h"
#include "ioutils.h"

/**
 * Load n bytes from the stream into memory and return the pointer the the first
 * byte.
 *
 * NOTE: If error ocurred, a null pointer will be returned
 *
 * @param stream The input stream
 * @param n Number of bytes to be loaded
 * @return A char array containing n bytes that are loaded from the stream
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

/**
 * Open a file and load the entire file as bytes array into memory. Then return
 * the pointer to the first byte.
 *
 * NOTE: If error ocurred, a null pointer will be returned
 *
 * @param path The path of the file
 * @return A char array containing n bytes that are loaded from the stream
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

/**
 * Cycle the state of the given pin `n` times through HIGH for
 * `onDuration` * milliseconds and LOW for `offDuration` milliseconds
 *
 * @param pin The pin id
 * @param onDuration Number of milliseconds that the pin is put on HIGH
 * @param offDuration Number of milliseconds that the pin is put on LOW
 * @param n Number of cycles
 */
void ledBlink(int pin, int onDuration, int offDuration, int n) {
  while (n--) {
    digitalWrite(pin, HIGH);
    delay(onDuration);
    digitalWrite(pin, LOW);
    if (n - 1 != 0) {
      delay(offDuration);
    }
  }
}