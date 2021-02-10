#ifndef IOUTILS_H_
#define IOUTILS_H_

/**
 * Load n bytes from the stream into a char arrary and return the pointer to the first element
 */
char* loadStream(Stream&, int);

/**
 * Load the entire content of on file on SPIFFS into a char array and return the pointer to the first element
 */
char* loadFile(const char*);

/**
 * Cycle the pin through HIGH and LOW with delays
 */
void ledBlink(int, int, int, int);

#endif  // IOUTILS_H_
