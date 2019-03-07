/*
 * BelaSerial.h
 *
 * Based on the following:
 * https://github.com/giuliomoro/serial-piano-scanner
 * https://github.com/dr-offig/BelaArduinoComms
 *
 */

#ifndef BelaSerial_H_
#define BelaSerial_H_

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#define MAX_FRAME_LENGTH 256
#define ESCAPE_CHARACTER 0xFE // Indicates control sequence

// Frame structure
enum {
  kControlCharacterFrameBegin = 0x00,
  kControlCharacterAck = 0x01,
  kControlCharacterNak = 0x02,
  kControlCharacterFrameError = 0xFD,
  kControlCharacterFrameEnd = 0xFF
};

// Frame types
enum {
  kControlType = 0
}

// 

class BelaSerial {
public:
  BelaSerial();
  ~BelaSerial();

  bool setup(const char* device, unsigned int speed);

  int serialRead(char* buf, size_t len, int timeoutMs);
  int serialWrite(const char* buf, size_t len);

private:
  int init (const char* device, unsigned int speed);
  void cleanup();

  void setBaudRate (unsigned int speed);
  void setMinCount (int fd, int mcount);
  int setInterfaceAttribs (int fd, int speed);
  
  unsigned int _speed;
  // static int _handle;

};

#endif /* BelaSerial_H_ */
