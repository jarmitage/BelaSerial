#include <Bela.h>
#include "BelaSerial.h"

#define SERIAL_BUFFER_SIZE 1024
static char belaSerialBuffer[SERIAL_BUFFER_SIZE];

AuxiliaryTask serialCommsTask;
BelaSerial belaSerial;
void serial_comms_background (void*);
void serial_comms();

bool setup(BelaContext *context, void *userData) {

  belaSerial.setup ("/dev/ttyGS0", 115200);

  if ((serialCommsTask = Bela_createAuxiliaryTask (&serial_comms_background, 80, "log-writing")) == 0) return false;

  return true;

}

void serial_comms_background (void*) { serial_comms(); }

void serial_comms() {
  
  int ret = belaSerial.serialRead (belaSerialBuffer, SERIAL_BUFFER_SIZE, -1);

  if (ret > 0) {
    if(ESCAPE_CHARACTER == belaSerialBuffer[0]) {
      
    } else {
      rt_printf("Received raw byte: %d\n", serialBuffer[0]);
    }
  }

}

void render(BelaContext *context, void *userData) {
  
  Bela_scheduleAuxiliaryTask (serialCommsTask);

}

void cleanup(BelaContext *context, void *userData) {}
