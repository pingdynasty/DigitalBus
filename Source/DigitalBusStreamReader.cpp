#include "DigitalBusStreamReader.h"

DigitalBusStreamReader::DigitalBusStreamReader(){
}

void DigitalBusStreamReader::process(){
  while(rxbuf.available() >= 4){
    uint8_t frame[4];
    rxbuf.pull(frame, 4);
    if(data[0] == OWL_COMMAND_RESET)
      rxbuf.clearUntilLast(OWL_COMMAND_RESET);
    else
      readBusFrame(frame);
  }
}

void DigitalBusStreamReader::reset(){
}
