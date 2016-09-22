#include "MidiReader.h"
#include "bus.h"
#include <iostream>

void MidiReader::readMidiFrame(uint8_t* frame){
  // apparently no running status in USB MIDI frames
  switch(frame[0]){
  case USB_COMMAND_MISC:
  case USB_COMMAND_CABLE_EVENT:
    // ignore
    std::cout << "rx misc" << std::endl;
    break;
  case USB_COMMAND_SINGLE_BYTE:
    handleSystemCommon(frame[1]);
    break;
  case USB_COMMAND_2BYTE_SYSTEM_COMMON:
    handleSystemCommon(frame[1], frame[2]);
    break;
  case USB_COMMAND_3BYTE_SYSTEM_COMMON:
    handleSystemCommon(frame[1], frame[2], frame[3]);
    break;
  case USB_COMMAND_SYSEX:
    if(pos+3 > size){
      bus_rx_error("SysEx buffer overflow");
    }else{
      buffer[pos++] = frame[1];
      buffer[pos++] = frame[2];
      buffer[pos++] = frame[3];
    }
    if(buffer[0] != SYSEX)
      bus_rx_error("Invalid SysEx");
    break;
  case USB_COMMAND_SYSEX_EOX1:
    if(pos < 3 || buffer[0] != SYSEX || frame[1] != SYSEX_EOX){
      bus_rx_error("Invalid SysEx");
    }else if(pos < size){
      buffer[pos++] = frame[1];
      handleSysEx(buffer, pos);
    }else{
      bus_rx_error("SysEx buffer overflow");
    }
    pos = 0;
    break;
  case USB_COMMAND_SYSEX_EOX2:
    if(pos < 3 || buffer[0] != SYSEX || frame[2] != SYSEX_EOX){
      bus_rx_error("Invalid SysEx");
    }else if(pos+2 > size){
      bus_rx_error("SysEx buffer overflow");
    }else{
      buffer[pos++] = frame[1];
      buffer[pos++] = frame[2];
      handleSysEx(buffer, pos);
    }
    pos = 0;
    break;
  case USB_COMMAND_SYSEX_EOX3:
    if(pos < 3 || buffer[0] != SYSEX || frame[3] != SYSEX_EOX){
      bus_rx_error("Invalid SysEx");
    }else if(pos+3 > size){
      bus_rx_error("SysEx buffer overflow");
    }else{
      buffer[pos++] = frame[1];
      buffer[pos++] = frame[2];
      buffer[pos++] = frame[3];
      handleSysEx(buffer, pos);
    }
    pos = 0;
    break;
  case USB_COMMAND_PROGRAM_CHANGE:
    handleProgramChange(frame[1], frame[2]);
    break;
  case USB_COMMAND_CHANNEL_PRESSURE:
    handleChannelPressure(frame[1], frame[2]);
    break;
  case USB_COMMAND_NOTE_OFF:
    handleNoteOff(frame[1], frame[2], frame[3]);
    break;
  case USB_COMMAND_NOTE_ON:
    if(frame[3] == 0)
      handleNoteOff(frame[1], frame[2], frame[3]);
    else
      handleNoteOn(frame[1], frame[2], frame[3]);
    break;
  case USB_COMMAND_POLY_KEY_PRESSURE:
    handlePolyKeyPressure(frame[1], frame[2], frame[3]);
    break;
  case USB_COMMAND_CONTROL_CHANGE:
    handleControlChange(frame[1], frame[2], frame[3]);
    break;
  case USB_COMMAND_PITCH_BEND_CHANGE:
    handlePitchBend(frame[1], frame[2] | (frame[3]<<7));
    break;
  default:
    bus_rx_error("Invalid USB MIDI message");
    break;
  }
}

void MidiReader::reset(){
  pos = 0;
}
