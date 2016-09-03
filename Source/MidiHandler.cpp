#include <string.h>
#include "MidiHandler.h"
#include <iostream>

void midi_write(uint8_t* data, size_t len);

MidiHandler::MidiHandler(){
  // memset(midi_values, 0, NOF_PARAMETERS*sizeof(uint16_t));
}

void MidiHandler::handlePitchBend(uint8_t status, uint16_t value){
  // setParameter(PARAMETER_G, ((int16_t)value - 8192)>>1);
  std::cout << "rx pb [" << (int)status << "][" << (int)value << "]" << std::endl;
}

void MidiHandler::handleNoteOn(uint8_t status, uint8_t note, uint8_t velocity){
  std::cout << "rx noteon [" << (int)status << "][" << (int)note << "][" << (int)velocity << "]" << std::endl;
  // setButton(MIDI_NOTE_BUTTON+note, velocity<<5);
  uint8_t data[] = {status, note, velocity};
  midi_write(data, sizeof(data));
}

void MidiHandler::handleNoteOff(uint8_t status, uint8_t note, uint8_t velocity){
  std::cout << "rx noteoff [" << (int)status << "][" << (int)note << "][" << (int)velocity << "]" << std::endl;
  // setButton(MIDI_NOTE_BUTTON+note, 0);
  uint8_t data[] = {status, note, velocity};
  midi_write(data, sizeof(data));
}

void MidiHandler::handleProgramChange(uint8_t status, uint8_t pid){
  std::cout << "rx pc [" << (int)status << "][" << (int)pid << "]" << std::endl;
  uint8_t data[] = {status, pid};
  midi_write(data, sizeof(data));
}

void MidiHandler::handleControlChange(uint8_t status, uint8_t cc, uint8_t value){
  std::cout << "rx cc [" << (int)status << "][" << (int)cc << "][" << (int)value << "]" << std::endl;
  uint8_t data[] = {status, cc, value};
  midi_write(data, sizeof(data));
}

void MidiHandler::handleSysEx(uint8_t* data, uint16_t size){
  std::cout << "rx sysex [" << (int)size << "]" << std::endl;
  midi_write(data, size);
}

