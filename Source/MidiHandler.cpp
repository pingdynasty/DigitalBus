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

void MidiHandler::handleSystemRealTime(uint8_t cmd){
  std::cout << "rx system realtime [0x" << std::hex << (int)cmd << "]" << std::endl;
  midi_write(&cmd, 1);
}

void MidiHandler::handleSystemCommon(uint8_t cmd1, uint8_t cmd2){
  std::cout << "rx system common [0x" << std::hex << (int)cmd1 << " 0x" << (int)cmd2 << "]" << std::endl;
  uint8_t data[] = {cmd1, cmd2};
  midi_write(data, sizeof(data));
}

void MidiHandler::handleSystemCommon(uint8_t cmd1, uint8_t cmd2, uint8_t cmd3){
  std::cout << "rx system common [0x" << std::hex << (int)cmd1 << " 0x" << (int)cmd2 << " 0x" << (int)cmd3 << "]" << std::endl;
  uint8_t data[] = {cmd1, cmd2, cmd3};
  midi_write(data, sizeof(data));
}

void MidiHandler::handleChannelPressure(uint8_t status, uint8_t value){
  std::cout << "rx cp [" << (int)status << "][" << (int)value << "]" << std::endl;
  uint8_t data[] = {status, value};
  midi_write(data, sizeof(data));
}

void MidiHandler::handlePolyKeyPressure(uint8_t status, uint8_t note, uint8_t value){
  std::cout << "rx at [" << (int)status << "][" << (int)note << "][" << (int)value << "]" << std::endl;
  uint8_t data[] = {status, note, value};
  midi_write(data, sizeof(data));
}
