#ifndef _MIDIREADER_H_
#define _MIDIREADER_H_

#include <inttypes.h>
#include "../JuceLibraryCode/JuceHeader.h"

#include "MidiStatus.h"

enum MidiReaderStatus {
  READY, INCOMPLETE, ERROR
};

class MidiReader {
private:
  unsigned char* buffer;
  MidiReaderStatus status; // state, should be status: READY / INCOMPLETE / ERROR
  unsigned char runningStatus;
  int size;
  int pos;
  MidiMessage msg;
public:
  MidiReader(unsigned int sz) : status(READY), runningStatus(0), size(sz), pos(0) {
    buffer = new unsigned char[size];
  }

  ~MidiReader(){
    delete buffer;
  }

  void clear(){
    runningStatus = buffer[0];
    pos = 0;
    status = READY;
  }

  unsigned char* getBuffer(int& length){
    length = pos;
    return buffer;
  }

  MidiMessage& getMessage(){
    return msg;
  }

  MidiReaderStatus read(unsigned char data){
    if(status == READY){
      clear(); // discard previous message
    }else if(pos > size){
      status = ERROR;
      // todo: throw exception
      return status;
    }
    buffer[pos++] = data;
    switch(buffer[0] & MIDI_STATUS_MASK){
    case PROGRAM_CHANGE:
    case CHANNEL_PRESSURE:
      // two byte messages
      if(pos == 2){
	status = READY;
	msg = MidiMessage(buffer[0], buffer[1]);
      }else{
	status = INCOMPLETE;
      }
      break;
    case NOTE_OFF:
    case NOTE_ON:
    case POLY_KEY_PRESSURE:
    case CONTROL_CHANGE:
    case PITCH_BEND_CHANGE:
      // three byte messages
      if(pos == 3){
	status = READY;
	msg = MidiMessage(buffer[0], buffer[1], buffer[2]);
      }else{
	status = INCOMPLETE;
      }
      break;
    case SYSTEM_COMMON:
      switch(buffer[0]){
      case TIME_CODE_QUARTER_FRAME:
      case RESERVED_F4:
      case RESERVED_F9:
      case TUNE_REQUEST:
      case TIMING_CLOCK:
      case START:
      case CONTINUE:
      case STOP:
      case RESERVED_FD:
      case ACTIVE_SENSING:
      case SYSTEM_RESET:
	// one byte messages
	status = READY;
	msg = MidiMessage(buffer[0]);
	break;
      case SYSEX:
	if(data == SYSEX_EOX && pos > 1){
	  status = READY;
	  msg = MidiMessage::createSysExMessage(buffer+1, pos-2);
	}else if(data >= STATUS_BYTE && pos > 1){
	  // SysEx message terminated by a status byte different from 0xf7
	  buffer[pos-1] = SYSEX_EOX;
	  status = READY;
	  msg = MidiMessage::createSysExMessage(buffer+1, pos-2);
	  buffer[0] = data; // save status byte for next message - will be saved as running status
	}else{
	  status = INCOMPLETE;
	}
	break;
      case SYSEX_EOX: // receiving SYSEX_EOX on its own is really an error
      default:
	status = ERROR;
	break;
      }
      break;
    default:
      if(pos == 1 && data < STATUS_BYTE && runningStatus >= STATUS_BYTE){
	// MIDI running status: this message is missing the status byte, re-use previous status
	buffer[pos++] = data;
	buffer[0] = runningStatus;
      }else{
	status = ERROR;
      }
    }
    return status;
  }

};

//   int readMidiMessage(juce::MidiMessage& msg, unsigned char lastbyte, unsigned char* buf, ssize_t len){
//     switch(buf[0]){
//     case NOTE_OFF:
//     case NOTE_ON:
//     case POLY_KEY_PRESSURE:
//     case CONTROL_CHANGE:
//     case PITCH_BEND_CHANGE:
//       midi_port_state->parse_mode = FIRST_OF_TWO_BYTES;	// expecting 2 MIDI Data bytes
//       break;
//     case PROGRAM_CHANGE:
//     case CHANNEL_PRESSURE:
//       midi_port_state->parse_mode = LAST_OF_ONE_BYTE;		// expecting 1 MIDI Data byte
//       break;
//     }
//     int used;
//     msg = juce::MidiMessage(buf, len, used, lastbyte);
//     return used;
//   }


// #define STANDBY  0
// #define NOTE_ON  1
// #define NOTE_OFF 2
// #define CONTROL_CHANGE 3
// #define PITCH_BEND 4
// #define CHANNEL_PRESSURE 5
// class MidiReader {
// public:
//   void init(MidiInterface* _midi){
//     midi = _midi;
//     reset();
//   }
//   void read(char incomingByte){
//     switch(state){
//     case NOTE_ON:
//       if(data1 == -1){
//         data1 = incomingByte;
//       }else if(data2 == -1){
//         data2 = incomingByte;
//         midi->noteOn(data1, data2);
//         reset();
//       }else{
//         reset();
//       }
//       break;
//     case NOTE_OFF:
//       if(data1 == -1){
//         data1 = incomingByte;
//       }else if(data2 == -1){
//         data2 = incomingByte;
//         midi->noteOff(data1, data2);
//         reset();
//       }else{
//         reset();
//       }
//       break;
//     case CONTROL_CHANGE:
//       if(data1 == -1){
//         data1 = incomingByte;
//       }else if(data2 == -1){
//         data2 = incomingByte;
//         midi->controlChange(data1, data2);
//         if(data1 == 0x7b)
//           midi->allNotesOff();
//         reset();
//       }else{
//         reset();
//       }
//       break;
//     case PITCH_BEND:
//       if(data1 == -1){
//         data1 = incomingByte;
//       }else if(data2 == -1){
//         data2 = incomingByte;
//         midi->pitchBend((data2 << 7) | data1);
//         reset();
//       }else{
//         reset();
//       }
//       break;
//     case CHANNEL_PRESSURE:
//       if(data1 == -1){
//         data1 = incomingByte;
//         midi->channelPressure(data1);
//         reset();
//       }else{
//         reset();
//       }
//       break;
//     case STANDBY:
//     default:
//       switch(incomingByte){
//       case 0xf8:
//         midi->midiClock();
//         break;
//       case 0xf9:
//         // A MIDI Tick message is sent at regular intervals of one message every 10 milliseconds. 
//         midi->midiTick();
//         break;
//       case 0xfa:
//         midi->startSong();
//         break;
//       case 0xfb:
//         midi->continueSong();
//         break;
//       case 0xfc:
//         midi->stopSong();
//         break;
//       default:
//         switch(incomingByte & 0xf0){
//         case 0x80:
//           state = NOTE_OFF;
//           break;
//         case 0x90:
//           state = NOTE_ON;
//           break;
//         case 0xb0:
//           state = CONTROL_CHANGE;
//           break;
//         case 0xd0:
//           state = CHANNEL_PRESSURE;
//           break;
//         case 0xe0:
//           state = PITCH_BEND;
//           break;
//         }
//       }
//     }
//   }
// private:
//   int8_t state, data1, data2;
//   MidiInterface* midi;

//   void reset(){
//     data1 = -1;
//     data2 = -1;
//     state = STANDBY;
//   }
// };

#endif /* _MIDIREADER_H_ */
