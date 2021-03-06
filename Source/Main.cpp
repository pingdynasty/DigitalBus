#include "../JuceLibraryCode/JuceHeader.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <strings.h>
#include <unistd.h>
#include <iostream>
#include <sys/ioctl.h>
#include <errno.h>
#include "DigitalBusReader.h"
#include "MidiStatus.h"
#include "SerialBuffer.hpp"

#define DEFAULT_SPEED              115200
#define DEFAULT_PORT               "/dev/ttyS1"
#define DEFAULT_BUFFER_SIZE        512

#define DIGITAL_BUS_RX_BUFFER_SIZE 128

bool DIGITAL_BUS_PROPAGATE_MIDI = 1;
bool DIGITAL_BUS_ENABLE_BUS = 1;

/**
 * to test with virtual serial port / pty:
 * DigitalBus -p /dev/ptyp0
 * DigitalBus -p /dev/ttyp0
 */

extern "C" {
  void serial_write(uint8_t* data, uint16_t len);
  void midi_write(uint8_t* data, uint16_t len);
}

class DigitalBusService : public juce::MidiInputCallback {
private:
  int m_fd;
  struct termios m_oldtio;
  juce::String m_port;
  int m_speed;
  bool m_verbose;
  bool m_connected;
  volatile bool m_running;
  MidiOutput* m_midiout;
  MidiInput* m_midiin;
  int bufferSize;
  DigitalBusReader bus;
  SerialBuffer<DIGITAL_BUS_RX_BUFFER_SIZE> rxbuf;

  juce::String print(const MidiMessage& msg){
    juce::String str;
    for(int i=0; i<msg.getRawDataSize(); ++i){
      str += " 0x";
      str += juce::String::toHexString(msg.getRawData()[i]);
    }
    return str;
  }

  juce::String print(const unsigned char* buf, int len){
    juce::String str;
    for(int i=0; i<len; ++i){
      str += " 0x";
      str += juce::String::toHexString(buf[i]);
    }
    return str;
  }

  void listDevices(const StringArray& names){
    for(int i=0; i<names.size(); ++i)
      std::cout << i << ": " << names[i] << std::endl;
  }

public:
  void reset(){
    rxbuf.reset();
    bus.reset();
    bus.sendReset();
  }

  void writeSerial(uint8_t* data, size_t len){
    if(m_verbose)
      std::cout << "tx uart " << m_port << " [" << (int)len << "]" << std::endl;
    if(write(m_fd, data, len) != len)
      perror(m_port.toUTF8());
  }

  void writeSerial(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3){
    uint8_t msg[4] = {d0, d1, d2, d3};
    writeSerial(msg, 4);
  }

  void writeSysex(uint8_t* data, uint16_t size){
    /* USB-MIDI devices transmit sysex messages in 4-byte packets which
     * contain a status byte and up to 3 bytes of the message itself.
     * If the message ends with fewer than 3 bytes, a different code is
     * sent. Go through the sysex 3 bytes at a time, including the leading
     * 0xF0 and trailing 0xF7.
     */
    uint8_t packet[4] = { USB_COMMAND_SYSEX };
    int count = size/3;
    uint8_t* src = data;
    while(count-- > 0){
      packet[1] = *src++;
      packet[2] = *src++;
      packet[3] = *src++;
      if(packet[3] == SYSEX_EOX)
	packet[0] = USB_COMMAND_SYSEX_EOX3;
      writeSerial(packet, 4);
    }
    count = size % 3;
    switch(count){
    case 0:
      break;
    case 1:
      packet[0] = USB_COMMAND_SYSEX_EOX1;
      packet[1] = *src++;
      packet[2] = 0;
      packet[3] = 0;
      break;
    case 2:
      packet[0] = USB_COMMAND_SYSEX_EOX2;
      packet[1] = *src++;
      packet[2] = *src++;
      packet[3] = 0;
      break;
    }
    writeSerial(packet, 4);
  }

  void handleIncomingMidiMessage(MidiInput *source,
                                 const MidiMessage &msg){
//     if(msg.isSysEx()){
//       handlePartialSysexMessage(source, msg.getRawData(), msg.getRawDataSize(), msg.getTimeStamp());
//     }else{
    if(m_verbose)
      std::cout << "rx midi: " << print(msg) << std::endl;
    if(msg.isSysEx()){
      writeSysex((uint8_t*)msg.getRawData(), msg.getRawDataSize());
    }else{
      const uint8_t* data = msg.getRawData();
      switch(msg.getRawDataSize()){
      case 3:
	writeSerial(data[0]>>4, data[0], data[1], data[2]);
	break;
      case 2:
	writeSerial(data[0]>>4, data[0], data[1], 0);
	break;
      case 1:
	writeSerial(data[0]>>4, data[0], 0, 0);
	break;
      default:
	std::cout << "YO MIDI " << msg.getRawDataSize() << std::endl;
	break;
      }
    }
  }

  // MemoryOutputStream sysexbuf;  
  // void handlePartialSysexMessage(MidiInput* source, const uint8* data,
  // 				 int size, double timestamp){
  //   if(m_verbose)
  //     std::cout << "rx midi: " << " sysex " << size << " bytes" << std::endl;
  //   sysexbuf.write(data, size);
  //   if(data[size-1] == SYSEX_EOX){
  //     writeSysex((uint8_t*)sysexbuf.getData(), sysexbuf.getDataSize());
  //     sysexbuf.reset();
  //   }
  // }

  void writeMidi(uint8_t* data, size_t len){
    MidiMessage msg(data, len);
    if(m_verbose)
      std::cout << "tx midi: " << print(msg) << std::endl;
    if(m_midiout != NULL)
      m_midiout->sendMessageNow(msg);
  }

  void usage(){
    std::cerr << "DigitalBus v1"  << std::endl << "usage:" << std::endl
              << "-p FILE\t set serial port" << std::endl
              << "-s NUM\t set serial speed (default: " << DEFAULT_SPEED << ")" << std::endl
              << "-v\t verbose, prints messages sent/received" << std::endl
              << "-m\t monitor and forward messages but don't partake (todo)" << std::endl
              << "--no-midi\t disable MIDI message forwarding" << std::endl
              << "--no-bus\t disable bus messaging" << std::endl
              << "-i NUM\t set MIDI input device" << std::endl
              << "-o NUM\t set MIDI output device" << std::endl
              << "-c NAME\t create MIDI input/output device" << std::endl
              << "-l\t list MIDI input/output devices and exit" << std::endl
              << "-h or --help\tprint this usage information and exit" << std::endl;
  }

  int connect(){
    if(m_midiin == NULL && m_midiout == NULL){
      // default behaviour if no interface specified
      m_midiout = MidiOutput::createNewDevice("DigitalBus");
      m_midiin = MidiInput::createNewDevice("DigitalBus", this);
    }
    m_fd = openSerial(m_port.toUTF8(), m_speed);
    if(m_fd < 0){
      perror(m_port.toUTF8()); 
      return -1; 
    }
    if(m_verbose)
      std::cout << "tty " << m_port << " at " << m_speed << " baud" << std::endl;
    //     fcntl(m_fd, F_SETFL, FNDELAY); // set non-blocking read
//     fcntl(m_fd, F_SETFL, 0); // set blocking read
    //     fcntl(fd, F_SETFL, O_APPEND); // append output
    //     fcntl(fd, F_NOCACHE, 1); // turn caching off
    m_connected = true;
    return 0;
  }

  int stop(){
    if(m_verbose)
      std::cout << "stopping" << std::endl;
    m_running = false;
    return 0;
  }

  int start(){
    if(m_midiin != NULL)
      m_midiin->start();
    m_running = true;
    return 0;
  }

  uint32_t getSysTicks(){
    return Time::currentTimeMillis();
  }

  uint32_t BUS_IDLE_INTERVAL = 2453;

  int bus_status(){
    static uint32_t lastpolled = 0;
    if(getSysTicks() > lastpolled + BUS_IDLE_INTERVAL){
      bus.connected();
      lastpolled = getSysTicks();
    }
    return bus.getStatus();
  }

  int run(){
    juce::MidiMessage msg;
    ssize_t len;
    int frompos;
    bus.sendReset();
    while(m_running) {
      uint8_t buf[4];
      len = read(m_fd, buf, 4);
      if(len < 0)
	return -1;
      rxbuf.push(buf, len);
      while(rxbuf.available() >= 4){
	rxbuf.pull(buf, 4);
	std::cout << "> [0x"<< std::hex << (int)buf[0] << " 0x" << (int)buf[1] << " 0x" << (int)buf[2] << " 0x" << (int)buf[3] << "]" << std::endl;
	bus.readBusFrame(buf);
      }

      bus_status();
      // uint8_t* buf = rxbuf.getWriteHead();
      // len = read(m_fd, buf, rxbuf.getWriteCapacity());
      // rxbuf.incrementWriteHead(len);
      // while(rxbuf.available() >= 4){
      // 	buf = rxbuf.getReadHead();
      // 	std::cout << "> [0x"<< std::hex << (int)buf[0] << " 0x" << (int)buf[1] << " 0x" << (int)buf[2] << " 0x" << (int)buf[3] << "]" << std::endl;
      // 	bus.readBusFrame(buf);
      // 	rxbuf.incrementReadHead(4);
      // }
    }
    return 0;
  }

  int openSerial(const char* serialport, int baud) {
    switch(baud){
    case 9600:
      baud = B9600;
      break;
    case 19200:
      baud = B19200;
      break;
    case 38400:
      baud = B38400;
      break;
    case 57600:
      baud = B57600;
      break;
    case 115200:
      baud = B115200;
      break;
    case 230400:
      baud = B230400;
      break;
#ifdef B460800
    case 460800:
      baud = B460800;
      break;
#endif
#ifdef B921600
    case 921600:
      baud = B921600;
      break;
#endif
    }
    struct termios tio;
    int fd;
    fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1){
      perror(serialport);
      return -1;
    }
    if(tcgetattr(fd, &tio) < 0){
      perror(serialport);
      return -1;
    }
    m_oldtio = tio;
    fcntl(fd, F_SETFL, 0);

    /* Configure port */
    tio.c_cflag = baud | CS8 | CLOCAL | CREAD;
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;
    if(cfsetispeed(&tio, baud) < 0)
      return -1;
    if(cfsetospeed(&tio, baud) < 0)
      return -1;
    // int actual = cfsetispeed(&tio, baud);
    // if(actual != baud)
    //   std::cerr << "Actual input speed: " << actual << " baud." << std::endl;      
    // actual = cfsetospeed(&tio, baud);
    // if(actual != baud)
    //   std::cerr << "Actual output speed: " << actual << " baud." << std::endl;
    // //     cfmakeraw(&tio);
    // if(actual < 0)
    //   return -1;
    tcflush(fd, TCIFLUSH);
    if(tcsetattr(fd, TCSANOW, &tio) < 0)
      return -1;
    return fd;
  }

  int configure(int argc, char* argv[]) {
    for(int i=1; i<argc; ++i){
      juce::String arg = juce::String(argv[i]);
      if(arg.compare("-p") == 0 && ++i < argc){
        m_port = juce::String(argv[i]);
      }else if(arg.compare("-v") == 0){
        m_verbose = true;
      }else if(arg.compare("--no-bus") == 0){
	DIGITAL_BUS_ENABLE_BUS = false;
      }else if(arg.compare("--no-midi") == 0){
        DIGITAL_BUS_PROPAGATE_MIDI = false;
      }else if(arg.compare("-l") == 0){
        std::cout << "MIDI output devices:" << std::endl;
        listDevices(MidiOutput::getDevices());
        std::cout << "MIDI input devices:" << std::endl;
        listDevices(MidiInput::getDevices());
        return 1;
      }else if(arg.compare("-s") == 0 && ++i < argc){
        m_speed = juce::String(argv[i]).getIntValue();
      }else if(arg.compare("-o") == 0 && ++i < argc && m_midiout == NULL){
        int index = juce::String(argv[i]).getIntValue();
        m_midiout = MidiOutput::openDevice(index);
        if(m_verbose)
          std::cout << "Opening MIDI output: " << MidiOutput::getDevices()[index] << std::endl;
      }else if(arg.compare("-i") == 0 && ++i < argc && m_midiin == NULL){
        int index = juce::String(argv[i]).getIntValue();
        m_midiin = MidiInput::openDevice(index, this);
        if(m_verbose)
          std::cout << "Opening MIDI input: " << MidiInput::getDevices()[index] << std::endl;
      }else if(arg.compare("-c") == 0 && ++i < argc && m_midiin == NULL && m_midiout == NULL){
        String name = juce::String(argv[i]);
        if(m_verbose)
          std::cout << "Creating MIDI input and output: " << name << std::endl;
        m_midiout = MidiOutput::createNewDevice(name);
        m_midiin = MidiInput::createNewDevice(name, this);
      }else if(arg.compare("-h") == 0 || arg.compare("--help") == 0 ){
        usage();
        return 1;
      }else{
        usage();
        errno = EINVAL;
        perror(arg.toUTF8());
        return -1;
      }
    }
    return 0;
  }

  int disconnect(){
    if(m_connected){
      if(m_verbose)
	std::cout << "disconnecting" << std::endl;
      tcsetattr(m_fd, TCSANOW, &m_oldtio);
      close(m_fd);
      m_connected = false;
    }
    if(m_midiout != NULL){
      delete m_midiout;
      m_midiout = NULL;
    }
    if(m_midiin != NULL){
      m_midiin->stop();
      delete m_midiin;
      m_midiin = NULL;
    }
    return 0;
  }

  DigitalBusService() :
    m_port(DEFAULT_PORT),
    m_speed(DEFAULT_SPEED), 
    bufferSize(DEFAULT_BUFFER_SIZE)
    // midireader(DEFAULT_BUFFER_SIZE)
  {
    m_midiin = NULL;
    m_midiout = NULL;
  }
  
  ~DigitalBusService(){
    if(m_connected)
      disconnect();
    if(m_running)
      stop();
  }
};

DigitalBusService service;

void sigfun(int sig){
  (void)signal(sig, SIG_DFL);
  service.disconnect();
  service.stop();
}

int main(int argc, char* argv[]) {
//   const ScopedJuceInitialiser_NonGUI juceSystemInitialiser;
  (void)signal(SIGINT, sigfun);
  (void)signal(SIGQUIT, sigfun);
  int ret = service.configure(argc, argv);
  if(!ret)
    ret = service.connect();
  if(!ret)
    ret = service.start();
  if(!ret)
    ret = service.run();
  ret |= service.disconnect();
  return ret;
}

void serial_write(uint8_t* data, uint16_t len){
  service.writeSerial(data, len);
}

void midi_write(uint8_t* data, uint16_t len){
  service.writeMidi(data, len);
}


void bus_setup(){}
int bus_status(){
  return 1;
}
uint8_t* bus_deviceid(){
  return (uint8_t*)"123443211234";
}

/* outgoing: send message over digital bus */
void bus_tx_parameter(uint8_t pid, int16_t value){
  // bus.sendParameterChange(pid, value);
}
void bus_tx_button(uint8_t bid, int16_t value){
  // bus.sendButtonChange(bid, value);
}
void bus_tx_command(uint8_t cmd, int16_t data){}
void bus_tx_message(const char* msg){}
void bus_tx_data(const uint8_t* data, uint16_t size){}

/* incoming: callback when message received on digital bus */
void bus_rx_parameter(uint8_t pid, int16_t value){
  std::cout << "bus rx parameter [" << (int)pid << "][" << value << "]" << std::endl;
}

void bus_rx_button(uint8_t bid, int16_t value){
  std::cout << "bus rx button [" << bid << "][" << value << "]" << std::endl;
}

void bus_rx_command(uint8_t cmd, int16_t data){
  std::cout << "bus rx command [" << (int)cmd << "][" << (int)data << "]" << std::endl;
}

void bus_rx_message(const char* msg){
  std::cout << "bus rx message [" << msg << "]" << std::endl;
}

void bus_rx_data(const uint8_t* ptr, uint16_t size){
  std::cout << "bus rx data [" << size << "]" << std::endl;
}

/* error: callback on error condition */
void bus_tx_error(const char* reason){
  std::cout << "bus tx error [" << reason << "]" << std::endl;
}

void bus_rx_error(const char* reason){
  std::cout << "bus rx error [" << reason << "]" << std::endl;
  service.reset();
}
