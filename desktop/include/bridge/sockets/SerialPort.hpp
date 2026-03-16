#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <string>

#ifdef _WIN32
// prevent windows.h from including the old winsock.h
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

// make sure compiler doesnt fight the winsock or something
#include <windows.h>
#include <winsock2.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
typedef int HANDLE;
#define INVALID_HANDLE_VALUE -1
#endif

class SerialPort {
 public:
  // portName: "COM3" on Windows or "/dev/ttyUSB0" on Linux
  SerialPort(const std::string& portName, int baudRate = 9600);
  ~SerialPort();

  void send(const void* data, int size);
  std::string receive();

  void printStatus();

 private:
#ifdef _WIN32
  HANDLE hSerial;
#else
  int fd;
#endif
  std::string m_portName;
  int m_baud;
};

#endif