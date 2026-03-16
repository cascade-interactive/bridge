#include "bridge/sockets/SerialPort.hpp"
#include <iostream>

SerialPort::SerialPort(const std::string& portName, int baudRate)
    : m_portName(portName), m_baud(baudRate) {

#ifdef _WIN32
  hSerial = CreateFileA(m_portName.c_str(), GENERIC_READ | GENERIC_WRITE, 0,
                        NULL, OPEN_EXISTING, 0, NULL);

  if (hSerial == INVALID_HANDLE_VALUE) {
    std::cerr << "[SERIAL] Error: Could not open " << m_portName << std::endl;
    return;
  }

  DCB dcb       = {0};
  dcb.DCBlength = sizeof(dcb);
  GetCommState(hSerial, &dcb);
  dcb.BaudRate = m_baud;
  dcb.ByteSize = 8;
  dcb.StopBits = ONESTOPBIT;
  dcb.Parity   = NOPARITY;
  SetCommState(hSerial, &dcb);

  COMMTIMEOUTS timeouts        = {0};
  timeouts.ReadIntervalTimeout = MAXDWORD;
  SetCommTimeouts(hSerial, &timeouts);
#else
  fd = open(m_portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd == -1) {
    std::cerr << "[SERIAL] Error: Could not open " << m_portName << std::endl;
    return;
  }

  struct termios tty;
  tcgetattr(fd, &tty);
  cfsetospeed(&tty, m_baud);
  cfsetispeed(&tty, m_baud);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tcsetattr(fd, TCSANOW, &tty);
#endif

  std::cout << "[SERIAL] " << m_portName << " initialized at " << m_baud
            << " baud." << std::endl;
}

SerialPort::~SerialPort() {
#ifdef _WIN32
  if (hSerial != INVALID_HANDLE_VALUE)
    CloseHandle(hSerial);
#else
  if (fd != -1)
    close(fd);
#endif
  std::cout << "[SERIAL] " << m_portName << " closed." << std::endl;
}

void SerialPort::printStatus() {
  bool isOpen = false;
#ifdef _WIN32
  isOpen = (hSerial != INVALID_HANDLE_VALUE);
#else
  isOpen = (fd != -1);
#endif

  std::cout << "--- Serial Status ---" << std::endl;
  std::cout << "Port:   " << m_portName << std::endl;
  std::cout << "Baud:   " << m_baud << std::endl;
  std::cout << "Status: " << (isOpen ? "CONNECTED" : "DISCONNECTED/FAILED")
            << std::endl;
  std::cout << "---------------------" << std::endl;
}

void SerialPort::send(const void* data, int size) {
#ifdef _WIN32
  DWORD written;
  WriteFile(hSerial, data, size, &written, NULL);
#else
  write(fd, data, size);
#endif
}

std::string SerialPort::receive() {
  char buffer[256];
  int bytesRead = 0;

#ifdef _WIN32
  DWORD dwRead;
  if (ReadFile(hSerial, buffer, sizeof(buffer), &dwRead, NULL)) {
    bytesRead = (int)dwRead;
  }
#else
  bytesRead = read(fd, buffer, sizeof(buffer));
#endif

  if (bytesRead > 0) {
    return std::string(buffer, bytesRead);
  }
  return "";
}