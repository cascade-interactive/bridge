#ifndef UDP_SOCKET_HPP
#define UDP_SOCKET_HPP

#include <string>

// make sure we use winsock for windows or it breaks
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
typedef int socketlen_t;
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

typedef int SOCKET;
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#endif

class UDPSocket {
 public:
  UDPSocket(int port = 0, bool non_blocking = true);
  ~UDPSocket();

  void send(std::string ip, int port, std::string message);
  std::string receive();

 private:
  SOCKET sock;
  bool m_non_blocking;
};

#endif