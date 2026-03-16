#include "bridge/sockets/UDPSocket.hpp"
#include <iostream>

UDPSocket::UDPSocket(int port, bool non_blocking)
    : m_non_blocking(non_blocking) {

// windows needs to initialize winsock
#ifdef _WIN32
  WSADATA wsaData;
  WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

  sock = socket(AF_INET, SOCK_DGRAM, 0);

  if (m_non_blocking) {
#ifdef _WIN32
    unsigned long mode = 1;
    ioctlsocket(sock, FIONBIO, &mode);
#else
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
#endif
  }

  if (sock == INVALID_SOCKET) {
    std::cerr << "Failed to create socket" << std::endl;
  }

  if (port > 0) {
    sockaddr_in addr;
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(port);

    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
      std::cerr << "Failed to bind socket. Port: " << port
                << " might be in use." << std::endl;
    } else {
      std::cout << "Socket bound to port " << port << std::endl;
    }
  }
}

UDPSocket::~UDPSocket() {
#ifdef _WIN32
  closesocket(sock);
  WSACleanup();
#else
  close(sock);
#endif
  std::cout << "Socket closed" << std::endl;
}

void UDPSocket::send(const std::string& ip, int port, const void* data,
                     int size) {
  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_port   = htons(port);
  inet_pton(AF_INET, ip.c_str(), &dest.sin_addr);

  sendto(sock, static_cast<const char*>(data), size, 0,
         reinterpret_cast<sockaddr*>(&dest), sizeof(dest));
}

std::string UDPSocket::receive() {
  char buffer[1024];
  sockaddr_in sender_addr;
  socklen_t sender_len = sizeof(sender_addr);

  int bytes_received = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
                                (sockaddr*)&sender_addr, &sender_len);

  if (bytes_received > 0) {
    return std::string(buffer, bytes_received);
  }

  // Non blocking returns -1 for bytes_received if no data is available, so we just return an empty string
  return "";
}