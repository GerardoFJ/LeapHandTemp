#pragma once

#ifdef _WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib")
#endif

#include <cstdio>
#include <string>


class UdpSender {
 public:
  ~UdpSender() {
    if (socket_ != INVALID_SOCKET) {
      closesocket(socket_);
    }
    if (started_) {
      WSACleanup();
    }
  }

  bool open(const std::string& host, int port) {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
      printf("WSAStartup failed.\n");
      return false;
    }
    started_ = true;

    socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ == INVALID_SOCKET) {
      printf("socket() failed: %d\n", WSAGetLastError());
      return false;
    }
    destination_ = {};
    destination_.sin_family = AF_INET;
    destination_.sin_port = htons(static_cast<u_short>(port));
    if (inet_pton(AF_INET, host.c_str(), &destination_.sin_addr) != 1) {
      printf("--host must be an IPv4 address, got '%s'.\n", host.c_str());
      return false;
    }
    return true;
  }

  void send(const std::string& message) {
    sendto(socket_, message.data(), static_cast<int>(message.size()), 0,
           reinterpret_cast<const sockaddr*>(&destination_), sizeof(destination_));
  }

 private:
  SOCKET socket_ = INVALID_SOCKET;
  bool started_ = false;
  sockaddr_in destination_ = {};
};
