#include "lidro/isl100/udp.hpp"

#include <sys/un.h>
#include <sys/socket.h>
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <iostream>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/stat.h>
#include "../log_message.h"

std::string uint32_to_ip(uint32_t ip) {
  struct in_addr addr;
  addr.s_addr = htonl(ip); // Convert to network byte order

  char ip_str[INET_ADDRSTRLEN];
  const char *result = inet_ntop(AF_INET, &addr, ip_str, INET_ADDRSTRLEN);

  if (result == nullptr) {
    return {};
  }

  return std::string(ip_str);
}

namespace net {

udp::~udp() {
  if (fd_ > 0)
    close(fd_);
  fd_ = -1;
}

void udp::setRemotePort(const std::string &ip, int port) {
  memset(&remote_addr_, 0, sizeof(remote_addr_));
  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_port = htons(port);
  remote_addr_.sin_addr.s_addr = inet_addr(ip.c_str());
}

int udp::init() {
  if (port_ == 0) {
    std::cerr << "udp not set" << std::endl;
    return -1;
  }

  int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) {
    fprintf(stderr, "failed to create a socket descriptor\n");
    return -1;
  }

  int enable = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));

  memset(&local_addr_, 0, sizeof(local_addr_));
  local_addr_.sin_family = AF_INET;
  local_addr_.sin_addr.s_addr = addr_.empty() ? htonl(INADDR_ANY) : inet_addr(addr_.c_str());
  local_addr_.sin_port = htons(port_);

  if (::bind(fd, (struct sockaddr *)&local_addr_, sizeof(local_addr_)) < 0) {
    fprintf(stderr, "failed to bind\n");
    return -1;
  }

  // Define the remote address
  fd_ = fd;

  return 0;
}

int udp::send(uint8_t *buf, size_t len) {
  // struct sockaddr_in server_addr;

  return sendto(fd_, buf, len, 0, (const struct sockaddr *)&remote_addr_, sizeof(remote_addr_));
}

int udp::read(uint8_t *buf, size_t len) {
  fd_set readfds;
  struct timeval tv = {};
  struct timeval *p_tv = NULL;

  FD_ZERO(&readfds);
  FD_SET(fd_, &readfds);

  if (timeout_ > 0) {
    tv.tv_sec = timeout_ / 1000;
    tv.tv_usec = (timeout_ % 1000) * 1000;
    p_tv = &tv;
  }

  int stat = select(fd_ + 1, &readfds, NULL, NULL, p_tv);
  if (stat == 0) { // time-over
    // std::cerr << "," << std::endl;
    return -1;
  }
  else if (0 < stat) {
    int n;
    if (ioctl(fd_, FIONREAD, &n) < 0) {
      std::cerr << "failed to ioctl" << std::endl;
      return -1;
    }

    if (n == 0) {
      FD_CLR(fd_, &readfds);
      std::cerr << "no data" << std::endl;
    }
  }

  return ::read(fd_, buf, len);
}

int udp::start() {
  thread_ = std::make_unique<std::thread>(
  [this](){
    uint8_t buf[1500] = {};
    while (!stopped_) {
      int n = read(buf, sizeof(buf));
      if (n > 0) {
        if (on_read_)
          on_read_(buf, n);
      }
    }
  });
}

void udp::stop() {
  if (stopped_) {
    return;
  }

  stopped_ = true;
  if (thread_)
    thread_->join();
  thread_ = nullptr;
}

} // namespace net
