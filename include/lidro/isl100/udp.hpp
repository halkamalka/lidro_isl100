#ifndef NET_UDP_H_
#define NET_UDP_H_

#include <thread>
#include <mutex>

#include <string>
#include <functional>
#include <cstdint>
#include <sys/un.h>
#include <sys/socket.h>
#include <stdint.h>
#include <netinet/in.h>
#include <functional>
#include <thread>
#include <atomic>

namespace net {

using Buffer = std::unique_ptr<uint8_t, std::function<void(uint8_t *)>>;
using OnRead = std::function<void(uint8_t *, int)>;

class udp {
public:
  udp() = default;
  ~udp();

  void setOnRead(OnRead on_read){on_read_ = on_read;}
  void setAddr(const std::string &addr){addr_ = addr;}
  void setPort(int port){port_ = port;}
  void setRemotePort(const std::string &ip, int port);

  int init();
  int read(uint8_t *buf, size_t len);
  int send(uint8_t *buf, size_t len);

  void setTimeout(unsigned long ms){timeout_ = ms;}

  int start();
  void stop();

private:
  std::string addr_{};
  int port_{0};
  int local_port_{0};

  int fd_{0};
  struct sockaddr_in local_addr_{};
  struct sockaddr_in remote_addr_{};
  unsigned long timeout_{100}; // ms

  std::unique_ptr<std::thread> thread_{nullptr};
  std::atomic<bool> stopped_{false};

  OnRead on_read_{nullptr};
};

} // namespace net

#endif // NET_UDP_H_
