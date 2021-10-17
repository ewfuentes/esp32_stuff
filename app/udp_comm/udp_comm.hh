
#pragma once

#include <cstdint>
#include <vector>

namespace app {

  struct UdpCommConfig {
    int port;
  };

class UdpComm {
public:
  UdpComm(const UdpCommConfig &config);
  ~UdpComm();
  UdpComm(const UdpComm &other) = delete;
  UdpComm &operator=(const UdpComm &other) = delete;

  template <typename T>
  void send_data(const T &data);
  void flush();
private:
  int port_;
  int socket_handle_;
  std::vector<std::uint8_t> packet_queue_;
};
} // namespace app
