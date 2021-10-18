
#pragma once

#include <cstdint>
#include <vector>

#include "protobuf-c/protobuf-c.h"

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

  void send_data(const ProtobufCMessage &msg);
private:
  int port_;
  int socket_handle_;
  std::vector<std::uint8_t> packet_queue_;
};
} // namespace app
