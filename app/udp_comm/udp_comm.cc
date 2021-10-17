
#include "udp_comm.hh"

#include "esp_log.h"
#include "lwip/sockets.h"

namespace app {
namespace {
const char *TAG = "UdpComm";
}

UdpComm::UdpComm(const UdpCommConfig &config) : port_(config.port) {
  packet_queue_.reserve(1200);
  socket_handle_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  ESP_LOGI(TAG, "Socket FD: %d", socket_handle_);
  struct sockaddr_in in;
  in.sin_family = AF_INET;
  in.sin_addr.s_addr = htonl(INADDR_ANY);
  in.sin_port = htons(port_);

  const int bind_result =
      bind(socket_handle_, (struct sockaddr *)&in, sizeof(sockaddr_in));
  ESP_LOGI(TAG, "Bind Result: %d", bind_result);
  int opt = 0;
  setsockopt(socket_handle_, IPPROTO_IP, IP_MULTICAST_LOOP, &opt, sizeof(opt));
}

void UdpComm::flush() {
  sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port_);
  addr.sin_addr.s_addr = htonl((239 << 24)| (1 << 16) | (2 << 8) | (4));
  sendto(socket_handle_, packet_queue_.data(), packet_queue_.size(),
         MSG_DONTWAIT, (const sockaddr *)&addr, sizeof(addr));
  packet_queue_.clear();
}

template <>
void UdpComm::send_data<T>(const T &proto_msg) {
  const auto data = proto_msg.SerializeToString();
  packet_queue_.clear();
  std::copy(data.begin(), data.end(), std::back_inserter(packet_queue_));
  flush();
}

UdpComm::~UdpComm() {
  shutdown(socket_handle_, 0);
  close(socket_handle_);
}
} // namespace app
