#pragma once

#include "proto-c/imu_samples.pb-c.h"
#include "icm20948.hh"

namespace app {
struct TimestampedIMU {
  int64_t time_us;
  ICM20948Sample sample;
};

namespace proto {
void add_into(const TimestampedIMU &in, App__UdpComm__ImuSamples *out);

void clear(App__UdpComm__ImuSamples *out);
bool is_full(const App__UdpComm__ImuSamples &in);
}
}
