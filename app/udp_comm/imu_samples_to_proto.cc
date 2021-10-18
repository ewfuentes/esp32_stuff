
#include "imu_samples_to_proto.hh"

namespace app {
namespace proto {
  namespace {
    constexpr int NUM_SAMPLES = 32;
  }


void add_into(const TimestampedIMU &in, App__UdpComm__ImuSamples *out) {
  if (out->timestamps == nullptr) {
    // Allocate enough space for 32 samples
    out->timestamps = (int64_t *)malloc(sizeof(int64_t) * NUM_SAMPLES);
    out->accel_x_mpss = (float *)malloc(sizeof(float) * NUM_SAMPLES);
    out->accel_y_mpss = (float *)malloc(sizeof(float) * NUM_SAMPLES);
    out->accel_z_mpss = (float *)malloc(sizeof(float) * NUM_SAMPLES);
    out->gyro_x_dps = (float *)malloc(sizeof(float) * NUM_SAMPLES);
    out->gyro_y_dps = (float *)malloc(sizeof(float) * NUM_SAMPLES);
    out->gyro_z_dps = (float *)malloc(sizeof(float) * NUM_SAMPLES);
    out->temp_degc = (float *)malloc(sizeof(float) * NUM_SAMPLES);
  }
  if (out->n_timestamps == 0) {
    out->time_of_validity_us = in.time_us;
  }
  out->timestamps[out->n_timestamps++] = in.time_us - out->time_of_validity_us;
  out->accel_x_mpss[out->n_accel_x_mpss++] = in.sample.accel_x_mpss;
  out->accel_y_mpss[out->n_accel_y_mpss++] = in.sample.accel_y_mpss;
  out->accel_z_mpss[out->n_accel_z_mpss++] = in.sample.accel_z_mpss;
  out->gyro_x_dps[out->n_gyro_x_dps++] = in.sample.gyro_x_dps;
  out->gyro_y_dps[out->n_gyro_y_dps++] = in.sample.gyro_y_dps;
  out->gyro_z_dps[out->n_gyro_z_dps++] = in.sample.gyro_z_dps;
  out->temp_degc[out->n_temp_degc++] = in.sample.temp_degc;
}

  void clear(App__UdpComm__ImuSamples *out) {
    out->time_of_validity_us = 0;
    out->n_timestamps = 0;
    out->n_accel_x_mpss = 0;
    out->n_accel_y_mpss = 0;
    out->n_accel_z_mpss = 0;
    out->n_gyro_x_dps = 0;
    out->n_gyro_y_dps = 0;
    out->n_gyro_z_dps = 0;
    out->n_temp_degc = 0;
  }

  bool is_full(const App__UdpComm__ImuSamples &in) {
    return in.timestamps != nullptr && in.n_timestamps >= NUM_SAMPLES;
  }
}
}
