/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: imu_samples.proto */

#ifndef PROTOBUF_C_imu_5fsamples_2eproto__INCLUDED
#define PROTOBUF_C_imu_5fsamples_2eproto__INCLUDED

#include <protobuf-c/protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1003000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1003003 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif


typedef struct _App__UdpComm__ImuSamples App__UdpComm__ImuSamples;


/* --- enums --- */


/* --- messages --- */

struct  _App__UdpComm__ImuSamples
{
  ProtobufCMessage base;
  int64_t time_of_validity_us;
  size_t n_timestamps;
  int64_t *timestamps;
  size_t n_accel_x_mpss;
  float *accel_x_mpss;
  size_t n_accel_y_mpss;
  float *accel_y_mpss;
  size_t n_accel_z_mpss;
  float *accel_z_mpss;
  size_t n_gyro_x_dps;
  float *gyro_x_dps;
  size_t n_gyro_y_dps;
  float *gyro_y_dps;
  size_t n_gyro_z_dps;
  float *gyro_z_dps;
  size_t n_temp_deg;
  float *temp_deg;
};
#define APP__UDP_COMM__IMU_SAMPLES__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&app__udp_comm__imu_samples__descriptor) \
    , 0, 0,NULL, 0,NULL, 0,NULL, 0,NULL, 0,NULL, 0,NULL, 0,NULL, 0,NULL }


/* App__UdpComm__ImuSamples methods */
void   app__udp_comm__imu_samples__init
                     (App__UdpComm__ImuSamples         *message);
size_t app__udp_comm__imu_samples__get_packed_size
                     (const App__UdpComm__ImuSamples   *message);
size_t app__udp_comm__imu_samples__pack
                     (const App__UdpComm__ImuSamples   *message,
                      uint8_t             *out);
size_t app__udp_comm__imu_samples__pack_to_buffer
                     (const App__UdpComm__ImuSamples   *message,
                      ProtobufCBuffer     *buffer);
App__UdpComm__ImuSamples *
       app__udp_comm__imu_samples__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   app__udp_comm__imu_samples__free_unpacked
                     (App__UdpComm__ImuSamples *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*App__UdpComm__ImuSamples_Closure)
                 (const App__UdpComm__ImuSamples *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCMessageDescriptor app__udp_comm__imu_samples__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_imu_5fsamples_2eproto__INCLUDED */
