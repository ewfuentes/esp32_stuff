idf_component_register(SRCS
  "udp_comm.cc"
  "proto/imu_samples.pb.cc"
  INCLUDE_DIRS "." "proto/"
  PRIV_REQURES protobuf-c)
