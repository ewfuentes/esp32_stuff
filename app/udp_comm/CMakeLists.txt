set(PROTO_COMPILER "protoc")
set(PROTO_C_COMPILER "protoc-c")
set(C_OUT_PATH "${CMAKE_CURRENT_LIST_DIR}/proto-c")
set(PY_OUT_PATH "${CMAKE_CURRENT_LIST_DIR}/proto-py")

set(PROTO_SRCS "${CMAKE_CURRENT_LIST_DIR}/imu_samples.proto")

set(pb_csrcs
  "${C_OUT_PATH}/imu_samples.pb-c.c")

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
  add_custom_command(OUTPUT ${pb_csrcs}
    COMMAND mkdir -p ${C_OUT_PATH}
    COMMAND mkdir -p ${PY_OUT_PATH}
    COMMAND ${PROTO_C_COMPILER} --c_out=${C_OUT_PATH} -I . --proto_path=${CMAKE_CURRENT_LIST_DIR} ${PROTO_SRCS}
    COMMAND ${PROTO_COMPILER} --python_out=${PY_OUT_PATH} -I . --proto_path=${CMAKE_CURRENT_LIST_DIR} ${PROTO_SRCS}
    DEPENDS ${PROTO_SRCS}
    VERBATIM)
endif()

idf_component_register(SRCS
  ${pb_csrcs}
  "udp_comm.cc"
  "imu_samples_to_proto.cc"
  INCLUDE_DIRS "."
  PRIV_REQUIRES protobuf-c icm20948)
