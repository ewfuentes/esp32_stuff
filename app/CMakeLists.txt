# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS
  $ENV{IDF_PATH}/app/fonts
  $ENV{IDF_PATH}/app/bmp280
  $ENV{IDF_PATH}/app/icm20948
  $ENV{IDF_PATH}/app/udp_comm
  $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(my_app)
