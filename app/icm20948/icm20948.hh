
#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"

namespace app {
  struct ICM20948Config {
    struct SPICommConfig {
      spi_host_device_t channel;
      int clock_speed_hz;
      gpio_num_t chip_select;
    };

    SPICommConfig comm_config;
  };

  class ICM20948 {
  public:
    ICM20948(const ICM20948Config &config);
    ~ICM20948();

    int who_am_i();

  private:
    void disable_i2c();

  public:
    ICM20948(const ICM20948 &other) = delete;
    ICM20948(ICM20948 &&other);

    ICM20948 &operator=(const ICM20948 &other) = delete;
    ICM20948 &operator=(ICM20948 &&other);

  private:
    ICM20948Config config_;
    spi_device_handle_t device_handle_;
    int active_bank_;


  };
}
