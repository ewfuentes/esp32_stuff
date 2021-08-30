
#pragma once

#include <array>
#include <vector>
#include <functional>

#include "driver/gpio.h"
#include "driver/spi_master.h"


namespace app {
  struct ICM20948Config {
    enum class GyroScale : uint8_t {k250_dps = 0, k500_dps = 1, k1000_dps = 2, k2000_dps = 3};

    enum class AccelScale : uint8_t {k2g = 0, k4g = 1, k8g = 2, k16g = 3};
    struct SPICommConfig {
      spi_host_device_t channel;
      int clock_speed_hz;
      gpio_num_t chip_select;
      gpio_num_t interrupt_pin;
    };

    SPICommConfig comm_config;
    float sample_rate_hz;
    GyroScale gyro_scale;
    AccelScale accel_scale;
  };

  struct ICM20948Sample {
    float accel_x_mpss;
    float accel_y_mpss;
    float accel_z_mpss;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temp_degC;
  };

  class ICM20948 {
  public:
    ICM20948(const ICM20948Config &config, const std::function<void(void)> &isr_callback);
    ~ICM20948();

    int who_am_i();

    void handle_interrupt();

    ICM20948Sample read_data();


    private:
    void disable_i2c();
    esp_err_t reset();
    esp_err_t configure_gyro(const float sample_rate_hz, const ICM20948Config::GyroScale scale);
    esp_err_t configure_accel(const float sample_rate_hz, const ICM20948Config::AccelScale scale);
    esp_err_t configure_interrupt();

    template <typename T>
    esp_err_t write_register(const T address, const std::vector<uint8_t> &data);

    template <typename T, size_t N>
    esp_err_t read_register(const T address, std::array<uint8_t, N> &data);

  public:
    ICM20948(const ICM20948 &other) = delete;
    ICM20948(ICM20948 &&other);

    ICM20948 &operator=(const ICM20948 &other) = delete;
    ICM20948 &operator=(ICM20948 &&other);

  private:
    ICM20948Config config_;
    spi_device_handle_t device_handle_;
    int active_bank_;
    std::function<void(void)> isr_callback_;


  };
}
