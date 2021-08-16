
#pragma once

#include <memory>

#include "hal/i2c_types.h"


namespace app {
  struct BMP280Config {
    struct I2CCommConfig {
      i2c_port_t i2c_num;
      uint8_t address;
    };
    enum class OversampleConfig {
                             skip, x1, x2, x4, x8, x16
    };
    enum class PowerMode {
                          sleep, forced, normal
    };
    enum class StandbyTime {ms_0_5, ms_62_5, ms_125, ms_250, ms_500, ms_1000, ms_2000, ms_4000};
    enum class FilterTimeConstant {off, x2, x4, x8, x16};
    I2CCommConfig comm_config;
    OversampleConfig temp_oversample_config;
    OversampleConfig pressure_oversample_config;
    PowerMode power_mode;
    StandbyTime standby_time;
    FilterTimeConstant filter_constant;
  };
  struct BMP280CalibrationParams;
  struct BMP280CalibrationParamsDeleter {void operator()(BMP280CalibrationParams *) const;};

  class BMP280 {
  public:
    struct TempAndPressure {
      double temperature_degC;
      double pressure_kPa;
    };

    BMP280(const BMP280Config &config);
    int chip_id();
    void reset();
    TempAndPressure read_sensor();

  private:
    BMP280Config config_;
    std::unique_ptr<BMP280CalibrationParams, BMP280CalibrationParamsDeleter> params_;
  };
}
