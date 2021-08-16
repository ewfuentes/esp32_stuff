
#include "bmp280.hh"

#include "driver/i2c.h"

#include <iostream>
#include <vector>

namespace app {
  struct BMP280CalibrationParams{
    std::uint16_t dig_t1;
    std::int16_t dig_t2;
    std::int16_t dig_t3;
    std::uint16_t dig_p1;
    std::int16_t dig_p2;
    std::int16_t dig_p3;
    std::int16_t dig_p4;
    std::int16_t dig_p5;
    std::int16_t dig_p6;
    std::int16_t dig_p7;
    std::int16_t dig_p8;
    std::int16_t dig_p9;
  };

  void BMP280CalibrationParamsDeleter::operator()(BMP280CalibrationParams *p) const {delete p;}


  namespace {
    enum Register : uint8_t {
      CALIB_START = 0x88,
      ID = 0xD0,
      RESET = 0xE0,
      STATUS = 0xF3,
      CTRL_MEAS = 0xF4,
      CONFIG = 0xF5,
      PRESS_MSB = 0xF7,
      PRESS_LSB = 0xF8,
      PRESS_XLSB = 0xF9,
      TEMP_MSB = 0xFA,
      TEMP_LSB = 0xFB,
      TEMP_XLSB = 0xFC,
    };

    std::vector<uint8_t> read_register(const BMP280Config::I2CCommConfig &cfg, const Register reg, const int size) {
      {
        const uint8_t buf[] = {reg};
        ESP_ERROR_CHECK(i2c_master_write_to_device(cfg.i2c_num, cfg.address, buf, sizeof(buf),
                                                   CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS));
      }
      std::vector<uint8_t> out(size, 0x00);
      ESP_ERROR_CHECK(i2c_master_read_from_device(cfg.i2c_num, cfg.address, out.data(), size,
                                                  CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS));
      return out;
    }

    void write_register(const BMP280Config::I2CCommConfig &cfg, const Register reg, const uint8_t data) {
      uint8_t buf[2] = {reg, data};
      ESP_ERROR_CHECK(i2c_master_write_to_device(cfg.i2c_num, cfg.address, buf, sizeof(buf), CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS));
    }

    uint8_t standby_from_config(const BMP280Config::StandbyTime time) {
      switch (time)  {
      case BMP280Config::StandbyTime::ms_0_5:
        return 0x00;
      case BMP280Config::StandbyTime::ms_62_5:
        return 0x01;
      case BMP280Config::StandbyTime::ms_125:
        return 0x02;
      case BMP280Config::StandbyTime::ms_250:
        return 0x03;
      case BMP280Config::StandbyTime::ms_500:
        return 0x04;
      case BMP280Config::StandbyTime::ms_1000:
        return 0x05;
      case BMP280Config::StandbyTime::ms_2000:
        return 0x06;
      case BMP280Config::StandbyTime::ms_4000:
        return 0x07;
      }
      return 0x00;
    }

    uint8_t filter_from_config(const BMP280Config::FilterTimeConstant constant) {
      switch(constant) {
      case BMP280Config::FilterTimeConstant::off:
        return 0x00;
      case BMP280Config::FilterTimeConstant::x2:
        return 0x01;
      case BMP280Config::FilterTimeConstant::x4:
        return 0x02;
      case BMP280Config::FilterTimeConstant::x8:
        return 0x03;
      case BMP280Config::FilterTimeConstant::x16:
        return 0x04;
      }
      return 0x00;
    }

    uint8_t oversample_from_config(const BMP280Config::OversampleConfig oversample) {
      switch(oversample) {
      case BMP280Config::OversampleConfig::skip:
        return 0x00;
      case BMP280Config::OversampleConfig::x1:
        return 0x01;
      case BMP280Config::OversampleConfig::x2:
        return 0x02;
      case BMP280Config::OversampleConfig::x4:
        return 0x03;
      case BMP280Config::OversampleConfig::x8:
        return 0x04;
      case BMP280Config::OversampleConfig::x16:
        return 0x05;
      }
      return 0x00;
    }

    uint8_t power_mode_from_config(const BMP280Config::PowerMode mode) {
      switch(mode) {
      case BMP280Config::PowerMode::sleep:
        return 0x00;
      case BMP280Config::PowerMode::forced:
        return 0x01;
      case BMP280Config::PowerMode::normal:
        return 0x03;
      }
      return 0x00;
    }

    void write_config(const BMP280Config &cfg) {
      uint8_t data = (standby_from_config(cfg.standby_time) << 5) | (filter_from_config(cfg.filter_constant) << 2);
      write_register(cfg.comm_config, Register::CONFIG, data);
    }

    void write_ctrl_meas(const BMP280Config &cfg) {
      uint8_t data = (oversample_from_config(cfg.temp_oversample_config) << 5 )
        | (oversample_from_config(cfg.pressure_oversample_config) << 2)
        | power_mode_from_config(cfg.power_mode);
      write_register(cfg.comm_config, Register::CTRL_MEAS, data);
    }

    std::unique_ptr<BMP280CalibrationParams, BMP280CalibrationParamsDeleter> read_calibration(const BMP280Config::I2CCommConfig &cfg) {
      const auto &bytes = read_register(cfg, Register::CALIB_START, 24);
      std::cout << "calibration: ";
      for (int b : bytes) {
        std::cout << b << " ";
      }
      std::cout << std::endl;;
      auto calibration_params = new BMP280CalibrationParams();
      calibration_params->dig_t1 = bytes.at(0) | (bytes.at(1) << 8);
      calibration_params->dig_t2 = bytes.at(2) | (bytes.at(3) << 8);
      calibration_params->dig_t3 = bytes.at(4) | (bytes.at(5) << 8);
      calibration_params->dig_p1 = bytes.at(6) | (bytes.at(7) << 8);
      calibration_params->dig_p2 = bytes.at(8) | (bytes.at(9) << 8);
      calibration_params->dig_p3 = bytes.at(10) | (bytes.at(11) << 8);
      calibration_params->dig_p4 = bytes.at(12) | (bytes.at(13) << 8);
      calibration_params->dig_p5 = bytes.at(14) | (bytes.at(15) << 8);
      calibration_params->dig_p6 = bytes.at(16) | (bytes.at(17) << 8);
      calibration_params->dig_p7 = bytes.at(18) | (bytes.at(19) << 8);
      calibration_params->dig_p8 = bytes.at(20) | (bytes.at(21) << 8);
      calibration_params->dig_p9 = bytes.at(22) | (bytes.at(23) << 8);
      return std::unique_ptr<BMP280CalibrationParams, BMP280CalibrationParamsDeleter>(calibration_params);
    }

    BMP280::TempAndPressure compensate_temp_and_pressure(const int temp_raw, const int pressure_raw, const BMP280CalibrationParams params) {
      double t_fine = 0;
      double temp_degC = 0;
      {
        double var1 = (temp_raw / 16384.0 - params.dig_t1 / 1024.0) * params.dig_t2;
        double var2 = ((temp_raw / 131072.0) - params.dig_t1 / 8192.0);
        var2 = var2 * var2 * params.dig_t3;
        t_fine = var1 + var2;
        temp_degC = t_fine / 5120.0;
      }
      double pressure_pa;
      {
        double var1 = (t_fine / 2.0) - 64000.0;
        double var2 = var1 * var1 * (params.dig_p6 / 32768.0);
        var2 = var2 + var1 * (params.dig_p5) * 2.0;
        var2 = (var2 / 4.0) + (params.dig_p4 * 65536.0);
        var1 = (params.dig_p3 * var1 * var1 / 524288.0  + params.dig_p2 * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * params.dig_p1;
        pressure_pa = 1048576.0 - pressure_raw;
        pressure_pa = (pressure_pa - var2 / 4096.0) * 6250.0 / var1;
        var1 = params.dig_p9 * pressure_pa * pressure_pa / 2147483648.0;
        var2 = pressure_pa * params.dig_p8 / 32768.0;
        pressure_pa = pressure_pa + (var1 + var2 + params.dig_p7) / 16.0;
      }
      return {.temperature_degC = temp_degC, .pressure_kPa = pressure_pa / 1000.0};
    }
  }

  BMP280::BMP280(const BMP280Config &config) : config_(config) {
    reset();
    for (int i = 0; i < 10; i++) {
      // perform some reads to give the chip a chance to start up
       chip_id();
    }
    params_ = std::move(read_calibration(config.comm_config));
    write_config(config_);
    write_ctrl_meas(config_);
  };

  int BMP280::chip_id() {
    const auto out = read_register(config_.comm_config, Register::ID, 1);
    return out[0];
  }

  void BMP280::reset() {
    constexpr uint8_t RESET_BYTE = 0xB6;
    write_register(config_.comm_config, Register::RESET, RESET_BYTE);
  }

  BMP280::TempAndPressure BMP280::read_sensor() {
    const auto bytes = read_register(config_.comm_config, Register::PRESS_MSB, 6);
    const int temp_raw = (bytes.at(3) << 12) | (bytes.at(4) << 4) | (bytes.at(5) >> 4);
    const int pressure_raw = (bytes.at(0) << 12) | (bytes.at(1) << 4) | (bytes.at(2) >> 4);
    return compensate_temp_and_pressure(temp_raw, pressure_raw, *params_);
  }
}
