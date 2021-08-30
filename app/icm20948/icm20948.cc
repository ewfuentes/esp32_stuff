
#include "icm20948.hh"

#include <algorithm>
#include <iostream>
#include <cstdint>
#include <type_traits>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/gpio.h"

namespace app {
  namespace {
    static const char *TAG = "ICM20948";
    constexpr std::uint8_t READ_COMMAND = 0x01;
    constexpr std::uint8_t WRITE_COMMAND = 0x00;
    enum class UserBank0: uint8_t {
      WHO_AM_I = 0x00,
      USER_CTRL = 0x03,
      LP_CONFIG = 0x05,
      PWR_MGMT_1 = 0x06,
      PWR_MGMT_2 = 0x07,
      INT_PIN_CFG = 0x0F,
      INT_ENABLE = 0x10,
      INT_ENABLE_1 = 0x11,
      INT_ENABLE_2 = 0x12,
      INT_ENABLE_3 = 0x13,
      I2C_MST_STATUS = 0x17,
      INT_STATUS = 0x19,
      INT_STATUS_1 = 0x1A,
      INT_STATUS_2 = 0x1B,
      INT_STATUS_3 = 0x1C,
      DELAY_TIMEH = 0x28,
      DELAY_TIMEL = 0x29,
      ACCEL_XOUT_H = 0x2D,
      ACCEL_XOUT_L = 0x2E,
      ACCEL_YOUT_H = 0x2F,
      ACCEL_YOUT_L = 0x30,
      ACCEL_ZOUT_H = 0x31,
      ACCEL_ZOUT_L = 0x32,
      GYRO_XOUT_H = 0x33,
      GYRO_XOUT_L = 0x34,
      GYRO_YOUT_H = 0x35,
      GYRO_YOUT_L = 0x36,
      GYRO_ZOUT_H = 0x37,
      GYRO_ZOUT_L = 0x38,
      TEMP_OUT_H = 0x39,
      TEMP_OUT_L = 0x3A,
      FIFO_EN_1 = 0x66,
      FIFO_EN_2 = 0x67,
      FIFO_RST = 0x68,
      FIFO_MODE = 0x69,
      FIFO_COUNT_H = 0x70,
      FIFO_COUNT_L = 0x71,
      FIFO_R_W = 0x72,
      DATA_RDY_STATUS = 0x74,
      FIFO_CFG = 0x76,
      REG_BANK_SEL = 0x7F,
    };
    enum class UserBank1: uint8_t {
      SELF_TEST_X_GYRO = 0x02,
      SELF_TEST_Y_GYRO = 0x03,
      SELF_TEST_Z_GYRO = 0x04,
      SELF_TEST_X_ACCEL = 0x0E,
      SELF_TEST_Y_ACCEL = 0x0F,
      SELF_TEST_Z_ACCEL = 0x10,
      XA_OFFS_H = 0x14,
      XA_OFFS_L = 0x15,
      YA_OFFS_H = 0x17,
      YA_OFFS_L = 0x18,
      ZA_OFFS_H = 0x1A,
      ZA_OFFS_L = 0x1B,
      TIMEBASE_CORRECTION_PLL = 0x28,
      REG_BANK_SEL = 0x7F,
    };
    enum class UserBank2: uint8_t {
      GYRO_SMPLRT_DIV = 0x00,
      GYRO_CONFIG_1 = 0x01,
      GYRO_CONFIG_2 = 0x02,
      XG_OFFS_USRH = 0x03,
      XG_OFFS_USRL = 0x04,
      YG_OFFS_USRH = 0x05,
      YG_OFFS_USRL = 0x06,
      ZG_OFFS_USRH = 0x07,
      ZG_OFFS_USRL = 0x08,
      ODR_ALIGN_EN = 0x09,
      ACCEL_SMPLRT_DIV_1 = 0x10,
      ACCEL_SMPLRT_DIV_2 = 0x11,
      ACCEL_INTEL_CTRL = 0x12,
      ACCEL_WOM_THR = 0x13,
      ACCEL_CONFIG = 0x14,
      ACCEL_CONFIG_2 = 0x15,
      FSYNC_CONFIG = 0x52,
      TEMP_CONFIG = 0x53,
      MOD_CTRL_USR = 0x54,
      REG_BANK_SEL = 0x7F,
    };
    enum class UserBank3: uint8_t {
      REG_BANK_SEL = 0x7F,
    };

    struct FilterSetting {
      float filter_bw_hz;
      float nyquist_bw_hz;
      uint8_t setting;
    };

    constexpr std::array GYRO_FILTER_CONFIG{
      FilterSetting{.filter_bw_hz = 361.4, .nyquist_bw_hz = 376.5, .setting = 0x07},
      FilterSetting{.filter_bw_hz = 196.6, .nyquist_bw_hz = 229.8, .setting = 0x00},
      FilterSetting{.filter_bw_hz = 151.8, .nyquist_bw_hz = 187.6, .setting = 0x01},
      FilterSetting{.filter_bw_hz = 119.5, .nyquist_bw_hz = 154.3, .setting = 0x02},
      FilterSetting{.filter_bw_hz = 51.2, .nyquist_bw_hz = 73.3, .setting = 0x03},
      FilterSetting{.filter_bw_hz = 23.9, .nyquist_bw_hz = 35.9, .setting = 0x04},
      FilterSetting{.filter_bw_hz = 11.6, .nyquist_bw_hz = 17.8, .setting = 0x05},
      FilterSetting{.filter_bw_hz = 5.7, .nyquist_bw_hz = 8.9, .setting = 0x06},
    };

    constexpr std::array ACCEL_FILTER_CONFIG{
      FilterSetting{.filter_bw_hz = 473.0, .nyquist_bw_hz = 499.0, .setting = 0x07},
      FilterSetting{.filter_bw_hz = 246.0, .nyquist_bw_hz = 265.0, .setting = 0x00},
      FilterSetting{.filter_bw_hz = 246.8, .nyquist_bw_hz = 265.0, .setting = 0x01},
      FilterSetting{.filter_bw_hz = 111.4, .nyquist_bw_hz = 136.0, .setting = 0x02},
      FilterSetting{.filter_bw_hz = 50.4, .nyquist_bw_hz = 68.8, .setting = 0x03},
      FilterSetting{.filter_bw_hz = 23.9, .nyquist_bw_hz = 34.4, .setting = 0x04},
      FilterSetting{.filter_bw_hz = 11.5, .nyquist_bw_hz = 17.0, .setting = 0x05},
      FilterSetting{.filter_bw_hz = 5.7, .nyquist_bw_hz = 8.3, .setting = 0x06},
    };

    template <typename T>
    constexpr auto operator+(T value) -> std::enable_if_t<std::is_enum<T>::value, std::underlying_type_t<T>> {
      return static_cast<std::underlying_type_t<T>>(value);
    }

    template <typename ... Ts> struct TypeList{
      template<typename, int>
      static constexpr int get_index() {return -1;}
    };

    template <typename T, typename... Rest> struct TypeList<T, Rest...>{
      template<typename T1, int IDX=0>
      static constexpr int get_index() {
        return std::is_same<T, T1>::value ? IDX : TypeList<Rest...>::template get_index<T1, IDX+1>();
      }
    };

    using UserBanks = TypeList<UserBank0, UserBank1, UserBank2, UserBank3>;

    void handle_interrupt(void *arg) {
      static_cast<ICM20948 *>(arg)->handle_interrupt();
    }

    esp_err_t select_active_bank(spi_device_handle_t dev_handle, int bank){
      uint8_t data = (bank & 0x03) << 4;
      spi_transaction_t transaction = {
         .flags = SPI_TRANS_USE_TXDATA,
         .cmd = WRITE_COMMAND,
         .addr = +UserBank0::REG_BANK_SEL,
         .length = 8,
         .rxlength = 0,
         .user = nullptr,
         .tx_data = {data},
         .rx_buffer = nullptr
      };
      return spi_device_transmit(dev_handle, &transaction);
    }

    int compute_sample_rate_divider(const float base_rate_hz, const float sample_rate_hz) {
      return static_cast<int>(base_rate_hz / sample_rate_hz) - 1;
    }

    template <size_t N>
    uint8_t get_filter_setting(const float sample_rate_hz, const std::array<FilterSetting, N> &filter_settings) {
      auto iter = std::find_if(filter_settings.begin(),
                               filter_settings.end(),
                               [sample_rate_hz](const auto &setting) {
                                  return setting.nyquist_bw_hz < sample_rate_hz / 2.0;
                               });
      if (iter == filter_settings.end()) {
        iter -= 1;
      }
      return iter->setting;
    }
  }

  esp_err_t ICM20948::configure_gyro(const float sample_rate_hz, const ICM20948Config::GyroScale scale) {
    constexpr float BASE_RATE_HZ = 1100.0;
    const int sample_rate_divider = compute_sample_rate_divider(BASE_RATE_HZ, sample_rate_hz);
    const float true_sample_rate_hz = BASE_RATE_HZ / (sample_rate_divider + 1);

    // Set the sample rate divider
    ESP_ERROR_CHECK(write_register(UserBank2::GYRO_SMPLRT_DIV, {static_cast<uint8_t>(sample_rate_divider)}));

    // Set the filtering options
    const uint8_t enable_filtering = 0x01;
    const uint8_t filter_setting = get_filter_setting(true_sample_rate_hz, GYRO_FILTER_CONFIG);
    const uint8_t gyro_config_1 = (filter_setting << 3) | (+scale << 1) | enable_filtering;
    ESP_ERROR_CHECK(write_register(UserBank2::GYRO_CONFIG_1, {gyro_config_1}));

    return ESP_OK;
  }
  esp_err_t ICM20948::configure_accel(const float sample_rate_hz, const ICM20948Config::AccelScale scale) {
    constexpr float BASE_RATE_HZ = 1125.0;
    const int sample_rate_divider = compute_sample_rate_divider(BASE_RATE_HZ, sample_rate_hz);
    const float true_sample_rate_hz = BASE_RATE_HZ / (sample_rate_divider + 1);

    // Set Sample rate
    ESP_ERROR_CHECK(write_register(UserBank2::ACCEL_SMPLRT_DIV_1, {static_cast<uint8_t>((sample_rate_divider >> 8) & 0xFF),
                                                                                                 static_cast<uint8_t>(sample_rate_divider & 0xFF)}));

    // Set the filtering options
    const uint8_t enable_filtering = 0x01;
    const uint8_t filter_setting = get_filter_setting(true_sample_rate_hz, ACCEL_FILTER_CONFIG);
    const uint8_t accel_config = (filter_setting << 3) | (+scale << 1) | enable_filtering;
    ESP_ERROR_CHECK(write_register(UserBank2::ACCEL_CONFIG, {accel_config}));

    return ESP_OK;
  }

  esp_err_t ICM20948::configure_interrupt() {
    gpio_reset_pin(config_.comm_config.interrupt_pin);

    gpio_config_t int_pin_config = {
      .pin_bit_mask = (1u << config_.comm_config.interrupt_pin),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&int_pin_config);

    ESP_ERROR_CHECK(gpio_isr_handler_add(config_.comm_config.interrupt_pin, app::handle_interrupt, this));

    // Setup the interrupt pin
    // Interrupt pin is active high, configured as push_pull, the interrupt is latched until a
    // read is performed
    ESP_ERROR_CHECK(write_register(UserBank0::INT_PIN_CFG, {0x20}));

    // Interrupt on Raw Data Ready
    ESP_ERROR_CHECK(write_register(UserBank0::INT_ENABLE, {0x00}));
    ESP_ERROR_CHECK(write_register(UserBank0::INT_ENABLE_1, {0x01}));
    ESP_ERROR_CHECK(write_register(UserBank0::INT_ENABLE_2, {0x00}));
    ESP_ERROR_CHECK(write_register(UserBank0::INT_ENABLE_3, {0x00}));

    gpio_intr_enable(config_.comm_config.interrupt_pin);
    return ESP_OK;
  }

  ICM20948::ICM20948(const ICM20948Config &config, const std::function<void(void)> &isr_callback) : config_(config), device_handle_(nullptr), active_bank_(0) , isr_callback_(isr_callback) {
    const spi_device_interface_config_t dev_config = {
      .command_bits = 1,
      .address_bits = 7,
      .dummy_bits = 0,
      .mode = 0,
      .duty_cycle_pos = 0,
      .cs_ena_pretrans = 0,
      .cs_ena_posttrans = 0,
      .clock_speed_hz = config_.comm_config.clock_speed_hz,
      .input_delay_ns = 0,
      .spics_io_num = config_.comm_config.chip_select,
      .flags = 0,
      .queue_size = 16,
      .pre_cb = nullptr,
      .post_cb = nullptr
    };
    ESP_ERROR_CHECK(spi_bus_add_device(config_.comm_config.channel, &dev_config, &device_handle_));

    // Set the active bank
    select_active_bank(device_handle_, active_bank_);
    ESP_ERROR_CHECK(reset());

    disable_i2c();

    // Setup power management
    // Clear the sleep bit and auto select clock
    ESP_ERROR_CHECK(write_register(UserBank0::PWR_MGMT_1, {0x01}));

    // Setup the gyro and accelerometer
    ESP_ERROR_CHECK(configure_gyro(config_.sample_rate_hz, config_.gyro_scale));
    ESP_ERROR_CHECK(configure_accel(config_.sample_rate_hz, config_.accel_scale));
    // Setup interrupts
    ESP_ERROR_CHECK(configure_interrupt());
  }

  ICM20948::ICM20948(ICM20948 &&other) : config_(other.config_), device_handle_(other.device_handle_), isr_callback_(other.isr_callback_) {
    other.device_handle_ = nullptr;
  }

  ICM20948 &ICM20948::operator=(ICM20948 &&other) {
    config_ = other.config_;
    device_handle_ = other.device_handle_;
    other.device_handle_ = nullptr;
    other.isr_callback_ = isr_callback_;
    return *this;
  }

  ICM20948::~ICM20948() {
    if (device_handle_ != nullptr) {
      spi_bus_remove_device(device_handle_);
    }
  }

  int ICM20948::who_am_i(){
    std::array<uint8_t, 1> out;
    ESP_ERROR_CHECK(read_register(UserBank0::WHO_AM_I, out));
    return out[0];
  }

  void ICM20948::disable_i2c() {
    ESP_ERROR_CHECK(write_register(UserBank0::USER_CTRL, {1 << 4}));
  }

  esp_err_t ICM20948::reset() {
    const std::vector<uint8_t> RESET_COMMAND = {0x80};
    ESP_ERROR_CHECK(write_register(UserBank0::PWR_MGMT_1, RESET_COMMAND));
    std::array<uint8_t, 1> data;
    for (int i = 0; i < 100; i++) {
      ESP_ERROR_CHECK(read_register(UserBank0::PWR_MGMT_1, data));
      if ((data[0]) == 0x41) {
        return ESP_OK;
      }
    }
    return ESP_FAIL;
  }

  void ICM20948::handle_interrupt() {
    isr_callback_();
  }

  ICM20948Sample ICM20948::read_data() {
    {
      std::array<uint8_t, 2> data;
      ESP_ERROR_CHECK(read_register(UserBank0::INT_STATUS, data));
    }

    std::array<uint8_t, 14> data;
    ESP_ERROR_CHECK(read_register(UserBank0::ACCEL_XOUT_H, data));

    const auto rescale = [&data](const int idx, const float scale) {
                           const int16_t whole_data = (static_cast<int16_t>(data.at(idx)) << 8) | data.at(idx+1);
                           const float fraction = whole_data / 32768.0;
                           return fraction * scale;
                         };

    const float accel_scale = [&]() -> float {
      switch(config_.accel_scale) {
      case ICM20948Config::AccelScale::k2g: return 2.0;
      case ICM20948Config::AccelScale::k4g: return 4.0;
      case ICM20948Config::AccelScale::k8g: return 8.0;
      case ICM20948Config::AccelScale::k16g: return 16.0;
      default: return 0.0;
      }}();
    const float gyro_scale = [&]() -> float {
      switch(config_.gyro_scale) {
      case ICM20948Config::GyroScale::k250_dps: return 250.0;
      case ICM20948Config::GyroScale::k500_dps: return 500.0;
      case ICM20948Config::GyroScale::k1000_dps: return 1000.0;
      case ICM20948Config::GyroScale::k2000_dps: return 2000.0;
      default: return 0.0;
      }}();
    return ICM20948Sample{
      .accel_x_mpss = rescale(0, accel_scale),
      .accel_y_mpss = rescale(2, accel_scale),
      .accel_z_mpss = rescale(4, accel_scale),
      .gyro_x_dps = rescale(6, gyro_scale),
      .gyro_y_dps = rescale(8, gyro_scale),
      .gyro_z_dps = rescale(10, gyro_scale),
      .temp_degC = rescale(12, 1.0),
    };
  }

    template <typename T>
    esp_err_t ICM20948::write_register(const T address, const std::vector<uint8_t> &data) {
      constexpr int bank = UserBanks::template get_index<T>();
      if (bank != active_bank_) {
        ESP_ERROR_CHECK(select_active_bank(device_handle_, bank));
        active_bank_ = bank;
      }
      spi_transaction_t transaction = {
          .flags = 0,
          .cmd = WRITE_COMMAND,
          .addr = +address,
          .length = 8 * data.size(),
          .rxlength = 0,
          .user = nullptr,
          .tx_buffer= data.data(),
          .rx_buffer = nullptr
      };
      return spi_device_transmit(device_handle_, &transaction);
    }

    template <typename T, size_t N>
    esp_err_t ICM20948::read_register(const T address, std::array<uint8_t, N> &data) {
      constexpr int bank = UserBanks::template get_index<T>();
      if (bank != active_bank_) {
        ESP_ERROR_CHECK(select_active_bank(device_handle_, bank));
        active_bank_ = bank;
      }
      spi_transaction_t transaction = {
        .flags = 0,
        .cmd = READ_COMMAND,
        .addr = +address,
        .length = 8 * N,
        .rxlength = 8 * N,
        .user = nullptr,
        .tx_buffer = nullptr,
        .rx_buffer = data.data(),
      };
      return spi_device_transmit(device_handle_, &transaction);
    }
}
