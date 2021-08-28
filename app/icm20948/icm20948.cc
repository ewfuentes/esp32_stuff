
#include "icm20948.hh"

#include <iostream>
#include <cstdint>
#include <type_traits>
#include <vector>

namespace app {
  namespace {
    constexpr std::uint8_t READ_COMMAND = 0x01;
    constexpr std::uint8_t WRITE_COMMAND = 0x00;
    enum class UserBank0: uint8_t {
                                   WHO_AM_I = 0x00,
                                   USER_CTRL = 0x01,
                                   REG_BANK_SEL = 0x7F,
    };
    enum class UserBank1: uint8_t {
                                   REG_BANK_SEL = 0x7F,
    };
    enum class UserBank2: uint8_t {
                                   REG_BANK_SEL = 0x7F,
    };
    enum class UserBank3: uint8_t {
                                   REG_BANK_SEL = 0x7F,
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

    template <typename T>
    esp_err_t write_register(const spi_device_handle_t dev_handle, const int active_bank, const T address, const std::vector<uint8_t> &data) {
      constexpr int bank = UserBanks::template get_index<T>();
      if (bank != active_bank) {
        ESP_ERROR_CHECK(select_active_bank(dev_handle, bank));
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
      return spi_device_transmit(dev_handle, &transaction);
    }

    template <typename T>
    esp_err_t read_register(const spi_device_handle_t dev_handle, const int active_bank, const T address, uint8_t &data) {
      constexpr int bank = UserBanks::template get_index<T>();
      if (bank != active_bank) {
        ESP_ERROR_CHECK(select_active_bank(dev_handle, bank));
      }
      spi_transaction_t transaction = {
        .flags = 0,
        .cmd = READ_COMMAND,
        .addr = +address,
        .length = 8,
        .rxlength = 8,
        .user = nullptr,
        .tx_buffer = nullptr,
        .rx_buffer= &data,
      };
      return spi_device_transmit(dev_handle, &transaction);
    }
  }

  ICM20948::ICM20948(const ICM20948Config &config) : config_(config), device_handle_(nullptr), active_bank_(0) {
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
    disable_i2c();
  }

  ICM20948::ICM20948(ICM20948 &&other) : config_(other.config_), device_handle_(other.device_handle_) {
    other.device_handle_ = nullptr;
  }

  ICM20948 &ICM20948::operator=(ICM20948 &&other) {
    config_ = other.config_;
    device_handle_ = other.device_handle_;
    other.device_handle_ = nullptr;
    return *this;
  }

  ICM20948::~ICM20948() {
    if (device_handle_ != nullptr) {
      spi_bus_remove_device(device_handle_);
    }
  }

  int ICM20948::who_am_i(){
    uint8_t out = 0x12;
    ESP_ERROR_CHECK(read_register(device_handle_, active_bank_, UserBank0::WHO_AM_I, out));
    return out;
  }

  void ICM20948::disable_i2c() {
    ESP_ERROR_CHECK(write_register(device_handle_, active_bank_, UserBank0::USER_CTRL, {1 << 4}));
  }
}
