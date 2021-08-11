/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "subway_ticker/font.hh"
#include <stdio.h>

static const char *TAG = "example";
constexpr gpio_num_t BLINK_GPIO = GPIO_NUM_13;
constexpr gpio_num_t DISPLAY_SDA = GPIO_NUM_23;
constexpr gpio_num_t DISPLAY_SCL = GPIO_NUM_22;
constexpr i2c_port_t DISPLAY_I2C = 0;
constexpr uint8_t DISPLAY_ADDR = 0x3D;

constexpr uint8_t DATA_BYTES = 0x00;
constexpr uint8_t COMMAND_BYTES = 0x00;

static bool s_led_state = 0;

static void blink_led(void) {
  /* Set the GPIO level according to the state (LOW or HIGH)*/
  gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void) {
  ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
  gpio_reset_pin(BLINK_GPIO);
  /* Set the GPIO as a push/pull output */
  gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

esp_err_t set_charge_pump(bool is_on) {
  const uint8_t CHARGE_PUMP_COMMAND = 0x8D;
  const uint8_t CHARGE_PUMP_SETTING = is_on ? 0x14 : 0x10;
  const uint8_t buf[] = {COMMAND_BYTES, CHARGE_PUMP_COMMAND,
                         CHARGE_PUMP_SETTING};
  return i2c_master_write_to_device(DISPLAY_I2C, DISPLAY_ADDR, buf, sizeof(buf),
                                    CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
}

esp_err_t set_display(bool is_on) {
  const uint8_t DISPLAY_OFF = 0xAE;
  const uint8_t DISPLAY_ON = 0xAF;
  const uint8_t buf[] = {COMMAND_BYTES, is_on ? DISPLAY_ON : DISPLAY_OFF};
  return i2c_master_write_to_device(DISPLAY_I2C, DISPLAY_ADDR, buf, sizeof(buf),
                                    CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
}

esp_err_t set_display_all_on(bool is_on) {
  constexpr uint8_t DISPLAY_OFF = 0xA4;
  constexpr uint8_t DISPLAY_ON = 0xA5;
  const uint8_t buf[] = {COMMAND_BYTES, is_on ? DISPLAY_ON : DISPLAY_OFF};
  return i2c_master_write_to_device(DISPLAY_I2C, DISPLAY_ADDR, buf, sizeof(buf),
                                    CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
}

esp_err_t set_contrast(uint8_t contrast) {
  constexpr uint8_t CONTRAST_CONTROL_COMMAND = 0x81;
  const uint8_t buf[] = {COMMAND_BYTES, CONTRAST_CONTROL_COMMAND, contrast};
  return i2c_master_write_to_device(DISPLAY_I2C, DISPLAY_ADDR, buf, sizeof(buf),
                                    CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
}

esp_err_t write_stripes() {
  constexpr uint8_t PAGE_ADDRESS_COMMAND = 0xB0;
  uint8_t page_address_buf[] = {COMMAND_BYTES, PAGE_ADDRESS_COMMAND};

  uint8_t buf[129] = {0};
  buf[0] = 0x40;
  for (int i = 0; i < 8; i++) {
    for (int j = 1; j < sizeof(buf); j++) {
      if (i == 0) {
        buf[j] = 0x1;
      } else if (i == 7) {
        buf[j] = 0x80;
      } else {
        buf[j] = 0x00;
      }
      if (j == 1 || j == 128) {
        buf[j] = 0xFF;
      }
    }
    // Set page address
    page_address_buf[1] = (page_address_buf[1] & 0xF0) | i;
    i2c_master_write_to_device(DISPLAY_I2C, DISPLAY_ADDR, page_address_buf,
                               sizeof(page_address_buf),
                               CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);

    i2c_master_write_to_device(DISPLAY_I2C, DISPLAY_ADDR, buf, sizeof(buf),
                               CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
  }
  return ESP_OK;
}

static void configure_i2c() {
  const i2c_config_t config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = DISPLAY_SDA,
      .scl_io_num = DISPLAY_SCL,
      .sda_pullup_en = false,
      .scl_pullup_en = false,
      .master = {.clk_speed = 400000},
      .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
  };

  constexpr size_t IGNORE = 0;
  {
    const esp_err_t status = i2c_param_config(DISPLAY_I2C, &config);
    if (status) {
      ESP_LOGI(TAG, "Failed to config i2c!");
    }
  }
  {
    const esp_err_t status =
        i2c_driver_install(DISPLAY_I2C, I2C_MODE_MASTER, IGNORE, IGNORE, 0);
    if (status) {
      ESP_LOGI(TAG, "Failed to install i2c!");
    }
  }
  ESP_LOGI(TAG, "Done configuring I2C!");
}

extern "C" void app_main(void) {

  /* Configure the peripheral according to the LED type */
  configure_led();
  configure_i2c();
  const app::font::Font &font = app::font::subway_ticker::get_font();

  {
    const esp_err_t status = set_charge_pump(true);
    ESP_LOGI(TAG, "charge pump on status: %d", status);
    vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
  }
  {
    const esp_err_t status = set_display(true);
    ESP_LOGI(TAG, "Display on status: %d", status);
    vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
  }
  {
    const esp_err_t status = set_contrast(255u);
    ESP_LOGI(TAG, "contrast status: %d", status);
    vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
  }

  while (1) {
    ESP_LOGI(TAG, "Turning the LED %s!", s_led_state ? "ON" : "OFF");
    blink_led();
    write_stripes();
    /* Toggle the LED state */
    s_led_state = !s_led_state;
    vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
  }
}
