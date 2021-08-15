/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <array>
#include <sstream>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "subway_ticker/font.hh"

static char HELLO_HTML[] =
    "<html><head><title>Hello ESP32</title></head>"
    "<body>Hello to you too!"
    "<form method=\"post\"><label for=\"message\">message:</label><br>"
    "<input type=\"text\" id=\"message\" name=\"message\"><br>"
    "<input type=\"submit\" value=\"Let's see that puppy\">"
    "</form></body>"
    "</html>";

static const char *TAG = "example";
constexpr gpio_num_t BLINK_GPIO = GPIO_NUM_13;
constexpr gpio_num_t DISPLAY_SDA = GPIO_NUM_23;
constexpr gpio_num_t DISPLAY_SCL = GPIO_NUM_22;
constexpr i2c_port_t DISPLAY_I2C = 0;
constexpr uint8_t DISPLAY_ADDR = 0x3D;

constexpr uint8_t DATA_BYTES = 0x00;
constexpr uint8_t COMMAND_BYTES = 0x00;

static bool s_led_state = 0;

std::string display_message = "my cool display message!";
static std::array<std::string, 8> display_data;

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

std::array<std::uint8_t, 128> data_from_string(const std::string &str) {
  int idx = 0;
  std::array<std::uint8_t, 128> out{0};
  const app::font::Font &font = app::font::subway_ticker::get_font();
  for (const char c : str) {
    if (font.glyph_from_codepoint.find(c) == font.glyph_from_codepoint.end()) {
      ESP_LOGI(TAG, "Unable to find char in font: %d", c);
      continue;
    }
    for (const std::uint8_t b : font.glyph_from_codepoint.at(c).data) {
      if (idx >= 128) {
        break;
      }
      out[idx] = b;
      idx++;
    }
    idx += 1;
  }
  return out;
}

esp_err_t write_stripes(const std::string &ip_address) {
  constexpr uint8_t PAGE_ADDRESS_COMMAND = 0xB0;
  uint8_t page_address_buf[] = {COMMAND_BYTES, PAGE_ADDRESS_COMMAND};

  uint8_t buf[129] = {0};
  buf[0] = 0x40;
  std::stringstream sstream;
  for (int i = 0; i < 8; i++) {
    sstream.str("");
    if (s_led_state == i % 2) {
      sstream << display_message;
    }
    if (i == 0) {
      sstream.str("");
      sstream << "IP: " << ip_address;
    }
    const std::array<uint8_t, 128> data = data_from_string(sstream.str());
    for (int j = 0; j < data.size(); j++) {
      buf[j + 1] = data[j];
    }

    // Set page address
    page_address_buf[1] = (page_address_buf[1] & 0xF0) | i;
    i2c_master_write_to_device(DISPLAY_I2C, DISPLAY_ADDR, page_address_buf,
                               sizeof(page_address_buf),
                               100 / portTICK_PERIOD_MS);

    i2c_master_write_to_device(DISPLAY_I2C, DISPLAY_ADDR, buf, sizeof(buf),
                               100 / portTICK_PERIOD_MS);
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

static esp_err_t root_handler(httpd_req_t *req) {

  ESP_LOGI(TAG, "Received root request. method: %d length: %d", req->method, req->content_len);
  if (req->method == HTTP_POST && req->content_len > 0) {
    char *content = (char *)calloc(req->content_len+1, sizeof(char));
    httpd_req_recv(req, content, req->content_len);
    ESP_LOGI(TAG, "Received POST request: \r\n%s", content);
    char message_str[64] = {0};
    ESP_ERROR_CHECK(httpd_query_key_value(content, "message", message_str,
                                          sizeof(message_str)));
    if (strlen(message_str)) {
      display_message = message_str;
    }
    free(content);
  }
  httpd_resp_set_status(req, HTTPD_200);
  httpd_resp_sendstr(req, HELLO_HTML);
  return ESP_OK;
}

static const httpd_uri_t root_get = {
    .uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = NULL};

static const httpd_uri_t root_post = {
    .uri = "/", .method = HTTP_POST, .handler = root_handler, .user_ctx = NULL};

static httpd_handle_t start_webserver(void) {
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.lru_purge_enable = true;

  // Start the httpd server
  ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
  if (httpd_start(&server, &config) == ESP_OK) {
    // Set URI handlers
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &root_get);
    httpd_register_uri_handler(server, &root_post);
    //    httpd_register_uri_handler(server, &echo);
    //    httpd_register_uri_handler(server, &ctrl);
    //#if CONFIG_EXAMPLE_BASIC_AUTH
    //    httpd_register_basic_auth(server);
    //#endif
    return server;
  }

  ESP_LOGI(TAG, "Error starting server!");
  return NULL;
}

std::string get_ip_address() {
  const int num_ifs = esp_netif_get_nr_of_ifs();
  esp_netif_t *netif = nullptr;
  esp_netif_ip_info_t ip_info;
  for (int i = 0; i < num_ifs; i++) {
    netif = esp_netif_next(netif);
    if (esp_netif_is_netif_up(netif)) {
      ESP_ERROR_CHECK(esp_netif_get_ip_info(netif, &ip_info));
      std::stringstream oss;
      for (int j = 0; j < 4; j++) {
        if (j > 0) {
          oss << ".";
        }
        const int curr_byte = (ip_info.ip.addr >> (8 * j)) & 0xFF;
        oss << curr_byte;
      }
      return oss.str();
    }
  }
  return "Unknown IP";
}

extern "C" void app_main(void) {

  static httpd_handle_t server = NULL;
  /* Configure the peripheral according to the LED type */
  configure_led();
  configure_i2c();

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  ESP_ERROR_CHECK(example_connect());

  {
    const esp_err_t status = set_display(false);
    ESP_LOGI(TAG, "Display off status: %d", status);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  {
    const esp_err_t status = set_charge_pump(true);
    ESP_LOGI(TAG, "charge pump on status: %d", status);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  {
    const esp_err_t status = set_display(true);
    ESP_LOGI(TAG, "Display on status: %d", status);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  {
    const esp_err_t status = set_contrast(255u);
    ESP_LOGI(TAG, "contrast status: %d", status);
  }

  server = start_webserver();
  const std::string ip_address = get_ip_address();

  while (1) {
    blink_led();
    write_stripes(ip_address);
    /* Toggle the LED state */
    s_led_state = !s_led_state;
    vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
  }
}
