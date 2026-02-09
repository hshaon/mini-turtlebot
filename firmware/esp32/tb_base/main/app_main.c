#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tb_wifi.h"
#include "tb_oled.h"
#include "tb_motors.h"
#include "tb_i2c.h"
#include "tb_imu.h"
#include "tb_ir.h"
#include "tb_tof.h"

// forward declare your TCP server task from your file
void tcp_server_task(void *arg);

static const char *TAG = "tb_main";

void app_main(void)
{
  ESP_LOGI(TAG, "boot");

  if (tb_wifi_init_and_connect() != ESP_OK) {
    ESP_LOGE(TAG, "wifi connect failed; rebooting in 3s");
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
  }

  if (tb_i2c_init() != ESP_OK) {
    ESP_LOGE(TAG, "i2c init failed; rebooting in 3s");
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
  }

  if (tb_oled_init(tb_i2c_get_bus())) {
    tb_oled_show_status("---", CONFIG_TB_WIFI_SSID, tb_wifi_get_ip_str());
  }

  tb_motors_init();

  if (tb_imu_init(tb_i2c_get_bus()) != ESP_OK) {
    ESP_LOGW(TAG, "IMU init failed");
  }
  if (tb_ir_init() != ESP_OK) {
    ESP_LOGW(TAG, "IR init failed");
  }
  if (tb_tof_init(tb_i2c_get_bus()) != ESP_OK) {
    ESP_LOGW(TAG, "ToF init failed");
  }

  xTaskCreatePinnedToCore(tcp_server_task, "tb_tcp_server", 8192, NULL, 5, NULL, 1);
}
