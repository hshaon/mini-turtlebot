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
#include "tb_lidar.h"
#include "esp_system.h"

// forward declare your TCP server task from your file
void tcp_server_task(void *arg);

static const char *TAG = "tb_main";

static const char *reset_reason_str(esp_reset_reason_t r)
{
  switch (r) {
    case ESP_RST_UNKNOWN:    return "UNKNOWN";
    case ESP_RST_POWERON:    return "POWERON";
    case ESP_RST_EXT:        return "EXT";
    case ESP_RST_SW:         return "SW";
    case ESP_RST_PANIC:      return "PANIC";
    case ESP_RST_INT_WDT:    return "INT_WDT";
    case ESP_RST_TASK_WDT:   return "TASK_WDT";
    case ESP_RST_WDT:        return "WDT";
    case ESP_RST_DEEPSLEEP:  return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:   return "BROWNOUT";
    case ESP_RST_SDIO:       return "SDIO";
    case ESP_RST_USB:        return "USB";
    case ESP_RST_JTAG:       return "JTAG";
    case ESP_RST_EFUSE:      return "EFUSE";
    case ESP_RST_PWR_GLITCH: return "PWR_GLITCH";
    case ESP_RST_CPU_LOCKUP: return "CPU_LOCKUP";
    default:                 return "UNK";
  }
}

void app_main(void)
{
  ESP_LOGI(TAG, "boot");
  const esp_reset_reason_t reset_reason = esp_reset_reason();
  ESP_LOGI(TAG, "reset_reason=%d", (int)reset_reason);

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
    char line[22];
    snprintf(line, sizeof(line), "RST: %s", reset_reason_str(reset_reason));
    tb_oled_write_line(3, line);
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
  if (tb_lidar_init() != ESP_OK) {
    ESP_LOGW(TAG, "LiDAR init failed");
  }


  xTaskCreatePinnedToCore(tcp_server_task, "tb_tcp_server", 8192, NULL, 5, NULL, 1);
}
