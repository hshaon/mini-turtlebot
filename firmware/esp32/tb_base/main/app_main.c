#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tb_wifi.h"
#include "tb_motors.h"

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

  tb_motors_init();

  xTaskCreatePinnedToCore(tcp_server_task, "tb_tcp_server", 8192, NULL, 5, NULL, 1);
}
