#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"

#include "tb_lidar.h"
#include "tb_pins"

static const char *TAG = "tb_lidar";

#define LIDAR_UART UART_NUM_1
#define LIDAR_BAUD 230400
#define LIDAR_FRAME_SIZE 47
#define LIDAR_HEADER 0x54
#define LIDAR_VERLEN 0x2C

static uint16_t s_ranges[TB_LIDAR_POINTS];
static uint32_t s_seq = 0;
static bool s_ready = false;
static SemaphoreHandle_t s_mtx = NULL;
static float s_last_end_angle = 0.0f;

static uint16_t read_u16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void process_frame(const uint8_t *frame)
{
  float start_angle = (float)read_u16(&frame[4]) / 100.0f;
  float end_angle = (float)read_u16(&frame[42]) / 100.0f;

  float diff = end_angle - start_angle;
  if (diff < 0.0f) diff += 360.0f;
  float step = diff / 11.0f;

  for (int i = 0; i < 12; ++i) {
    float angle = start_angle + step * (float)i;
    if (angle >= 360.0f) angle -= 360.0f;
    int idx = (int)(angle + 0.5f) % TB_LIDAR_POINTS;
    uint16_t dist = read_u16(&frame[6 + i * 3]);
    s_ranges[idx] = dist;
  }

  if (end_angle < s_last_end_angle && s_last_end_angle > 0.0f) {
    s_seq++;
    s_ready = true;
  }
  s_last_end_angle = end_angle;
}

static void lidar_task(void *arg)
{
  (void)arg;

  uint8_t stream[256];
  size_t stream_len = 0;

  while (1) {
    uint8_t rx[128];
    int r = uart_read_bytes(LIDAR_UART, rx, sizeof(rx), pdMS_TO_TICKS(10));
    if (r <= 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    if (stream_len + (size_t)r > sizeof(stream)) {
      stream_len = 0;
    }
    memcpy(stream + stream_len, rx, (size_t)r);
    stream_len += (size_t)r;

    while (stream_len >= 2) {
      size_t idx = 0;
      bool found = false;
      for (size_t i = 0; i + 1 < stream_len; ++i) {
        if (stream[i] == LIDAR_HEADER && stream[i + 1] == LIDAR_VERLEN) {
          idx = i;
          found = true;
          break;
        }
      }

      if (!found) {
        stream_len = 0;
        break;
      }

      if (idx > 0) {
        memmove(stream, stream + idx, stream_len - idx);
        stream_len -= idx;
      }

      if (stream_len < LIDAR_FRAME_SIZE) break;

      if (s_mtx && xSemaphoreTake(s_mtx, pdMS_TO_TICKS(5)) == pdTRUE) {
        process_frame(stream);
        xSemaphoreGive(s_mtx);
      }

      memmove(stream, stream + LIDAR_FRAME_SIZE, stream_len - LIDAR_FRAME_SIZE);
      stream_len -= LIDAR_FRAME_SIZE;
    }
  }
}


esp_err_t tb_lidar_init(void)
{
  if (!s_mtx) s_mtx = xSemaphoreCreateMutex();
  memset(s_ranges, 0, sizeof(s_ranges));
  if (!s_mtx) return ESP_ERR_NO_MEM;

  uart_config_t cfg = {
    .baud_rate = LIDAR_BAUD,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };

  ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART, 2048, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(LIDAR_UART, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART, TB_UART_TX_GPIO, TB_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  xTaskCreatePinnedToCore(lidar_task, "tb_lidar", 4096, NULL, 4, NULL, 0);
  ESP_LOGI(TAG, "LD19P UART ready");
  return ESP_OK;
}

bool tb_lidar_pop_sweep(uint16_t *ranges_mm, size_t count, uint32_t *seq_out)
{
  if (!ranges_mm || count < TB_LIDAR_POINTS) return false;
  if (!s_mtx) return false;

  bool ok = false;
  if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(5)) == pdTRUE) {
    if (s_ready) {
      memcpy(ranges_mm, s_ranges, TB_LIDAR_POINTS * sizeof(uint16_t));
      if (seq_out) *seq_out = s_seq;
      s_ready = false;
      ok = true;
    }
    xSemaphoreGive(s_mtx);
  }
  return ok;
}
