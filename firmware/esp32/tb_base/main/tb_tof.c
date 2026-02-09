#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/i2c.h"

#include "tb_tof.h"

static const char *TAG = "tb_tof";

#define VL53L0X_ADDR 0x29

#define REG_SYSRANGE_START            0x00
#define REG_SYSTEM_INTERRUPT_CLEAR    0x0B
#define REG_RESULT_INTERRUPT_STATUS   0x13
#define REG_RESULT_RANGE_STATUS       0x14
#define REG_IDENTIFICATION_MODEL_ID   0xC0

static bool s_tof_ok = false;
static uint8_t s_stop_variable = 0;

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t val)
{
  i2c_cmd_handle_t h = i2c_cmd_link_create();
  i2c_master_start(h);
  i2c_master_write_byte(h, (VL53L0X_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(h, reg, true);
  i2c_master_write_byte(h, val, true);
  i2c_master_stop(h);
  esp_err_t r = i2c_master_cmd_begin(I2C_NUM_0, h, pdMS_TO_TICKS(50));
  i2c_cmd_link_delete(h);
  return r;
}

static esp_err_t i2c_read_reg(uint8_t reg, uint8_t *buf, size_t len)
{
  i2c_cmd_handle_t h = i2c_cmd_link_create();
  i2c_master_start(h);
  i2c_master_write_byte(h, (VL53L0X_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(h, reg, true);
  i2c_master_start(h);
  i2c_master_write_byte(h, (VL53L0X_ADDR << 1) | I2C_MASTER_READ, true);
  if (len > 1) {
    i2c_master_read(h, buf, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(h, buf + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(h);
  esp_err_t r = i2c_master_cmd_begin(I2C_NUM_0, h, pdMS_TO_TICKS(50));
  i2c_cmd_link_delete(h);
  return r;
}

bool tb_tof_is_ready(void)
{
  return s_tof_ok;
}

esp_err_t tb_tof_init(void)
{
  uint8_t model = 0;
  esp_err_t r = i2c_read_reg(REG_IDENTIFICATION_MODEL_ID, &model, 1);
  if (r != ESP_OK) {
    ESP_LOGW(TAG, "model id read failed");
  } else {
    ESP_LOGI(TAG, "model id=0x%02x", (unsigned)model);
  }

  // Basic init sequence (minimal)
  (void)i2c_write_reg(0x88, 0x00);
  (void)i2c_write_reg(0x80, 0x01);
  (void)i2c_write_reg(0xFF, 0x01);
  (void)i2c_write_reg(0x00, 0x00);
  (void)i2c_read_reg(0x91, &s_stop_variable, 1);
  (void)i2c_write_reg(0x00, 0x01);
  (void)i2c_write_reg(0xFF, 0x00);
  (void)i2c_write_reg(0x80, 0x00);

  s_tof_ok = true;
  ESP_LOGI(TAG, "VL53L0X init ok (minimal)");
  return ESP_OK;
}

esp_err_t tb_tof_read(float *range_m)
{
  if (!range_m) return ESP_ERR_INVALID_ARG;
  if (!s_tof_ok) return ESP_ERR_INVALID_STATE;

  // Start single ranging
  (void)i2c_write_reg(0x80, 0x01);
  (void)i2c_write_reg(0xFF, 0x01);
  (void)i2c_write_reg(0x00, 0x00);
  (void)i2c_write_reg(0x91, s_stop_variable);
  (void)i2c_write_reg(0x00, 0x01);
  (void)i2c_write_reg(0xFF, 0x00);
  (void)i2c_write_reg(0x80, 0x00);
  (void)i2c_write_reg(REG_SYSRANGE_START, 0x01);

  const TickType_t start = xTaskGetTickCount();
  while (1) {
    uint8_t status = 0;
    esp_err_t r = i2c_read_reg(REG_RESULT_INTERRUPT_STATUS, &status, 1);
    if (r != ESP_OK) return r;
    if (status & 0x07) break;
    if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(50)) {
      return ESP_ERR_TIMEOUT;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  uint8_t buf[2];
  esp_err_t r = i2c_read_reg(REG_RESULT_RANGE_STATUS + 10, buf, sizeof(buf));
  if (r != ESP_OK) return r;

  uint16_t mm = (uint16_t)buf[0] << 8 | buf[1];
  *range_m = (float)mm / 1000.0f;

  (void)i2c_write_reg(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
  return ESP_OK;
}
