#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/i2c.h"

#include "tb_imu.h"

static const char *TAG = "tb_imu";

#define MPU6050_ADDR 0x68

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

static bool s_imu_ok = false;

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t val)
{
  i2c_cmd_handle_t h = i2c_cmd_link_create();
  i2c_master_start(h);
  i2c_master_write_byte(h, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
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
  i2c_master_write_byte(h, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(h, reg, true);
  i2c_master_start(h);
  i2c_master_write_byte(h, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
  if (len > 1) {
    i2c_master_read(h, buf, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(h, buf + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(h);
  esp_err_t r = i2c_master_cmd_begin(I2C_NUM_0, h, pdMS_TO_TICKS(50));
  i2c_cmd_link_delete(h);
  return r;
}

bool tb_imu_is_ready(void)
{
  return s_imu_ok;
}

esp_err_t tb_imu_init(void)
{
  uint8_t who = 0;
  esp_err_t r = i2c_read_reg(REG_WHO_AM_I, &who, 1);
  if (r != ESP_OK) {
    ESP_LOGW(TAG, "WHO_AM_I read failed");
  } else {
    ESP_LOGI(TAG, "WHO_AM_I=0x%02x", (unsigned)who);
  }

  if ((r = i2c_write_reg(REG_PWR_MGMT_1, 0x00)) != ESP_OK) return r;
  vTaskDelay(pdMS_TO_TICKS(50));

  if ((r = i2c_write_reg(REG_SMPLRT_DIV, 0x04)) != ESP_OK) return r; // ~200 Hz
  if ((r = i2c_write_reg(REG_CONFIG, 0x03)) != ESP_OK) return r;     // DLPF
  if ((r = i2c_write_reg(REG_GYRO_CONFIG, 0x00)) != ESP_OK) return r; // 250 dps
  if ((r = i2c_write_reg(REG_ACCEL_CONFIG, 0x00)) != ESP_OK) return r; // 2 g

  s_imu_ok = true;
  ESP_LOGI(TAG, "MPU6050 init ok");
  return ESP_OK;
}

esp_err_t tb_imu_read(tb_imu_raw_t *out)
{
  if (!out) return ESP_ERR_INVALID_ARG;
  if (!s_imu_ok) return ESP_ERR_INVALID_STATE;

  uint8_t buf[14];
  esp_err_t r = i2c_read_reg(REG_ACCEL_XOUT_H, buf, sizeof(buf));
  if (r != ESP_OK) return r;

  int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
  int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
  int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
  int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

  const float accel_scale = 9.80665f / 16384.0f;
  const float gyro_scale = 0.01745329252f / 131.0f;

  out->ax = (float)ax * accel_scale;
  out->ay = (float)ay * accel_scale;
  out->az = (float)az * accel_scale;
  out->gx = (float)gx * gyro_scale;
  out->gy = (float)gy * gyro_scale;
  out->gz = (float)gz * gyro_scale;

  return ESP_OK;
}
