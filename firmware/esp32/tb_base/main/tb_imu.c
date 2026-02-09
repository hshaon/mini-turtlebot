#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/i2c_master.h"

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
static i2c_master_dev_handle_t s_dev = NULL;

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = {reg, val};
  return i2c_master_transmit(s_dev, buf, sizeof(buf), -1);
}

static esp_err_t i2c_read_reg(uint8_t reg, uint8_t *buf, size_t len)
{
  return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, -1);
}

bool tb_imu_is_ready(void)
{
  return s_imu_ok;
}

esp_err_t tb_imu_init(i2c_master_bus_handle_t bus)
{
  if (!bus) return ESP_ERR_INVALID_ARG;

  i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MPU6050_ADDR,
    .scl_speed_hz = 400000,
  };

  esp_err_t r = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
  if (r != ESP_OK) return r;

  uint8_t who = 0;
  r = i2c_read_reg(REG_WHO_AM_I, &who, 1);
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
