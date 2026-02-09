#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

typedef struct {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
} tb_imu_raw_t;

esp_err_t tb_imu_init(i2c_master_bus_handle_t bus);
esp_err_t tb_imu_read(tb_imu_raw_t *out);
bool tb_imu_is_ready(void);
