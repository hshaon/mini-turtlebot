#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

esp_err_t tb_tof_init(i2c_master_bus_handle_t bus);
esp_err_t tb_tof_read(float *range_m);
bool tb_tof_is_ready(void);
