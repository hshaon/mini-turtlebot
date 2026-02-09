#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

esp_err_t tb_i2c_init(void);
i2c_master_bus_handle_t tb_i2c_get_bus(void);
