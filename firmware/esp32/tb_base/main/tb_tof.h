#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t tb_tof_init(void);
esp_err_t tb_tof_read(float *range_m);
bool tb_tof_is_ready(void);
