#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define TB_LIDAR_POINTS 360

esp_err_t tb_lidar_init(void);
bool tb_lidar_pop_sweep(uint16_t *ranges_mm, size_t count, uint32_t *seq_out);
