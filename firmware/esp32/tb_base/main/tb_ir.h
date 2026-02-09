#pragma once

#include "esp_err.h"

esp_err_t tb_ir_init(void);
esp_err_t tb_ir_read(float *left, float *right);
