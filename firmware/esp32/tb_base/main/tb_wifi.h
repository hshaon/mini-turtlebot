#pragma once
#include "esp_err.h"
esp_err_t tb_wifi_init_and_connect(void);
const char* tb_wifi_get_ip_str(void);
