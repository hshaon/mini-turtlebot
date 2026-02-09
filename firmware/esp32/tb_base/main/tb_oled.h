#pragma once
#include <stdbool.h>
#include "driver/i2c_master.h"

bool tb_oled_init(i2c_master_bus_handle_t bus);
void tb_oled_clear(void);
void tb_oled_write_line(int line, const char *text);
void tb_oled_show_status(const char *robot_id, const char *ssid, const char *ip);
