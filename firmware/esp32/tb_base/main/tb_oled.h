#pragma once
#include <stdbool.h>

bool tb_oled_init(void);
void tb_oled_clear(void);
void tb_oled_write_line(int line, const char *text);
void tb_oled_show_status(const char *robot_id, const char *ssid, const char *ip);
