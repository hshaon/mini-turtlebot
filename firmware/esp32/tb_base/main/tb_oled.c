#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "driver/i2c.h"

#include "tb_oled.h"
#include "tb_font6x8.h"
#include "tb_pins"

static const char *TAG = "tb_oled";

#define OLED_ADDR 0x3C
#define OLED_W 128
#define OLED_H 64
#define OLED_PAGES (OLED_H / 8)

static uint8_t fb[OLED_W * OLED_PAGES];

static esp_err_t i2c_write_cmd(uint8_t cmd)
{
  i2c_cmd_handle_t h = i2c_cmd_link_create();
  i2c_master_start(h);
  i2c_master_write_byte(h, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(h, 0x00, true);
  i2c_master_write_byte(h, cmd, true);
  i2c_master_stop(h);
  esp_err_t r = i2c_master_cmd_begin(I2C_NUM_0, h, pdMS_TO_TICKS(50));
  i2c_cmd_link_delete(h);
  return r;
}

static esp_err_t i2c_write_data(const uint8_t *data, size_t len)
{
  i2c_cmd_handle_t h = i2c_cmd_link_create();
  i2c_master_start(h);
  i2c_master_write_byte(h, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(h, 0x40, true);
  i2c_master_write(h, (uint8_t*)data, len, true);
  i2c_master_stop(h);
  esp_err_t r = i2c_master_cmd_begin(I2C_NUM_0, h, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(h);
  return r;
}

static void fb_set_page_col(uint8_t page, uint8_t col)
{
  i2c_write_cmd(0xB0 | (page & 0x07));
  i2c_write_cmd(0x00 | (col & 0x0F));
  i2c_write_cmd(0x10 | ((col >> 4) & 0x0F));
}

static void draw_char_6x8(int x, int page, char c)
{
  if (c < 32 || c > 127) c = '?';
  const uint8_t *g = tb_font6x8[(int)c - 32];
  if (x < 0 || x >= OLED_W) return;
  if (page < 0 || page >= OLED_PAGES) return;

  for (int i = 0; i < 6; i++) {
    int xi = x + i;
    if (xi >= 0 && xi < OLED_W) {
      fb[page * OLED_W + xi] = g[i];
    }
  }
}

static void draw_text_6x8(int x, int page, const char *s)
{
  int cx = x;
  while (*s && cx < OLED_W - 6) {
    draw_char_6x8(cx, page, *s++);
    cx += 6;
  }
}

bool tb_oled_init(void)
{
  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = TB_I2C_SDA_GPIO,
    .scl_io_num = TB_I2C_SCL_GPIO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 400000,
  };
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));

  if (i2c_write_cmd(0xAE) != ESP_OK) {
    ESP_LOGW(TAG, "OLED not responding at 0x3C (check wiring/address)");
    return false;
  }
  i2c_write_cmd(0xD5); i2c_write_cmd(0x80);
  i2c_write_cmd(0xA8); i2c_write_cmd(0x3F);
  i2c_write_cmd(0xD3); i2c_write_cmd(0x00);
  i2c_write_cmd(0x40);
  i2c_write_cmd(0x8D); i2c_write_cmd(0x14);
  i2c_write_cmd(0x20); i2c_write_cmd(0x00);
  // Rotate 180 degrees: segment remap and COM scan direction.
  i2c_write_cmd(0xA0);
  i2c_write_cmd(0xC0);
  i2c_write_cmd(0xDA); i2c_write_cmd(0x12);
  i2c_write_cmd(0x81); i2c_write_cmd(0x7F);
  i2c_write_cmd(0xD9); i2c_write_cmd(0xF1);
  i2c_write_cmd(0xDB); i2c_write_cmd(0x40);
  i2c_write_cmd(0xA4);
  i2c_write_cmd(0xA6);
  i2c_write_cmd(0xAF);

  tb_oled_clear();
  return true;
}

void tb_oled_clear(void)
{
  memset(fb, 0x00, sizeof(fb));
  for (int p = 0; p < OLED_PAGES; p++) {
    fb_set_page_col((uint8_t)p, 0);
    i2c_write_data(&fb[p * OLED_W], OLED_W);
  }
}

void tb_oled_write_line(int line, const char *text)
{
  if (line < 0) line = 0;
  if (line >= OLED_PAGES) line = OLED_PAGES - 1;

  memset(&fb[line * OLED_W], 0x00, OLED_W);
  draw_text_6x8(0, line, text);

  fb_set_page_col((uint8_t)line, 0);
  i2c_write_data(&fb[line * OLED_W], OLED_W);
}

void tb_oled_show_status(const char *robot_id, const char *ssid, const char *ip)
{
  char l1[32];
  char l2[32];
  char l3[32];

  snprintf(l1, sizeof(l1), "ID: %s", robot_id ? robot_id : "---");
  snprintf(l2, sizeof(l2), "SSID: %s", ssid ? ssid : "---");
  snprintf(l3, sizeof(l3), "IP: %s", ip ? ip : "---");

  tb_oled_write_line(0, l1);
  tb_oled_write_line(1, l2);
  tb_oled_write_line(2, l3);
}
