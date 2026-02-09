#include "tb_i2c.h"

#include "tb_pins"

static i2c_master_bus_handle_t s_bus = NULL;

esp_err_t tb_i2c_init(void)
{
  if (s_bus) return ESP_OK;

  i2c_master_bus_config_t cfg = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = TB_I2C_SDA_GPIO,
    .scl_io_num = TB_I2C_SCL_GPIO,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags = {
      .enable_internal_pullup = true,
    },
  };

  return i2c_new_master_bus(&cfg, &s_bus);
}

i2c_master_bus_handle_t tb_i2c_get_bus(void)
{
  return s_bus;
}
