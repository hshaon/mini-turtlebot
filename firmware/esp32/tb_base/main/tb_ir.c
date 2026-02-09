#include <string.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

#include "tb_ir.h"
#include "tb_pins"

static const char *TAG = "tb_ir";

static adc_oneshot_unit_handle_t s_unit = NULL;
static adc_channel_t s_chan_left;
static adc_channel_t s_chan_right;
static bool s_ready = false;

esp_err_t tb_ir_init(void)
{
  adc_unit_t unit_left = ADC_UNIT_1;
  adc_unit_t unit_right = ADC_UNIT_1;

  esp_err_t r = adc_oneshot_io_to_channel(TB_IRD1_GPIO, &unit_left, &s_chan_left);
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "io_to_channel left failed: %d", (int)r);
    return r;
  }

  r = adc_oneshot_io_to_channel(TB_IRD2_GPIO, &unit_right, &s_chan_right);
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "io_to_channel right failed: %d", (int)r);
    return r;
  }

  if (unit_left != unit_right) {
    ESP_LOGE(TAG, "IR sensors on different ADC units");
    return ESP_ERR_INVALID_STATE;
  }

  adc_oneshot_unit_init_cfg_t cfg = {
    .unit_id = unit_left,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  r = adc_oneshot_new_unit(&cfg, &s_unit);
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %d", (int)r);
    return r;
  }

  adc_oneshot_chan_cfg_t chan_cfg = {
    .atten = ADC_ATTEN_DB_11,
    .bitwidth = ADC_BITWIDTH_12,
  };
  r = adc_oneshot_config_channel(s_unit, s_chan_left, &chan_cfg);
  if (r != ESP_OK) return r;
  r = adc_oneshot_config_channel(s_unit, s_chan_right, &chan_cfg);
  if (r != ESP_OK) return r;

  s_ready = true;
  ESP_LOGI(TAG, "IR ADC init ok");
  return ESP_OK;
}

static float normalize_adc(int raw)
{
  if (raw < 0) raw = 0;
  if (raw > 4095) raw = 4095;
  return (float)raw / 4095.0f;
}

esp_err_t tb_ir_read(float *left, float *right)
{
  if (!left || !right) return ESP_ERR_INVALID_ARG;
  if (!s_ready) return ESP_ERR_INVALID_STATE;

  int l = 0;
  int r = 0;
  esp_err_t err = adc_oneshot_read(s_unit, s_chan_left, &l);
  if (err != ESP_OK) return err;
  err = adc_oneshot_read(s_unit, s_chan_right, &r);
  if (err != ESP_OK) return err;

  *left = normalize_adc(l);
  *right = normalize_adc(r);

  return ESP_OK;
}
