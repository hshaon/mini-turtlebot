#include "esp_log.h"
#include "esp_err.h"

#include "tb_tof.h"
#include "vl53l0x.h"

static const char *TAG = "tb_tof";

static vl53l0x_t *s_dev = NULL;
static bool s_tof_ok = false;

static const uint8_t k_address = 0x29;

bool tb_tof_is_ready(void)
{
  return s_tof_ok;
}

esp_err_t tb_tof_init(i2c_master_bus_handle_t bus)
{
  if (!bus) return ESP_ERR_INVALID_ARG;

  s_dev = vl53l0x_config_with_bus(bus, -1, k_address, 0);
  if (!s_dev) {
    ESP_LOGW(TAG, "vl53l0x_config_with_bus failed");
    return ESP_FAIL;
  }

  const char *err = vl53l0x_init(s_dev);
  if (err) {
    ESP_LOGW(TAG, "vl53l0x_init failed: %s", err);
    return ESP_FAIL;
  }

  (void)vl53l0x_setMeasurementTimingBudget(s_dev, 33000);
  (void)vl53l0x_setSignalRateLimit(s_dev, 0.25f);

  s_tof_ok = true;
  ESP_LOGI(TAG, "VL53L0X init ok");
  return ESP_OK;
}

esp_err_t tb_tof_read(float *range_m)
{
  if (!range_m) return ESP_ERR_INVALID_ARG;
  if (!s_tof_ok || !s_dev) return ESP_ERR_INVALID_STATE;

  uint16_t mm = vl53l0x_readRangeSingleMillimeters(s_dev);
  if (vl53l0x_timeoutOccurred(s_dev) || vl53l0x_i2cFail(s_dev)) {
    return ESP_FAIL;
  }

  *range_m = (float)mm / 1000.0f;
  return ESP_OK;
}
