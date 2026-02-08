#include <math.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tb_pins"
#include "tb_motors.h"

static const char *TAG = "tb_motors";

#define TB_MOTOR_PWM_HZ       20000
#define TB_MOTOR_PWM_RES      LEDC_TIMER_10_BIT
#define TB_MOTOR_PWM_MAX_DUTY ((1 << 10) - 1)

#define TB_CH_A1 LEDC_CHANNEL_0
#define TB_CH_A2 LEDC_CHANNEL_1
#define TB_CH_B1 LEDC_CHANNEL_2
#define TB_CH_B2 LEDC_CHANNEL_3

// Set to 1 to flip motor direction if wiring is reversed.
#define TB_MOTOR_INVERT_LEFT  0
#define TB_MOTOR_INVERT_RIGHT 1

#define TB_MOTOR_WATCHDOG_MS 300
#define TB_MOTOR_WATCHDOG_POLL_MS 50

static int64_t s_last_cmd_us = 0;

static inline uint32_t duty_from_unit(float u) {
  if (u < 0.0f) u = 0.0f;
  if (u > 1.0f) u = 1.0f;
  return (uint32_t)lroundf(u * TB_MOTOR_PWM_MAX_DUTY);
}

static void ledc_apply(ledc_channel_t ch, uint32_t duty) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

static void set_motor_inin(uint32_t duty, int dir, ledc_channel_t in1, ledc_channel_t in2)
{
  if (dir > 0) {
    ledc_apply(in1, duty);
    ledc_apply(in2, 0);
  } else if (dir < 0) {
    ledc_apply(in1, 0);
    ledc_apply(in2, duty);
  } else {
    ledc_apply(in1, 0);
    ledc_apply(in2, 0);
  }
}

static void motors_apply(float speed_left, float speed_right)
{
  if (TB_MOTOR_INVERT_LEFT)  speed_left = -speed_left;
  if (TB_MOTOR_INVERT_RIGHT) speed_right = -speed_right;

  if (speed_left > 1.0f) speed_left = 1.0f;
  if (speed_left < -1.0f) speed_left = -1.0f;
  if (speed_right > 1.0f) speed_right = 1.0f;
  if (speed_right < -1.0f) speed_right = -1.0f;

  int dir_l = (speed_left > 0.001f) ? 1 : (speed_left < -0.001f) ? -1 : 0;
  int dir_r = (speed_right > 0.001f) ? 1 : (speed_right < -0.001f) ? -1 : 0;

  uint32_t duty_l = duty_from_unit(fabsf(speed_left));
  uint32_t duty_r = duty_from_unit(fabsf(speed_right));

  // A = left, B = right (swap here if your wiring is reversed).
  set_motor_inin(duty_l, dir_l, TB_CH_A1, TB_CH_A2);
  set_motor_inin(duty_r, dir_r, TB_CH_B1, TB_CH_B2);
}

static void motor_watchdog_task(void *arg)
{
  (void)arg;
  while (1) {
    int64_t now_us = esp_timer_get_time();
    int64_t age_ms = (now_us - s_last_cmd_us) / 1000;
    if (age_ms > TB_MOTOR_WATCHDOG_MS) {
      motors_apply(0.0f, 0.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(TB_MOTOR_WATCHDOG_POLL_MS));
  }
}

void tb_motors_init(void)
{
  ledc_timer_config_t tcfg = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .timer_num        = LEDC_TIMER_0,
    .duty_resolution  = TB_MOTOR_PWM_RES,
    .freq_hz          = TB_MOTOR_PWM_HZ,
    .clk_cfg          = LEDC_AUTO_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

  ledc_channel_config_t c = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_sel  = LEDC_TIMER_0,
    .duty       = 0,
    .hpoint     = 0,
  };

  c.gpio_num = TB_MTA1_GPIO; c.channel = TB_CH_A1; ESP_ERROR_CHECK(ledc_channel_config(&c));
  c.gpio_num = TB_MTA2_GPIO; c.channel = TB_CH_A2; ESP_ERROR_CHECK(ledc_channel_config(&c));
  c.gpio_num = TB_MTB1_GPIO; c.channel = TB_CH_B1; ESP_ERROR_CHECK(ledc_channel_config(&c));
  c.gpio_num = TB_MTB2_GPIO; c.channel = TB_CH_B2; ESP_ERROR_CHECK(ledc_channel_config(&c));

  ESP_LOGI(TAG, "motors init A=(%d,%d) B=(%d,%d) pwm=%dHz",
           TB_MTA1_GPIO, TB_MTA2_GPIO, TB_MTB1_GPIO, TB_MTB2_GPIO, TB_MOTOR_PWM_HZ);

  s_last_cmd_us = esp_timer_get_time();
  xTaskCreatePinnedToCore(motor_watchdog_task, "tb_motor_wd", 2048, NULL, 5, NULL, 1);
}

void tb_motors_set(float speed_left, float speed_right)
{
  s_last_cmd_us = esp_timer_get_time();
  motors_apply(speed_left, speed_right);
}
