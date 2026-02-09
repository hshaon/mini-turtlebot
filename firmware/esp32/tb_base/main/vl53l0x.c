// VL53L0X control
// Copyright © 2019 Adrian Kennard, Andrews & Arnold Ltd. See LICENCE file for details. GPL 3.0
// Updated for ESP-IDF v5.2+ I2C Master Driver API
// Based on https://github.com/pololu/vl53l0x-arduino
static const char __attribute__((unused)) TAG[] = "ranger";

#include "vl53l0x.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <string.h>

#define TIMEOUT_MS 100 // I2C command timeout in milliseconds
#define I2C_BUSSPEED 100000 // I2C Bus Speed

#ifdef	CONFIG_VL53L0X_DEBUG
#define VL53L0X_LOG   ESP_LOGI        // Set to allow I2C logging
#endif

#ifndef VL53L0X_LOG
#define VL53L0X_LOG(tag,...) err=err;
#endif

enum
{
   SYSRANGE_START = 0x00,

   SYSTEM_THRESH_HIGH = 0x0C,
   SYSTEM_THRESH_LOW = 0x0E,

   SYSTEM_SEQUENCE_CONFIG = 0x01,
   SYSTEM_RANGE_CONFIG = 0x09,
   SYSTEM_INTERMEASUREMENT_PERIOD = 0x04,

   SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,

   GPIO_HV_MUX_ACTIVE_HIGH = 0x84,

   SYSTEM_INTERRUPT_CLEAR = 0x0B,

   RESULT_INTERRUPT_STATUS = 0x13,
   RESULT_RANGE_STATUS = 0x14,

   RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC,
   RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0,
   RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0,
   RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4,
   RESULT_PEAK_SIGNAL_RATE_REF = 0xB6,

   ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28,

   MSRC_CONFIG_CONTROL = 0x60,

   PRE_RANGE_CONFIG_MIN_SNR = 0x27,
   PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56,
   PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57,
   PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64,

   FINAL_RANGE_CONFIG_MIN_SNR = 0x67,
   FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47,
   FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48,
   FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

   PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61,
   PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62,

   PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50,
   PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51,
   PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52,

   SYSTEM_HISTOGRAM_BIN = 0x81,
   HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33,
   HISTOGRAM_CONFIG_READOUT_CTRL = 0x55,

   FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70,
   FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71,
   FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72,
   CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20,

   MSRC_CONFIG_TIMEOUT_MACROP = 0x46,

   I2C_SLAVE_DEVICE_ADDRESS = 0x8A,

   SOFT_RESET_GO2_SOFT_RESET_N = 0xBF,
   IDENTIFICATION_MODEL_ID = 0xC0,
   IDENTIFICATION_REVISION_ID = 0xC2,

   OSC_CALIBRATE_VAL = 0xF8,

   GLOBAL_CONFIG_VCSEL_WIDTH = 0x32,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5,

   GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
   DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
   DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
   POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80,

   VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,

   ALGO_PHASECAL_LIM = 0x30,
   ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30,
};

struct vl53l0x_s
{
   i2c_master_bus_handle_t bus_handle;
   i2c_master_dev_handle_t dev_handle;
   uint8_t address;
   int8_t xshut;
   uint16_t io_timeout;
   uint8_t io_2v8:1;
   uint8_t did_timeout:1;
   uint8_t i2c_fail:1;
   uint8_t owns_bus:1;  // true if this instance created the bus
};

typedef struct
{
   uint8_t tcc:1;
   uint8_t msrc:1;
   uint8_t dss:1;
   uint8_t pre_range:1;
   uint8_t final_range:1;
} SequenceStepEnables;

typedef struct
{
   uint16_t pre_range_vcsel_period_pclks,
     final_range_vcsel_period_pclks;
   uint16_t msrc_dss_tcc_mclks,
     pre_range_mclks,
     final_range_mclks;
   uint32_t msrc_dss_tcc_us,
     pre_range_us,
     final_range_us;
}
SequenceStepTimeouts;

static uint8_t stop_variable;
static uint16_t timeout_start_ms;
static uint32_t measurement_timing_budget_us;
#define millis() (esp_timer_get_time()/1000LL)

// Record the current time to check an upcoming timeout against
#define startTimeout() (timeout_start_ms = millis())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (v->io_timeout > 0 && ((uint16_t)(millis() - timeout_start_ms)) > v->io_timeout)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

void
vl53l0x_writeReg8Bit (vl53l0x_t * v, uint8_t reg, uint8_t val)
{
   uint8_t write_buf[2] = {reg, val};
   esp_err_t err = i2c_master_transmit(v->dev_handle, write_buf, 2, TIMEOUT_MS);
   if (err != ESP_OK)
      v->i2c_fail = 1;
   VL53L0X_LOG (TAG, "W %02X=%02X %s", reg, val, esp_err_to_name (err));
}

void
vl53l0x_writeReg16Bit (vl53l0x_t * v, uint8_t reg, uint16_t val)
{
   uint8_t write_buf[3] = {reg, val >> 8, val & 0xFF};
   esp_err_t err = i2c_master_transmit(v->dev_handle, write_buf, 3, TIMEOUT_MS);
   if (err != ESP_OK)
      v->i2c_fail = 1;
   VL53L0X_LOG (TAG, "W %02X=%04X %s", reg, val, esp_err_to_name (err));
}

void
vl53l0x_writeReg32Bit (vl53l0x_t * v, uint8_t reg, uint32_t val)
{
   uint8_t write_buf[5] = {reg, val >> 24, val >> 16, val >> 8, val & 0xFF};
   esp_err_t err = i2c_master_transmit(v->dev_handle, write_buf, 5, TIMEOUT_MS);
   if (err != ESP_OK)
      v->i2c_fail = 1;
   VL53L0X_LOG (TAG, "W %02X=%08X %s", reg, val, esp_err_to_name (err));
}

uint8_t
vl53l0x_readReg8Bit (vl53l0x_t * v, uint8_t reg)
{
   uint8_t buf[1] = {0};
   esp_err_t err = i2c_master_transmit_receive(v->dev_handle, &reg, 1, buf, 1, TIMEOUT_MS);
   if (err != ESP_OK)
      v->i2c_fail = 1;
   VL53L0X_LOG (TAG, "R %02X=%02X %s", reg, buf[0], esp_err_to_name (err));
   return buf[0];
}

uint16_t
vl53l0x_readReg16Bit (vl53l0x_t * v, uint8_t reg)
{
   uint8_t buf[2] = {0};
   esp_err_t err = i2c_master_transmit_receive(v->dev_handle, &reg, 1, buf, 2, TIMEOUT_MS);
   if (err != ESP_OK)
      v->i2c_fail = 1;
   VL53L0X_LOG (TAG, "R %02X=%02X%02X %s", reg, buf[0], buf[1], esp_err_to_name (err));
   return (buf[0] << 8) + buf[1];
}

uint32_t
vl53l0x_readReg32Bit (vl53l0x_t * v, uint8_t reg)
{
   uint8_t buf[4] = {0};
   esp_err_t err = i2c_master_transmit_receive(v->dev_handle, &reg, 1, buf, 4, TIMEOUT_MS);
   if (err != ESP_OK)
      v->i2c_fail = 1;
   VL53L0X_LOG (TAG, "R %02X=%02X%02X%02X%02X %s", reg, buf[0], buf[1], buf[2], buf[3], esp_err_to_name (err));
   return (buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];
}

// Read an arbitrary number of bytes from the sensor, starting at the given register
void
vl53l0x_readMulti (vl53l0x_t * v, uint8_t reg, uint8_t * dst, uint8_t count)
{
   esp_err_t err = i2c_master_transmit_receive(v->dev_handle, &reg, 1, dst, count, TIMEOUT_MS);
   if (err != ESP_OK)
      v->i2c_fail = 1;
   VL53L0X_LOG (TAG, "R %02X (%d) %s", reg, count, esp_err_to_name (err));
}

// Write an arbitrary number of bytes from the given array to the sensor, starting at the given register
void
vl53l0x_writeMulti (vl53l0x_t * v, uint8_t reg, uint8_t const *src, uint8_t count)
{
   uint8_t *write_buf = malloc(count + 1);
   if (!write_buf) {
      v->i2c_fail = 1;
      return;
   }
   write_buf[0] = reg;
   memcpy(write_buf + 1, src, count);
   esp_err_t err = i2c_master_transmit(v->dev_handle, write_buf, count + 1, TIMEOUT_MS);
   free(write_buf);
   if (err != ESP_OK)
      v->i2c_fail = 1;
   VL53L0X_LOG (TAG, "W %02X (%d) %s", reg, count, esp_err_to_name (err));
}

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
static uint16_t
decodeTimeout (uint16_t reg_val)
{
   return (uint16_t) ((reg_val & 0x00FF) << (uint16_t) ((reg_val & 0xFF00) >> 8)) + 1;
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
static uint32_t
timeoutMclksToMicroseconds (uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
   uint32_t macro_period_ns = calcMacroPeriod (vcsel_period_pclks);
   return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// based on VL53L0X_perform_single_ref_calibration()
static const char *
performSingleRefCalibration (vl53l0x_t * v, uint8_t vhv_init_byte)
{
   vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x01 | vhv_init_byte);
   startTimeout ();
   while ((vl53l0x_readReg8Bit (v, RESULT_INTERRUPT_STATUS) & 0x07) == 0)
   {
      if (checkTimeoutExpired ())
         return "CAL Timeout";
   }
   vl53l0x_writeReg8Bit (v, SYSTEM_INTERRUPT_CLEAR, 0x01);
   vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x00);
   return NULL;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
static uint16_t
encodeTimeout (uint16_t timeout_mclks)
{
   uint32_t ls_byte = 0;
   uint16_t ms_byte = 0;

   if (timeout_mclks > 0)
   {
      ls_byte = timeout_mclks - 1;

      while ((ls_byte & 0xFFFFFF00) > 0)
      {
         ls_byte >>= 1;
         ms_byte++;
      }

      return (ms_byte << 8) | (ls_byte & 0xFF);
   } else
   {
      return 0;
   }
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
static uint8_t
getVcselPulsePeriod (vl53l0x_t * v, vl53l0x_vcselPeriodType type)
{
   if (type == VcselPeriodPreRange)
   {
      return decodeVcselPeriod (vl53l0x_readReg8Bit (v, PRE_RANGE_CONFIG_VCSEL_PERIOD));
   } else if (type == VcselPeriodFinalRange)
   {
      return decodeVcselPeriod (vl53l0x_readReg8Bit (v, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
   } else
   {
      return 255;
   }
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
static void
getSequenceStepEnables (vl53l0x_t * v, SequenceStepEnables * enables)
{
   uint8_t sequence_config = vl53l0x_readReg8Bit (v, SYSTEM_SEQUENCE_CONFIG);

   enables->tcc = (sequence_config >> 4) & 0x1;
   enables->dss = (sequence_config >> 3) & 0x1;
   enables->msrc = (sequence_config >> 2) & 0x1;
   enables->pre_range = (sequence_config >> 6) & 0x1;
   enables->final_range = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout()
static void
getSequenceStepTimeouts (vl53l0x_t * v, SequenceStepEnables const *enables, SequenceStepTimeouts * timeouts)
{
   timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod (v, VcselPeriodPreRange);

   timeouts->msrc_dss_tcc_mclks = vl53l0x_readReg8Bit (v, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
   timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds (timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

   timeouts->pre_range_mclks = decodeTimeout (vl53l0x_readReg16Bit (v, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
   timeouts->pre_range_us = timeoutMclksToMicroseconds (timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

   timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod (v, VcselPeriodFinalRange);

   timeouts->final_range_mclks = decodeTimeout (vl53l0x_readReg16Bit (v, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

   if (enables->pre_range)
   {
      timeouts->final_range_mclks -= timeouts->pre_range_mclks;
   }

   timeouts->final_range_us = timeoutMclksToMicroseconds (timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
static uint32_t
timeoutMicrosecondsToMclks (uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
   uint32_t macro_period_ns = calcMacroPeriod (vcsel_period_pclks);

   return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

const char *
vl53l0x_setSignalRateLimit (vl53l0x_t * v, float limit_Mcps)
{
   if (limit_Mcps < 0 || limit_Mcps > 511.99)
      return "Bad rate";
   // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
   vl53l0x_writeReg16Bit (v, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
   return NULL;
}

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device()
const char *
vl53l0x_getSpadInfo (vl53l0x_t * v, uint8_t * count, int *type_is_aperture)
{
   uint8_t tmp;

   vl53l0x_writeReg8Bit (v, 0x80, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x00);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x06);
   vl53l0x_writeReg8Bit (v, 0x83, vl53l0x_readReg8Bit (v, 0x83) | 0x04);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x07);
   vl53l0x_writeReg8Bit (v, 0x81, 0x01);

   vl53l0x_writeReg8Bit (v, 0x80, 0x01);

   vl53l0x_writeReg8Bit (v, 0x94, 0x6b);
   vl53l0x_writeReg8Bit (v, 0x83, 0x00);
   startTimeout ();
   while (vl53l0x_readReg8Bit (v, 0x83) == 0x00)
   {
      if (checkTimeoutExpired ())
         return "SPAD Timeout";
   }
   vl53l0x_writeReg8Bit (v, 0x83, 0x01);
   tmp = vl53l0x_readReg8Bit (v, 0x92);

   *count = tmp & 0x7f;
   *type_is_aperture = (tmp >> 7) & 0x01;

   vl53l0x_writeReg8Bit (v, 0x81, 0x00);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x06);
   vl53l0x_writeReg8Bit (v, 0x83, vl53l0x_readReg8Bit (v, 0x83) & ~0x04);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x01);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x80, 0x00);

   return NULL;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
uint32_t
vl53l0x_getMeasurementTimingBudget (vl53l0x_t * v)
{
   SequenceStepEnables enables;
   SequenceStepTimeouts timeouts;

   uint16_t const StartOverhead = 1910;
   uint16_t const EndOverhead = 960;
   uint16_t const MsrcOverhead = 660;
   uint16_t const TccOverhead = 590;
   uint16_t const DssOverhead = 690;
   uint16_t const PreRangeOverhead = 660;
   uint16_t const FinalRangeOverhead = 550;

   uint32_t budget_us = StartOverhead + EndOverhead;

   getSequenceStepEnables (v, &enables);
   getSequenceStepTimeouts (v, &enables, &timeouts);

   if (enables.tcc)
   {
      budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
   }

   if (enables.dss)
   {
      budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
   } else if (enables.msrc)
   {
      budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
   }

   if (enables.pre_range)
   {
      budget_us += (timeouts.pre_range_us + PreRangeOverhead);
   }

   if (enables.final_range)
   {
      budget_us += (timeouts.final_range_us + FinalRangeOverhead);
   }

   measurement_timing_budget_us = budget_us;
   return budget_us;
}

// Set the measurement timing budget in microseconds
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
const char *
vl53l0x_setMeasurementTimingBudget (vl53l0x_t * v, uint32_t budget_us)
{
   SequenceStepEnables enables;
   SequenceStepTimeouts timeouts;

   uint16_t const StartOverhead = 1320;
   uint16_t const EndOverhead = 960;
   uint16_t const MsrcOverhead = 660;
   uint16_t const TccOverhead = 590;
   uint16_t const DssOverhead = 690;
   uint16_t const PreRangeOverhead = 660;
   uint16_t const FinalRangeOverhead = 550;

   uint32_t const MinTimingBudget = 20000;

   if (budget_us < MinTimingBudget)
      return "Low budget";

   uint32_t used_budget_us = StartOverhead + EndOverhead;

   getSequenceStepEnables (v, &enables);
   getSequenceStepTimeouts (v, &enables, &timeouts);

   if (enables.tcc)
      used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);

   if (enables.dss)
      used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
   else if (enables.msrc)
      used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);

   if (enables.pre_range)
      used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);

   if (enables.final_range)
   {
      used_budget_us += FinalRangeOverhead;

      if (used_budget_us > budget_us)
         return "High budget";

      uint32_t final_range_timeout_us = budget_us - used_budget_us;

      uint16_t
         final_range_timeout_mclks = timeoutMicrosecondsToMclks (final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

      if (enables.pre_range)
         final_range_timeout_mclks += timeouts.pre_range_mclks;

      vl53l0x_writeReg16Bit (v, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout (final_range_timeout_mclks));

      measurement_timing_budget_us = budget_us;
   }
   return NULL;
}

// Create vl53l0x device using an existing I2C bus handle
vl53l0x_t *
vl53l0x_config_with_bus (i2c_master_bus_handle_t bus_handle, int8_t xshut, uint8_t address, uint8_t io_2v8)
{
   if (!bus_handle)
      return NULL;
   
   if (xshut >= 0 && !GPIO_IS_VALID_OUTPUT_GPIO (xshut))
      return NULL;
   
   vl53l0x_t *v = malloc (sizeof (*v));
   if (!v)
      return NULL;
   
   memset (v, 0, sizeof (*v));
   
   // Configure the I2C device on the existing bus
   i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = address,
      .scl_speed_hz = I2C_BUSSPEED,
   };
   
   esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &v->dev_handle);
   if (err != ESP_OK) {
      free(v);
      return NULL;
   }
   
   v->bus_handle = bus_handle;
   v->xshut = xshut;
   v->io_2v8 = io_2v8;
   v->address = address;
   v->io_timeout = 100;
   v->owns_bus = 0;  // Caller manages the bus
   
   if (xshut >= 0)
   {
      gpio_reset_pin (xshut);
      gpio_set_level (xshut, 0);        // Off
      gpio_set_drive_capability (xshut, GPIO_DRIVE_CAP_3);
      gpio_set_direction (xshut, GPIO_MODE_OUTPUT);
   }
   
   return v;
}

// Legacy API: Set up I2C bus and create the vl53l0x structure
vl53l0x_t *
vl53l0x_config (int8_t port, int8_t scl, int8_t sda, int8_t xshut, uint8_t address, uint8_t io_2v8)
{
   if (port < 0 || scl < 0 || sda < 0 || scl == sda)
      return NULL;
   if (!GPIO_IS_VALID_OUTPUT_GPIO (scl) || !GPIO_IS_VALID_OUTPUT_GPIO (sda))
      return NULL;
   if (xshut >= 0 && !GPIO_IS_VALID_OUTPUT_GPIO (xshut))
      return NULL;

   // Create new I2C master bus
   i2c_master_bus_config_t bus_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = port,
      .scl_io_num = scl,
      .sda_io_num = sda,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
   };
   
   i2c_master_bus_handle_t bus_handle;
   esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
   if (err != ESP_OK)
      return NULL;
   
   // Create device on the bus
   vl53l0x_t *v = vl53l0x_config_with_bus(bus_handle, xshut, address, io_2v8);
   if (!v) {
      i2c_del_master_bus(bus_handle);
      return NULL;
   }
   
   v->owns_bus = 1;  // This instance owns the bus
   return v;
}

// Initialize sensor
const char *
vl53l0x_init (vl53l0x_t * v)
{
   const char *err;
   // Set up the VL53L0X
   if (v->xshut >= 0)
   {
      gpio_set_level (v->xshut, 0);     // Off
      usleep (100000);
      gpio_set_level (v->xshut, 1);     // On
      usleep (10000);
   }
   
   // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
   if (v->io_2v8)
      vl53l0x_writeReg8Bit (v, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, vl53l0x_readReg8Bit (v, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
   
   // "Set I2C standard mode"
   vl53l0x_writeReg8Bit (v, 0x88, 0x00);

   vl53l0x_writeReg8Bit (v, 0x80, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x00);
   stop_variable = vl53l0x_readReg8Bit (v, 0x91);
   vl53l0x_writeReg8Bit (v, 0x00, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x80, 0x00);

   // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
   vl53l0x_writeReg8Bit (v, MSRC_CONFIG_CONTROL, vl53l0x_readReg8Bit (v, MSRC_CONFIG_CONTROL) | 0x12);

   // set final range signal rate limit to 0.25 MCPS (million counts per second)
   if ((err = vl53l0x_setSignalRateLimit (v, 0.25)))
      return err;

   vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0xFF);

   // VL53L0X_DataInit() end

   // VL53L0X_StaticInit() begin

   uint8_t spad_count;
   int spad_type_is_aperture;
   if ((err = vl53l0x_getSpadInfo (v, &spad_count, &spad_type_is_aperture)))
      return err;
   
   // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
   // the API, but the same data seems to be more easily readable from
   // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
   uint8_t ref_spad_map[6];
   vl53l0x_readMulti (v, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

   // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
   vl53l0x_writeReg8Bit (v, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

   uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
   uint8_t spads_enabled = 0;

   for (uint8_t i = 0; i < 48; i++)
   {
      if (i < first_spad_to_enable || spads_enabled == spad_count)
      {
         ref_spad_map[i / 8] &= ~(1 << (i % 8));
      } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
      {
         spads_enabled++;
      }
   }

   vl53l0x_writeMulti (v, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

   // -- VL53L0X_set_reference_spads() end

   // -- VL53L0X_load_tuning_settings() begin

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x00);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x09, 0x00);
   vl53l0x_writeReg8Bit (v, 0x10, 0x00);
   vl53l0x_writeReg8Bit (v, 0x11, 0x00);

   vl53l0x_writeReg8Bit (v, 0x24, 0x01);
   vl53l0x_writeReg8Bit (v, 0x25, 0xFF);
   vl53l0x_writeReg8Bit (v, 0x75, 0x00);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x4E, 0x2C);
   vl53l0x_writeReg8Bit (v, 0x48, 0x00);
   vl53l0x_writeReg8Bit (v, 0x30, 0x20);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x30, 0x09);
   vl53l0x_writeReg8Bit (v, 0x54, 0x00);
   vl53l0x_writeReg8Bit (v, 0x31, 0x04);
   vl53l0x_writeReg8Bit (v, 0x32, 0x03);
   vl53l0x_writeReg8Bit (v, 0x40, 0x83);
   vl53l0x_writeReg8Bit (v, 0x46, 0x25);
   vl53l0x_writeReg8Bit (v, 0x60, 0x00);
   vl53l0x_writeReg8Bit (v, 0x27, 0x00);
   vl53l0x_writeReg8Bit (v, 0x50, 0x06);
   vl53l0x_writeReg8Bit (v, 0x51, 0x00);
   vl53l0x_writeReg8Bit (v, 0x52, 0x96);
   vl53l0x_writeReg8Bit (v, 0x56, 0x08);
   vl53l0x_writeReg8Bit (v, 0x57, 0x30);
   vl53l0x_writeReg8Bit (v, 0x61, 0x00);
   vl53l0x_writeReg8Bit (v, 0x62, 0x00);
   vl53l0x_writeReg8Bit (v, 0x64, 0x00);
   vl53l0x_writeReg8Bit (v, 0x65, 0x00);
   vl53l0x_writeReg8Bit (v, 0x66, 0xA0);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x22, 0x32);
   vl53l0x_writeReg8Bit (v, 0x47, 0x14);
   vl53l0x_writeReg8Bit (v, 0x49, 0xFF);
   vl53l0x_writeReg8Bit (v, 0x4A, 0x00);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x7A, 0x0A);
   vl53l0x_writeReg8Bit (v, 0x7B, 0x00);
   vl53l0x_writeReg8Bit (v, 0x78, 0x21);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x23, 0x34);
   vl53l0x_writeReg8Bit (v, 0x42, 0x00);
   vl53l0x_writeReg8Bit (v, 0x44, 0xFF);
   vl53l0x_writeReg8Bit (v, 0x45, 0x26);
   vl53l0x_writeReg8Bit (v, 0x46, 0x05);
   vl53l0x_writeReg8Bit (v, 0x40, 0x40);
   vl53l0x_writeReg8Bit (v, 0x0E, 0x06);
   vl53l0x_writeReg8Bit (v, 0x20, 0x1A);
   vl53l0x_writeReg8Bit (v, 0x43, 0x40);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x34, 0x03);
   vl53l0x_writeReg8Bit (v, 0x35, 0x44);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x31, 0x04);
   vl53l0x_writeReg8Bit (v, 0x4B, 0x09);
   vl53l0x_writeReg8Bit (v, 0x4C, 0x05);
   vl53l0x_writeReg8Bit (v, 0x4D, 0x04);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x44, 0x00);
   vl53l0x_writeReg8Bit (v, 0x45, 0x20);
   vl53l0x_writeReg8Bit (v, 0x47, 0x08);
   vl53l0x_writeReg8Bit (v, 0x48, 0x28);
   vl53l0x_writeReg8Bit (v, 0x67, 0x00);
   vl53l0x_writeReg8Bit (v, 0x70, 0x04);
   vl53l0x_writeReg8Bit (v, 0x71, 0x01);
   vl53l0x_writeReg8Bit (v, 0x72, 0xFE);
   vl53l0x_writeReg8Bit (v, 0x76, 0x00);
   vl53l0x_writeReg8Bit (v, 0x77, 0x00);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x0D, 0x01);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x80, 0x01);
   vl53l0x_writeReg8Bit (v, 0x01, 0xF8);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x8E, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x80, 0x00);

   // -- VL53L0X_load_tuning_settings() end

   // "Set interrupt config to new sample ready"
   vl53l0x_writeReg8Bit (v, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
   vl53l0x_writeReg8Bit (v, GPIO_HV_MUX_ACTIVE_HIGH, vl53l0x_readReg8Bit (v, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
   vl53l0x_writeReg8Bit (v, SYSTEM_INTERRUPT_CLEAR, 0x01);

   measurement_timing_budget_us = vl53l0x_getMeasurementTimingBudget (v);

   // "Disable MSRC and TCC by default"
   vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0xE8);

   // "Recalculate timing budget"
   if ((err = vl53l0x_setMeasurementTimingBudget (v, measurement_timing_budget_us)))
      return err;

   // VL53L0X_StaticInit() end

   // VL53L0X_PerformRefCalibration() begin

   vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0x01);
   if ((err = performSingleRefCalibration (v, 0x40)))
      return err;

   vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0x02);
   if ((err = performSingleRefCalibration (v, 0x00)))
      return err;

   // "restore the previous Sequence Config"
   vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0xE8);

   // VL53L0X_PerformRefCalibration() end
   if (vl53l0x_i2cFail (v))
      return "I2C fail";

   return NULL;
}

void
vl53l0x_end (vl53l0x_t * v)
{
   if (!v)
      return;
   
   if (v->dev_handle) {
      i2c_master_bus_rm_device(v->dev_handle);
   }
   
   if (v->owns_bus && v->bus_handle) {
      i2c_del_master_bus(v->bus_handle);
   }
   
   free (v);
}

void
vl53l0x_setAddress (vl53l0x_t * v, uint8_t new_addr)
{
   vl53l0x_writeReg8Bit (v, I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
   v->address = new_addr;
      // Remove the old device handle and create a new one with the new address
   if (v->dev_handle) {
      i2c_master_bus_rm_device(v->dev_handle);
      
      // Create new device handle with updated address
      i2c_device_config_t dev_cfg = {
         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
         .device_address = new_addr,
         .scl_speed_hz = I2C_BUSSPEED,  // Match your current speed setting
      };
      
      i2c_master_bus_add_device(v->bus_handle, &dev_cfg, &v->dev_handle);
   }
}

uint8_t
vl53l0x_getAddress (vl53l0x_t * v)
{
   return v->address;
}

void
vl53l0x_setTimeout (vl53l0x_t * v, uint16_t new_timeout)
{
   v->io_timeout = new_timeout;
}

uint16_t
vl53l0x_getTimeout (vl53l0x_t * v)
{
   return v->io_timeout;
}

int
vl53l0x_timeoutOccurred (vl53l0x_t * v)
{
   int tmp = v->did_timeout;
   v->did_timeout = 0;
   return tmp;
}

int
vl53l0x_i2cFail (vl53l0x_t * v)
{
   int tmp = v->i2c_fail;
   v->i2c_fail = 0;
   return tmp;
}

float
vl53l0x_getSignalRateLimit (vl53l0x_t * v)
{
   return (float) vl53l0x_readReg16Bit (v, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Set the VCSEL pulse period for the given period type
const char *
vl53l0x_setVcselPulsePeriod (vl53l0x_t * v, vl53l0x_vcselPeriodType type, uint8_t period_pclks)
{
   uint8_t vcsel_period_reg = encodeVcselPeriod (period_pclks);

   SequenceStepEnables enables;
   SequenceStepTimeouts timeouts;

   getSequenceStepEnables (v, &enables);
   getSequenceStepTimeouts (v, &enables, &timeouts);

   if (type == VcselPeriodPreRange)
   {
      switch (period_pclks)
      {
      case 12:
         vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
         break;
      case 14:
         vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
         break;
      case 16:
         vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
         break;
      case 18:
         vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
         break;
      default:
         return "Invalid period";
      }
      vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
      vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

      uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks (timeouts.pre_range_us, period_pclks);
      vl53l0x_writeReg16Bit (v, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout (new_pre_range_timeout_mclks));

      uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks (timeouts.msrc_dss_tcc_us, period_pclks);
      vl53l0x_writeReg8Bit (v, MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
   } else if (type == VcselPeriodFinalRange)
   {
      switch (period_pclks)
      {
      case 8:
         vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
         vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
         vl53l0x_writeReg8Bit (v, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
         vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
         vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
         vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_LIM, 0x30);
         vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
         break;
      case 10:
         vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
         vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
         vl53l0x_writeReg8Bit (v, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
         vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
         vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
         vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_LIM, 0x20);
         vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
         break;
      case 12:
         vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
         vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
         vl53l0x_writeReg8Bit (v, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
         vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
         vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
         vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_LIM, 0x20);
         vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
         break;
      case 14:
         vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
         vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
         vl53l0x_writeReg8Bit (v, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
         vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
         vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
         vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_LIM, 0x20);
         vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
         break;
      default:
         return "Invalid period";
      }

      vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

      uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks (timeouts.final_range_us, period_pclks);

      if (enables.pre_range)
         new_final_range_timeout_mclks += timeouts.pre_range_mclks;

      vl53l0x_writeReg16Bit (v, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout (new_final_range_timeout_mclks));
   } else
      return "Invalid type";

   const char *err;
   if ((err = vl53l0x_setMeasurementTimingBudget (v, measurement_timing_budget_us)))
      return err;

   uint8_t sequence_config = vl53l0x_readReg8Bit (v, SYSTEM_SEQUENCE_CONFIG);
   vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0x02);
   performSingleRefCalibration (v, 0x0);
   vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, sequence_config);

   return NULL;
}

void
vl53l0x_startContinuous (vl53l0x_t * v, uint32_t period_ms)
{
   vl53l0x_writeReg8Bit (v, 0x80, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x00);
   vl53l0x_writeReg8Bit (v, 0x91, stop_variable);
   vl53l0x_writeReg8Bit (v, 0x00, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x80, 0x00);

   if (period_ms != 0)
   {
      uint16_t osc_calibrate_val = vl53l0x_readReg16Bit (v, OSC_CALIBRATE_VAL);

      if (osc_calibrate_val != 0)
      {
         period_ms *= osc_calibrate_val;
      }

      vl53l0x_writeReg32Bit (v, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
      vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x04);
   } else
   {
      vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x02);
   }
}

void
vl53l0x_stopContinuous (vl53l0x_t * v)
{
   vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x01);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x00);
   vl53l0x_writeReg8Bit (v, 0x91, 0x00);
   vl53l0x_writeReg8Bit (v, 0x00, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
}

uint16_t
vl53l0x_readRangeContinuousMillimeters (vl53l0x_t * v)
{
   startTimeout ();
   while ((vl53l0x_readReg8Bit (v, RESULT_INTERRUPT_STATUS) & 0x07) == 0)
   {
      if (checkTimeoutExpired ())
      {
         v->did_timeout = 1;
         return 65535;
      }
   }
   uint16_t range = vl53l0x_readReg16Bit (v, RESULT_RANGE_STATUS + 10);
   vl53l0x_writeReg8Bit (v, SYSTEM_INTERRUPT_CLEAR, 0x01);
   return range;
}

uint16_t
vl53l0x_readRangeSingleMillimeters (vl53l0x_t * v)
{
   vl53l0x_writeReg8Bit (v, 0x80, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x00);
   vl53l0x_writeReg8Bit (v, 0x91, stop_variable);
   vl53l0x_writeReg8Bit (v, 0x00, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x80, 0x00);

   vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x01);

   startTimeout ();
   while (vl53l0x_readReg8Bit (v, SYSRANGE_START) & 0x01)
   {
      if (checkTimeoutExpired ())
      {
         v->did_timeout = 1;
         return 65535;
      }
   }

   return vl53l0x_readRangeContinuousMillimeters (v);
}
