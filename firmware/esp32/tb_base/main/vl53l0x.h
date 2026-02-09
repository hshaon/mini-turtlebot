// VL53L0X control
// Copyright © 2019 Adrian Kennard, Andrews & Arnold Ltd. See LICENCE file for details. GPL 3.0
// Updated for ESP-IDF v5.2+ I2C Master Driver API

#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <malloc.h>
#include <driver/i2c_master.h>

typedef struct vl53l0x_s vl53l0x_t;

typedef enum
{ VcselPeriodPreRange, VcselPeriodFinalRange } vl53l0x_vcselPeriodType;

// Functions returning const char * are OK for NULL, else error string

// Legacy API: Set up I2C bus and create the vl53l0x structure
// This will create and manage its own I2C bus (single device use case)
vl53l0x_t *vl53l0x_config (int8_t port, int8_t scl, int8_t sda, int8_t xshut, uint8_t address, uint8_t io_2v8);

// New API: Create vl53l0x device using an existing I2C bus handle
// Use this when you have multiple devices on the same I2C bus
// bus_handle: existing I2C master bus (from i2c_new_master_bus)
// Returns NULL on failure
vl53l0x_t *vl53l0x_config_with_bus (i2c_master_bus_handle_t bus_handle, int8_t xshut, uint8_t address, uint8_t io_2v8);

// Initialise the VL53L0X
const char *vl53l0x_init (vl53l0x_t *);

// End I2C and free the structure
// If using vl53l0x_config_with_bus, this will NOT delete the bus (caller manages it)
void vl53l0x_end (vl53l0x_t *);

void vl53l0x_setAddress (vl53l0x_t *, uint8_t new_addr);
uint8_t vl53l0x_getAddress (vl53l0x_t * v);

void vl53l0x_writeReg8Bit (vl53l0x_t *, uint8_t reg, uint8_t value);
void vl53l0x_writeReg16Bit (vl53l0x_t *, uint8_t reg, uint16_t value);
void vl53l0x_writeReg32Bit (vl53l0x_t *, uint8_t reg, uint32_t value);
uint8_t vl53l0x_readReg8Bit (vl53l0x_t *, uint8_t reg);
uint16_t vl53l0x_readReg16Bit (vl53l0x_t *, uint8_t reg);
uint32_t vl53l0x_readReg32Bit (vl53l0x_t *, uint8_t reg);

void vl53l0x_writeMulti (vl53l0x_t *, uint8_t reg, uint8_t const *src, uint8_t count);
void vl53l0x_readMulti (vl53l0x_t *, uint8_t reg, uint8_t * dst, uint8_t count);

const char *vl53l0x_setSignalRateLimit (vl53l0x_t *, float limit_Mcps);
float vl53l0x_getSignalRateLimit (vl53l0x_t *);

const char *vl53l0x_setMeasurementTimingBudget (vl53l0x_t *, uint32_t budget_us);
uint32_t vl53l0x_getMeasurementTimingBudget (vl53l0x_t *);

const char *vl53l0x_setVcselPulsePeriod (vl53l0x_t *, vl53l0x_vcselPeriodType type, uint8_t period_pclks);
uint8_t vl53l0x_getVcselPulsePeriod (vl53l0x_t *, vl53l0x_vcselPeriodType type);

void vl53l0x_startContinuous (vl53l0x_t *, uint32_t period_ms);
void vl53l0x_stopContinuous (vl53l0x_t *);
uint16_t vl53l0x_readRangeContinuousMillimeters (vl53l0x_t *);
uint16_t vl53l0x_readRangeSingleMillimeters (vl53l0x_t *);

void vl53l0x_setTimeout (vl53l0x_t *, uint16_t timeout);
uint16_t vl53l0x_getTimeout (vl53l0x_t *);
int vl53l0x_timeoutOccurred (vl53l0x_t *);
int vl53l0x_i2cFail (vl53l0x_t *);


#endif
