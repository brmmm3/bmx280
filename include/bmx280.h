/**
 * BMX280 - BME280 & BMP280 Driver for Esspressif ESP-32.
 *
 * MIT License
 *
 * Copyright (C) 2020 Halit Utku Maden
 * Please contact at <utkumaden@hotmail.com>
 */

#ifndef _BMX280_H_
#define _BMX280_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <limits.h>
#include <assert.h>
#include "sdkconfig.h"

#include "bmx280_bits.h"
#if !(CONFIG_USE_I2C_MASTER_DRIVER)
#include "driver/i2c.h"
#else
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define BMXAPI extern

typedef struct {
  float temperature; // °C
  float humidity;    // %
  float pressure;    // hPa
  float altitude;    // m
} bmx280_values_t;

/**
 * Structure driver settings.
 */
typedef struct bmx280_s {
#if !(CONFIG_USE_I2C_MASTER_DRIVER)
  // I2C port.
  i2c_port_t i2c_port;
  // Slave Address of sensor.
  uint8_t slave;
#else
  // I2C master handle via port with configuration
  i2c_master_dev_handle_t i2c_dev;
  // I2C master configuration
  i2c_device_config_t dev_cfg;
  // I2C master handle via port
  i2c_master_bus_handle_t bus_handle;
#endif
  // Chip ID of sensor
  uint8_t address;
  uint8_t device_id;
  // Compensation data
  struct {
    uint16_t T1;
    int16_t T2;
    int16_t T3;
    uint16_t P1;
    int16_t P2;
    int16_t P3;
    int16_t P4;
    int16_t P5;
    int16_t P6;
    int16_t P7;
    int16_t P8;
    int16_t P9;
#if !(CONFIG_BMX280_EXPECT_BMP280)
    uint8_t H1;
    int16_t H2;
    uint8_t H3;
    int16_t H4;
    int16_t H5;
    int8_t H6;
#endif
  } cmps;
  // Storage for a variable proportional to temperature.
  int32_t t_fine;
  bmx280_mode_t mode;
  bmx280_values_t values;
  uint8_t debug;
} bmx280_t;

#if CONFIG_USE_I2C_MASTER_DRIVER
/**
 * Create an instance of the BMX280 driver.
 * @param bus_handle The I2C master handle via port.
 * @return A non-null pointer to the driver structure on success.
 */
BMXAPI bmx280_t *bmx280_create_master(i2c_master_bus_handle_t bus_handle);
// legacy define for existing code bases
#define bmx280_create(port) bmx280_create_legacy(port)
#define bmx280_create_legacy(port)                                             \
  static_assert(0, "You have the wrong driver configuration for using the "    \
                   "legacy I2C driver.")

esp_err_t bmx280_init(bmx280_t **bmx280, i2c_master_bus_handle_t bus_handle, bool address_hi);

#else

/**
 * Create an instance of the BMX280 driver.
 * @param port The I2C port to use.
 * @return A non-null pointer to the driver structure on success.
 */
BMXAPI bmx280_t *bmx280_create_legacy(i2c_port_t port);
// legacy define for existing code bases
#define bmx280_create(port) bmx280_create_legacy(port)
#define bmx280_create_master(port)                                             \
  static_assert(0, "You have the wrong driver configuration for using the "    \
                   "new I2C master driver.")
#endif

/**
 * Restart the sensor, effectively puting it into sleep mode.
 * @param bmx280 The instance to reset.
 */
esp_err_t bmx280_reset(bmx280_t *bmx280);

/**
 * Destroy your the instance.
 * @param bmx280 The instance to destroy.
 */
BMXAPI void bmx280_close(bmx280_t *bmx280);

/**
 * Probe for the sensor and read calibration data.
 * @param bmx280 Driver structure.
 */
BMXAPI esp_err_t bmx280_device_init(bmx280_t *bmx280, bool address_hi);
/**
 * Configure the sensor with the given parameters.
 * @param bmx280 Driver structure.
 * @param configuration The configuration to use.
 */
BMXAPI esp_err_t bmx280_configure(bmx280_t *bmx280, bmx280_config_t *cfg);

/**
 * Set the sensor mode of operation.
 * @param bmx280 Driver structure.
 * @param mode The mode to set the sensor to.
 */
BMXAPI esp_err_t bmx280_setMode(bmx280_t *bmx280, bmx280_mode_t mode);
/**
 * Get the sensor current mode of operation.
 * @param bmx280 Driver structure.
 * @param mode Pointer to write current mode to.
 */
BMXAPI esp_err_t bmx280_getMode(bmx280_t *bmx280);

/**
 * Returns true if sensor is currently sampling environment conditions.
 * @param bmx280 Driver structure.
 */
BMXAPI bool bmx280_isSampling(bmx280_t *bmx280);

/**
 * Read sensor values as fixed point numbers.
 * @param bmx280 Driver structure.
 */
BMXAPI esp_err_t bmx280_readout(bmx280_t *bmx280);

void bmx280_dump(bmx280_t *sensor);

#ifdef __cplusplus
};
#endif

#endif
