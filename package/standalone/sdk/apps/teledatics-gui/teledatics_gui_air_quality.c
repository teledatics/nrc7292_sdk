/*
 * MIT License
 *
 * Copyright (c) 2022 Teledatics, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

/**
 * @file teledatics_gui_air_quality.c
 * @author James Ewing
 * @date 8 Apr 2022
 * @brief Teledatics Air Quality hAT utilities for Halo TD-XPAH
 */

#include "teledatics_gui_air_quality.h"
#include "teledatics_gui.h"

#ifdef SUPPORT_SHT30
#include "sht30.h"
#endif

#ifdef SUPPORT_SGP30
#include "sgp30.h"
#endif

typedef struct
{
#ifdef SUPPORT_SHT30
  struct sht30_dev sht30;
#endif
#ifdef SUPPORT_SGP30
  struct sgp30_dev sgp30;
#endif
  i2c_device_t dev_i2c;
} sht_sensor_t;

sht_sensor_t sensor;

static int8_t
i2c_read(uint8_t dev_id, uint8_t* data, uint16_t len)
{
  nrc_i2c_start(&sensor.dev_i2c);
  nrc_i2c_writebyte(&sensor.dev_i2c, (dev_id << 1) | 0x01);
  for (int i = 0; i < len; i++)
    nrc_i2c_readbyte(&sensor.dev_i2c, data++, i < len - 1);

  nrc_i2c_stop(&sensor.dev_i2c);
  _delay_ms(I2C_XACT_DELAY_MS);

  return SENSOR_OK;
}

static int8_t
i2c_write(uint8_t dev_id, uint8_t* data, uint16_t len)
{
  nrc_i2c_start(&sensor.dev_i2c);
  nrc_i2c_writebyte(&sensor.dev_i2c, (dev_id << 1) | 0x00);
  for (int i = 0; i < len; i++)
    nrc_i2c_writebyte(&sensor.dev_i2c, *data++);
  nrc_i2c_stop(&sensor.dev_i2c);
  _delay_ms(I2C_XACT_DELAY_MS);

  return SENSOR_OK;
}

static int
i2c_init(i2c_device_t* i2c)
{

  if (nrc_i2c_init(i2c) != NRC_SUCCESS) {
    return SENSOR_FAIL;
  }

  if (nrc_i2c_enable(i2c, true) != NRC_SUCCESS) {
    return SENSOR_FAIL;
  }

  return SENSOR_OK;
}

static void
delay_ms(uint32_t period)
{
  _delay_ms(period);
}

int
sensors_init(sht_sensor_t* sensor)
{
  uint8_t set_required_settings = 0;
  int8_t rslt = 0;

#ifdef SUPPORT_SHT30

  memset(&sensor->sht30, 0, sizeof(struct sht30_dev));

  sensor->sht30.chip_id = SHT30_CHIP_ID_PRIMARY;
  sensor->sht30.read = i2c_read;
  sensor->sht30.write = i2c_write;
  sensor->sht30.delay_ms = delay_ms;
#endif

#ifdef SUPPORT_SGP30
  sensor->sgp30.chip_id = SGP30_CHIP_ID_PRIMARY;
  sensor->sgp30.read = i2c_read;
  sensor->sgp30.write = i2c_write;
  sensor->sgp30.delay_ms = delay_ms;
#endif

  /* Set i2c */
  sensor->dev_i2c.pin_sda = SENSOR_I2C_SDA;
  sensor->dev_i2c.pin_scl = SENSOR_I2C_SCL;
  sensor->dev_i2c.clock_source = SENSOR_I2C_CLOCK_SOURCE;
  sensor->dev_i2c.controller = I2C_MASTER_0;
  sensor->dev_i2c.clock = SENSOR_I2C_CLOCK;
  sensor->dev_i2c.width = I2C_8BIT;

  i2c_init(&sensor->dev_i2c);
  nrc_i2c_enable(&sensor->dev_i2c, true);
  _delay_ms(100);

#ifdef SUPPORT_SHT30
  rslt = sht30_init(&sensor->sht30);
  
  nrc_usr_print("[%s] sht30_init %d\n", __func__, rslt);
#endif

#ifdef SUPPORT_SGP30
  if (rslt == SENSOR_OK) {
    rslt = sgp30_init(&sensor->sgp30);
    nrc_usr_print("[%s] sgp30_init %d\n", __func__, rslt);
#ifdef SUPPORT_SHT30
    if (rslt == SENSOR_OK) {
      td_set_absolute_humidity(td_get_air_quality_temperature(),
                               td_get_air_quality_humidity());
    }
#endif
  }
#endif

  return rslt;
}

int
sensors_deinit(sht_sensor_t* sensor)
{
  return nrc_i2c_enable(&sensor->dev_i2c, false);
}

#ifdef SUPPORT_SHT30
/**
 * @brief Retrieve temperature from air quality sensor hAT
 *
 * Get current temperature from air quality hAT
 *
 * @returns floating point temperature
 */
float
td_get_air_quality_temperature(void)
{
  return sht30_get_temp(&sensor.sht30);
}
#else
float
td_get_air_quality_temperature(void)
{
  return UNKNOWN_VALUE;
}
#endif

#ifdef SUPPORT_SHT30
/**
 * @brief Retrieve humidity from air quality sensor hAT
 *
 * Get current humidity from air quality hAT
 *
 * @returns floating point humidity
 */
float
td_get_air_quality_humidity(void)
{
  return sht30_get_humidity(&sensor.sht30);
}
#else
float
td_get_air_quality_humidity(void)
{
  return UNKNOWN_VALUE;
}
#endif

#ifdef SUPPORT_SGP30
/**
 * @brief Retrieve CO2 from air quality sensor hAT
 *
 * Get current CO2 levels from air quality hAT
 *
 * @returns floating point co2
 */
float
td_get_air_quality_co2(void)
{
  return sgp30_get_co2(&sensor.sgp30);
}
#else
float
td_get_air_quality_co2(void)
{
  return UNKNOWN_VALUE;
}
#endif

#ifdef SUPPORT_SGP30
/**
 * @brief Retrieve VOC from air quality sensor hAT
 *
 * Get current VOC levels from air quality hAT
 *
 * @returns floating point voc
 */
float
td_get_air_quality_voc(void)
{
  return sgp30_get_voc(&sensor.sgp30);
}
#else
float
td_get_air_quality_voc(void)
{
  return UNKNOWN_VALUE;
}
#endif

#ifdef SUPPORT_SGP30
/**
 * @brief Set absolute humidty in air quality sensor hAT
 *
 * Set absolute humidity from temp, humidity air quality hAT
 *
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t
td_set_absolute_humidity(float temp, float humidity)
{
  if (sgp30_set_absolute_humidity(&sensor.sgp30, temp, humidity) == SGP30_OK)
    return NRC_SUCCESS;

  return NRC_FAIL;
}
#else
nrc_err_t
td_set_relative_humidity(void)
{
  return NRC_FAIL;
}
#endif

/**
 * @brief Initialize air quality sensor hAT
 *
 * Initialize air quality sensor
 *
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t
td_init_air_quality_hat(void)
{

  if (sensors_init(&sensor) != NRC_SUCCESS) {
    return NRC_FAIL;
  }

  nrc_usr_print("[%s] air quality sensor initialized\n", __func__);

  return NRC_SUCCESS;
}

/**
 * @brief Initialize air quality sensor hAT
 *
 * Initialize air quality sensor
 *
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t
td_shutdown_air_quality_hat(void)
{

  if (sensors_deinit(&sensor) != NRC_SUCCESS) {
    return NRC_FAIL;
  }

  nrc_usr_print("[%s] air quality sensor de-initialized\n", __func__);

  return NRC_SUCCESS;
}
