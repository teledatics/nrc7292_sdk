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
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
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
 * @file teledatics_gui_air_quality.h
 * @author James Ewing
 * @date 8 Apr 2022
 * @brief Teledatics Air Quality hAT utilities for Halo TD-XPAH
 */

#ifndef TELEDATICS_AIR_QUALITY_H
#define TELEDATICS_AIR_QUALITY_H

#define SUPPORT_SHT30 1
// #define SUPPORT_SGP30 1

#define SENSOR_I2C_SCL             16
#define SENSOR_I2C_SDA             17
#define SENSOR_I2C_CLOCK           100000
#define SENSOR_I2C_CLOCK_SOURCE    0 /* 0:clock controller, 1:PCLK */
#define I2C_XACT_DELAY_MS       1

#define SENSOR_OK           0
#define SENSOR_FAIL        -1

nrc_err_t td_init_air_quality_hat(void);
float td_get_air_quality_temperature(void);
float td_get_air_quality_humidity(void);
float td_get_air_quality_co2(void);
float td_get_air_quality_voc(void);
nrc_err_t td_set_absolute_humidity(float temp, float humidity);
#endif /* TELEDATICS_AIR_QUALITY_H */
