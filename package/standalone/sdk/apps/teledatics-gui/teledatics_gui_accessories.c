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
 * @file teledatics_gui_accessories.c
 * @author James Ewing
 * @date 8 Apr 2022
 * @brief Teledatics Accessory Routines
 */

#include "teledatics_gui.h"
#include "teledatics_gui_air_quality.h"
 
/**
 * @brief Identify attached accessories
 * 
 * Identify and populate accessories config indicator
 * 
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t td_init_accessories(td_wifi_config_t* tf_config)
{
        nrc_usr_print ("[%s]\n", __func__);
        
        if(!tf_config)
                return NRC_FAIL;
        
        tf_config->accessories = 0;
        
        if(td_init_air_quality_hat() == NRC_SUCCESS) {
                nrc_usr_print ("[%s] found air quality sensor\n", __func__);
                tf_config->accessories |= HAVE_AIR_QUALITY_SENSOR_HAT;
        }
        
        return NRC_SUCCESS;
}
