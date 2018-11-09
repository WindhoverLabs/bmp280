/****************************************************************************
*
*   Copyright (c) 2018 Windhover Labs, L.L.C. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name Windhover Labs nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*****************************************************************************/

#ifndef BMP280_DEFINES_H
#define BMP280_DEFINES_H

/* BMP280 register map. */
/** \brief BMP280 calibration parameter dig_t1 LSB. */
#define BMP280_CAL_DIG_T1_LSB   (0x88)
/** \brief BMP280 calibration parameter dig_t1 MSB. */
#define BMP280_CAL_DIG_T1_MSB   (0x89)
/** \brief BMP280 calibration parameter dig_t2 LSB. */
#define BMP280_CAL_DIG_T2_LSB   (0x8A)
/** \brief BMP280 calibration parameter dig_t2 MSB. */
#define BMP280_CAL_DIG_T2_MSB   (0x8B)
/** \brief BMP280 calibration parameter dig_t3 LSB. */
#define BMP280_CAL_DIG_T3_LSB   (0x8C)
/** \brief BMP280 calibration parameter dig_t3 MSB. */
#define BMP280_CAL_DIG_T3_MSB   (0x8D)
/** \brief BMP280 calibration parameter dig_p1 LSB. */
#define BMP280_CAL_DIG_P1_LSB   (0x8E)
/** \brief BMP280 calibration parameter dig_p1 MSB. */
#define BMP280_CAL_DIG_P1_MSB   (0x8F)
/** \brief BMP280 calibration parameter dig_t2 LSB. */
#define BMP280_CAL_DIG_P2_LSB   (0x90)
/** \brief BMP280 calibration parameter dig_p2 MSB. */
#define BMP280_CAL_DIG_P2_MSB   (0x91)
/** \brief BMP280 calibration parameter dig_p3 LSB. */
#define BMP280_CAL_DIG_P3_LSB   (0x92)
/** \brief BMP280 calibration parameter dig_p3 MSB. */
#define BMP280_CAL_DIG_P3_MSB   (0x93)
/** \brief BMP280 calibration parameter dig_p4 MSB. */
#define BMP280_CAL_DIG_P4_LSB   (0x94)
/** \brief BMP280 calibration parameter dig_p4 MSB. */
#define BMP280_CAL_DIG_P4_MSB   (0x95)
/** \brief BMP280 calibration parameter dig_p5 LSB. */
#define BMP280_CAL_DIG_P5_LSB   (0x96)
/** \brief BMP280 calibration parameter dig_p5 MSB. */
#define BMP280_CAL_DIG_P5_MSB   (0x97)
/** \brief BMP280 calibration parameter dig_p6 LSB. */
#define BMP280_CAL_DIG_P6_LSB   (0x98)
/** \brief BMP280 calibration parameter dig_p6 MSB. */
#define BMP280_CAL_DIG_P6_MSB   (0x99)
/** \brief BMP280 calibration parameter dig_p7 LSB. */
#define BMP280_CAL_DIG_P7_LSB   (0x9A)
/** \brief BMP280 calibration parameter dig_p7 MSB. */
#define BMP280_CAL_DIG_P7_MSB   (0x9B)
/** \brief BMP280 calibration parameter dig_p8 LSB. */
#define BMP280_CAL_DIG_P8_LSB   (0x9C)
/** \brief BMP280 calibration parameter dig_p8 MSB. */
#define BMP280_CAL_DIG_P8_MSB   (0x9D)
/** \brief BMP280 calibration parameter dig_p9 LSB. */
#define BMP280_CAL_DIG_P9_LSB   (0x9E)
/** \brief BMP280 calibration parameter dig_p9 MSB. */
#define BMP280_CAL_DIG_P9_MSB   (0x9F)
/** \brief BMP280 register chip identification number. */
#define BMP280_REG_ID           (0xD0)
/** \brief BMP280 register reset. */
#define BMP280_REG_RESET        (0xE0)
/** \brief BMP280 register status. */
#define BMP280_REG_STATUS       (0xF3)
/** \brief BMP280 register data acquisition options. */
#define BMP280_REG_CTRL_MEAS    (0xF4)
/** \brief BMP280 register configuration options. */
#define BMP280_REG_CONFIG       (0xF5)
/** \brief BMP280 register raw pressure measurement MSB. */
#define BMP280_REG_PRESS_MSB    (0xF7)
/** \brief BMP280 register raw pressure measurement LSB. */
#define BMP280_REG_PRESS_LSB    (0xF8)
/** \brief BMP280 register raw pressure measurement XLSB. */
#define BMP280_REG_PRESS_XLSB   (0xF9)
/** \brief BMP280 register raw temperature measurement MSB. */
#define BMP280_REG_TEMP_MSB     (0xFA)
/** \brief BMP280 register raw temperature measurement LSB. */
#define BMP280_REG_TEMP_LSB     (0xFB)
/** \brief BMP280 register raw temperature measurement XLSB. */
#define BMP280_REG_TEMP_XLSB    (0xFC)

/* BMP280 bit mask. */
/** \brief BMP280 bits oversample temperature 2X. */
#define BMP280_BITS_CTRL_MEAS_OVERSAMPL_TEMP2X     (0b01000000)
/** \brief BMP280 bits oversample pressure 8X. */
#define BMP280_BITS_CTRL_MEAS_OVERSAMPL_PRESS8X    (0b00010000)
/** \brief BMP280 bits oversample pressure 16X. */
#define BMP280_BITS_CTRL_MEAS_OVERSAMPL_PRESS16X   (0b00010100)
/** \brief BMP280 bits power mode normal. */
#define BMP280_BITS_CTRL_MEAS_POWER_MODE_NORMAL    (0b00000011)
/** \brief BMP280 bits standby time 5ms. */
#define BMP280_BITS_CONFIG_STANDBY_0MS5            (0b00000000)
/** \brief BMP280 bits IIR filter off. */
#define BMP280_BITS_CONFIG_FILTER_OFF              (0b00000000)
/** \brief BMP280 bits SPI off. */
#define BMP280_BITS_CONFIG_SPI_OFF                 (0b00000000)

/* BMP280 general device defines. */
/** \brief BMP280 device ID to verify on I2C. */
#define BMP280_WHOAMI_ID                    (0x58)
/** \brief BMP280 I2C slave address. */
#define BMP280_I2C_SLAVE_ADDRESS            (0b1110110)

#endif /* BMP280_DEFINES_H */
