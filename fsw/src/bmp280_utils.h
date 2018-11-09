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


#ifndef BMP280_UTILS_H
#define BMP280_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************************
** Structure Definitions
*************************************************************************/

/**
 * \brief Factory calibration.
 */
typedef struct
{
    uint16 dig_T1;
    uint16 dig_P1;
    int16 dig_T2;
    int16 dig_T3;
    int16 dig_P2;
    int16 dig_P3;
    int16 dig_P4;
    int16 dig_P5;
    int16 dig_P6;
    int16 dig_P7;
    int16 dig_P8;
    int16 dig_P9;
} BMP280_Calibration_t;


/************************************************************************
** Function Definitions
*************************************************************************/


/************************************************************************/
/** \brief Returns temperature in degrees C.
 **
 **  \par Description
 **       From the BMP280 datasheet "Returns temperature in DegC, 
 **       resolution is 0.01 DegC.  Output value of “5123” equals 51.23 
 **       DegC."
 **
 **  \par Assumptions, External Events, and Notes:
 **       This function must be called after compensate temperature.
 **
 **  \param [in]       adc_P         Raw temperature value.
 **
 **  \param [in/out]   t_fine    Fine temperature.
 **
 **  \param [in/out]   cal       Factory calibration.
 **
 **  \returns
 **  Compensated temperature.
 **  \endreturns
 **
 *************************************************************************/
int32 BMP280_Compensate_Temperature(int32 adc_T, int32 *t_fine, BMP280_Calibration_t *cal);


/************************************************************************/
/** \brief Returns pressure in Pa.
 **
 **  \par Description
 **       From the BMP280 datasheet "Returns pressure in Pa as unsigned
 **       32 bit integer in Q24.8 format (24 integer bits and 8 
 **       fractional bits). Output value of “24674867” represents 
 **       24674867/256 = 96386.2 Pa = 963.862 hPa."
 **
 **  \par Assumptions, External Events, and Notes:
 **       This function must be called after compensate temperature.
 **
 **  \param [in]   adc_P         Raw pressure value.
 **
 **  \param [in]   t_fine        Temperature fine from the compensate 
 **                              temperature function.
 **
 **  \param [in/out]   cal       Factory calibration.
 **
 **  \returns
 **  Compensated pressure.
 **  \endreturns
 **
 *************************************************************************/
uint32 BMP280_Compensate_Pressure(int64 adc_P, int32 t_fine, BMP280_Calibration_t *cal);


#ifdef __cplusplus
}
#endif 

#endif /* BMP280_UTILS_H */

/************************/
/*  End of File Comment */
/************************/
