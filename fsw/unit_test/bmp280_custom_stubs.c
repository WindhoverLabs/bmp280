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

#include "bmp280_custom_stubs.h"
#include "bmp280_utils.h"


void BMP280_Custom_InitData(void)
{
    return;
}


boolean BMP280_Custom_Init(void)
{
    return TRUE;
}


boolean BMP280_Custom_Uninit(void)
{
    return TRUE;
}


void BMP280_Critical_Cleanup(void)
{
    return;
}


boolean BMP280_Custom_Measure(uint32 *RawPressure, uint32 *RawTemperature)
{
    *RawTemperature = 519888;
    *RawPressure    = 415148;
    return TRUE;
}


boolean BMP280_Custom_ReadCalibration(BMP280_Calibration_t *cal)
{
    /* Example values from BMP280 data sheet */
    cal->dig_T1 = 27504;
    cal->dig_T2 = 26435;
    cal->dig_T3 = -1000;
    cal->dig_P1 = 36477;
    cal->dig_P2 = -10685;
    cal->dig_P3 = 3024;
    cal->dig_P4 = 2855;
    cal->dig_P5 = 140;
    cal->dig_P6 = -7;
    cal->dig_P7 = 15500;
    cal->dig_P8 = -14600;
    cal->dig_P9 = 6000;

    return TRUE;
}


int32 BMP280_Custom_Init_EventFilters(int32 ind, CFE_EVS_BinFilter_t *EventTbl)
{
    return 0;
}


void BMP280_Custom_Critical_Cleanup(void)
{
    return;
}


uint64 PX4LIB_GetPX4TimeUs(void)
{
    return 0;
}
