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

/************************************************************************
** Includes
*************************************************************************/
#include "bmp280_utils.h"
#include "cfe.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Convert temperature.                                            */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int32 BMP280_Compensate_Temperature(int32 adc_T, int32 *t_fine, BMP280_Calibration_t *cal)
{
    int32 var1, var2, T;

    if(cal == 0 || t_fine == 0)
    {
        T = 0;
        goto end_of_function;
    }

    var1 = ((((adc_T >> 3) - ((int32) cal->dig_T1 << 1)))
            * ((int32) cal->dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32) cal->dig_T1))
        * ((adc_T >> 4) - ((int32) cal->dig_T1)))
        >> 12) * ((int32) cal->dig_T3)) >> 14;

    *t_fine = var1 + var2;

    T = (*t_fine * 5 + 128) >> 8;

end_of_function:
    return T;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Convert pressure.                                               */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
uint32 BMP280_Compensate_Pressure(int64 adc_P, int32 t_fine, BMP280_Calibration_t *cal)
{
    int64 var1, var2, p;

    if(cal == 0)
    {
        p = 0;
        goto end_of_function;
    }

    var1 = ((int64) t_fine) - 128000;
    var2 = var1 * var1 * (int64) cal->dig_P6;
    var2 = var2 + ((var1 * (int64) cal->dig_P5) << 17);
    var2 = var2 + (((int64) cal->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64) cal->dig_P3) >> 8)
            + ((var1 * (int64) cal->dig_P2) << 12);
    var1 = (((((int64) 1) << 47) + var1) * ((int64) cal->dig_P1)) >> 33;

    if (var1 == 0) 
    {
        /* avoid exception caused by division by zero */
        p = 0;
        goto end_of_function;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64) cal->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64) cal->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64) cal->dig_P7) << 4);

end_of_function:
    return (uint32)p;
}

