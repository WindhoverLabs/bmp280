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

#ifndef BMP280_CUSTOM_H
#define BMP280_CUSTOM_H

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************************
** Pragmas
*************************************************************************/

/************************************************************************
** Includes
*************************************************************************/
#include "bmp280_utils.h"

/************************************************************************
** Local Defines
*************************************************************************/

/************************************************************************
** Local Structure Definitions
*************************************************************************/

/************************************************************************
** External Global Variables
*************************************************************************/

/************************************************************************
** Function Prototypes
*************************************************************************/

/************************************************************************/
/** \brief Custom function to initialize custom device data structure. 
**
**  \par Description
**       This function is called on app startup, reload, restart etc
**       to initialize non-zero data.
**
*************************************************************************/
void BMP280_Custom_InitData(void);


/************************************************************************/
/** \brief Custom function to initialize custom device(s).
**
**  \par Description
**       This function is called at initialization and allows the
**       custom layer to provide specific functionality to initialize
**       internal objects.
**
**  \returns
**  TRUE if successful, FALSE otherwise.
**  \endreturns
**
*************************************************************************/
boolean BMP280_Custom_Init(void);


/************************************************************************/
/** \brief Custom function to uninitialize custom device(s).
**
**  \par Description
**       This function is called in preparation for loading a new
**       configuration, allowing the custom layer to do whatever it
**       needs with the current configuration before reconfiguration,
**       if anything. Also, called in cleanup to close and uninitialize
**       device resources.
**
**  \returns
**  TRUE if successful, FALSE otherwise.
**  \endreturns
**
*************************************************************************/
boolean BMP280_Custom_Uninit(void);


/************************************************************************/
/** \brief Custom function to initialize custom events. 
**
**  \par Description
**       This function is called in init event before CFE_EVS_Register
**       to add custom events to the event filter table.
**
**  \par Assumptions, External Events, and Notes:
**       This function must be defined, but not all custom
**       layers will do anything in this function.
**
**  \returns
**       The number of events written to the filter table and -1 for 
**       failure i.e. CFE_EVS_MAX_EVENT_FILTERS reached.
**
*************************************************************************/
int32 BMP280_Custom_Init_EventFilters(int32 ind, CFE_EVS_BinFilter_t *EventTbl);


/************************************************************************/
/** \brief Read the BMP280 factory calibration.
**
**  \par Description
**       This function reads the BMP280 factory calibration data.
**
**  \par Assumptions, External Events, and Notes:
**       This function must be called after initialization and before
**       reading sensor data to get the specific factory calibration.
**
**  \param [in/out]   cal    calibration.
**
**  \returns    TRUE for success, FALSE for failure.
**
*************************************************************************/
boolean BMP280_Custom_ReadCalibration(BMP280_Calibration_t *cal);


/************************************************************************/
/** \brief Read raw pressure and raw temperature data from the BMP280.
**
**  \par Description
**       Gets a raw pressure and temperature measurement from the
**       BMP280.
**
**  \par Assumptions, External Events, and Notes:
**       This function must be called after initialization.
**
**  \param [in/out]   RawPressure       raw pressure.
**
**  \param [in/out]   RawTemperature    raw temperature.
**
**  \returns    TRUE for success, FALSE for failure.
**
*************************************************************************/
boolean BMP280_Custom_Measure(uint32 *RawPressure, uint32 *RawTemperature);


/************************************************************************/
/** \brief Custom function to perform any critical cleanup after a 
**         failure.
**
**  \par Description
**       This function is called in cleanup to close and uninitialize
**       device resources.
**
**  \par Assumptions, External Events, and Notes:
**       This function must be defined, but not all custom
**       layers will do anything in this function.
**
*************************************************************************/
void BMP280_Custom_Critical_Cleanup(void);


#ifdef __cplusplus
}
#endif 

#endif /* BMP280_CUSTOM_H */
