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
** Pragmas
*************************************************************************/

/************************************************************************
** Includes
*************************************************************************/
#include <string.h>
#include "cfe.h"
#include "bmp280_custom.h"
#include "bmp280_app.h"
#include "bmp280_msg.h"
#include "bmp280_version.h"
#include <math.h>
#include "px4lib.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Instantiate the application object.                             */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
BMP280 oBMP280;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Default constructor.                                            */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
BMP280::BMP280()
{

}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Destructor constructor.                                         */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
BMP280::~BMP280()
{

}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Initialize event tables.                                        */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int32 BMP280::InitEvent()
{
    int32  iStatus         = CFE_SUCCESS;
    int32  ind             = 0;
    int32 customEventCount = 0;
    
    CFE_EVS_BinFilter_t   EventTbl[CFE_EVS_MAX_EVENT_FILTERS];

    /* Initialize the event filter table.
     * Note: 0 is the CFE_EVS_NO_FILTER mask and event 0 is reserved (not used) */
    memset(EventTbl, 0x00, sizeof(EventTbl));

    /* CFE_EVS_MAX_EVENT_FILTERS limits the number of filters per app. */
    /* Add platform independent events to filter */
    EventTbl[  ind].EventID = BMP280_RESERVED_EID;
    EventTbl[ind++].Mask    = CFE_EVS_NO_FILTER;
    EventTbl[  ind].EventID = BMP280_READ_ERR_EID;
    EventTbl[ind++].Mask    = CFE_EVS_FIRST_16_STOP;

    /* Add custom events to the filter table */
    customEventCount = BMP280_Custom_Init_EventFilters(ind, EventTbl);
    
    if(-1 == customEventCount)
    {
        iStatus = CFE_EVS_APP_FILTER_OVERLOAD;
        (void) CFE_ES_WriteToSysLog("Failed to init custom event filters (0x%08X)\n", (unsigned int)iStatus);
        goto end_of_function;
    }

    /* Register the table with CFE */
    iStatus = CFE_EVS_Register(EventTbl, (ind + customEventCount), CFE_EVS_BINARY_FILTER);
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_ES_WriteToSysLog("BMP280 - Failed to register with EVS (0x%08lX)\n", iStatus);
    }

end_of_function:

    return (iStatus);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Initialize Message Pipes                                        */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int32 BMP280::InitPipe()
{
    int32  iStatus = CFE_SUCCESS;

    /* Init schedule pipe and subscribe to wakeup messages */
    iStatus = CFE_SB_CreatePipe(&SchPipeId,
            BMP280_SCH_PIPE_DEPTH,
            BMP280_SCH_PIPE_NAME);
    if (iStatus == CFE_SUCCESS)
    {
        iStatus = CFE_SB_SubscribeEx(BMP280_MEASURE_MID, SchPipeId, 
                CFE_SB_Default_Qos, BMP280_MEASURE_MID_MAX_MSG_COUNT);
        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(BMP280_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
                    "Sch Pipe failed to subscribe to BMP280_MEASURE_MID. (0x%08lX)",
                    iStatus);
            goto BMP280_InitPipe_Exit_Tag;
        }

        iStatus = CFE_SB_SubscribeEx(BMP280_SEND_HK_MID, SchPipeId, 
                CFE_SB_Default_Qos, BMP280_SEND_HK_MID_MAX_MSG_COUNT);
        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(BMP280_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
                     "CMD Pipe failed to subscribe to BMP280_SEND_HK_MID. (0x%08X)",
                     (unsigned int)iStatus);
            goto BMP280_InitPipe_Exit_Tag;
        }
    }
    else
    {
        (void) CFE_EVS_SendEvent(BMP280_PIPE_INIT_ERR_EID, CFE_EVS_ERROR,
             "Failed to create SCH pipe (0x%08lX)",
             iStatus);
        goto BMP280_InitPipe_Exit_Tag;
    }

    /* Init command pipe and subscribe to command messages */
    iStatus = CFE_SB_CreatePipe(&CmdPipeId,
            BMP280_CMD_PIPE_DEPTH,
            BMP280_CMD_PIPE_NAME);
    if (iStatus == CFE_SUCCESS)
    {
        /* Subscribe to command messages */
        iStatus = CFE_SB_Subscribe(BMP280_CMD_MID, CmdPipeId);

        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(BMP280_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
                 "CMD Pipe failed to subscribe to BMP280_CMD_MID. (0x%08lX)",
                 iStatus);
            goto BMP280_InitPipe_Exit_Tag;
        }
    }
    else
    {
        (void) CFE_EVS_SendEvent(BMP280_PIPE_INIT_ERR_EID, CFE_EVS_ERROR,
             "Failed to create CMD pipe (0x%08lX)",
             iStatus);
        goto BMP280_InitPipe_Exit_Tag;
    }

BMP280_InitPipe_Exit_Tag:
    return (iStatus);
}
    

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Initialize Global Variables                                     */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void BMP280::InitData()
{
    /* Init housekeeping message. */
    CFE_SB_InitMsg(&m_HkTlm,
            BMP280_HK_TLM_MID, sizeof(m_HkTlm), TRUE);
    /* Init output messages */
    CFE_SB_InitMsg(&m_SensorBaro,
            PX4_SENSOR_BARO_MID, sizeof(PX4_m_SensorBaroMsg_t), TRUE);
    /* Init diagnostic message */
    CFE_SB_InitMsg(&m_Diag,
            BMP280_DIAG_TLM_MID, sizeof(BMP280_m_DiagPacket_t), TRUE);
    /* Init custom data */
    BMP280_Custom_InitData();
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* BMP280 initialization                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int32 BMP280::InitApp()
{
    int32  iStatus      = CFE_SUCCESS;
    int8   hasEvents    = 0;
    boolean returnBool  = TRUE;
    int32 i             = 0;
    
    iStatus = InitEvent();
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_ES_WriteToSysLog("BMP280 - Failed to init events (0x%08lX)\n", iStatus);
        goto BMP280_InitApp_Exit_Tag;
    }
    else
    {
        hasEvents = 1;
    }

    iStatus = InitPipe();
    if (iStatus != CFE_SUCCESS)
    {
        goto BMP280_InitApp_Exit_Tag;
    }

    iStatus = InitConfigTbl();
    if (iStatus != CFE_SUCCESS)
    {
        goto BMP280_InitApp_Exit_Tag;
    }

    /* Init data include copy params from config table */
    InitData();

    returnBool = BMP280_Custom_Init();
    if (FALSE == returnBool)
    {
        iStatus = -1;
        goto BMP280_InitApp_Exit_Tag;
    }

    /* Get device calibration. */
    returnBool = BMP280_Custom_ReadCalibration(&m_Calibration);

    if (FALSE == returnBool)
    {
        iStatus = -1;
        (void) CFE_EVS_SendEvent(BMP280_INIT_ERR_EID, CFE_EVS_ERROR,
            "BMP280 failed read device calibration.");
        goto BMP280_InitApp_Exit_Tag; 
    }

    /* Register the cleanup callback */
    iStatus = OS_TaskInstallDeleteHandler(&BMP280_CleanupCallback);
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_EVS_SendEvent(BMP280_INIT_ERR_EID, CFE_EVS_ERROR,
                                 "Failed to init register cleanup callback (0x%08X)",
                                 (unsigned int)iStatus);
        goto BMP280_InitApp_Exit_Tag;
    }

    /* Set application state to initialized */
    m_HkTlm.State = BMP280_INITIALIZED;

BMP280_InitApp_Exit_Tag:
    if (iStatus == CFE_SUCCESS)
    {
        (void) CFE_EVS_SendEvent(BMP280_INIT_INF_EID, CFE_EVS_INFORMATION,
                                 "Initialized.  Version %d.%d.%d.%d",
                                 BMP280_MAJOR_VERSION,
                                 BMP280_MINOR_VERSION,
                                 BMP280_REVISION,
                                 BMP280_MISSION_REV);
    }
    else
    {
        if (hasEvents == 1)
        {
            (void) CFE_ES_WriteToSysLog("BMP280 - Application failed to initialize\n");
        }
    }

    return (iStatus);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Receive and Process Messages                                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int32 BMP280::RcvSchPipeMsg(int32 iBlocking)
{
    int32           iStatus = CFE_SUCCESS;
    CFE_SB_Msg_t*   MsgPtr  = NULL;
    CFE_SB_MsgId_t  MsgId   = 0;

    /* Stop Performance Log entry */
    CFE_ES_PerfLogExit(BMP280_MAIN_TASK_PERF_ID);

    /* Wait for WakeUp messages from scheduler */
    iStatus = CFE_SB_RcvMsg(&MsgPtr, SchPipeId, iBlocking);

    /* Start Performance Log entry */
    CFE_ES_PerfLogEntry(BMP280_MAIN_TASK_PERF_ID);

    if (iStatus == CFE_SUCCESS)
    {
        MsgId = CFE_SB_GetMsgId(MsgPtr);
        switch (MsgId)
        {
            case BMP280_SEND_HK_MID:
            {
                ProcessCmdPipe();
                ReportHousekeeping();
                break;
            }
            case BMP280_MEASURE_MID:
            {
                ReadDevice();
                break;
            }
            default:
            {
                (void) CFE_EVS_SendEvent(BMP280_MSGID_ERR_EID, CFE_EVS_ERROR,
                        "Recvd invalid SCH msgId (0x%04X)", MsgId);
            }
        }
    }
    else if (iStatus == CFE_SB_NO_MESSAGE)
    {
        /* If there's no incoming message, do nothing. 
         *  Note, this section is dead code only if the iBlocking arg
         * is CFE_SB_PEND_FOREVER. */
        iStatus = CFE_SUCCESS;
    }
    else if (iStatus == CFE_SB_TIME_OUT)
    {
        /* If there's no incoming message within a specified time (via the
         * iBlocking arg, do nothing.
         * Note, this section is dead code only if the iBlocking arg
         * is CFE_SB_PEND_FOREVER. */
        iStatus = CFE_SUCCESS;
    }
    else
    {
        (void) CFE_EVS_SendEvent(BMP280_RCVMSG_ERR_EID, CFE_EVS_ERROR,
                "SCH pipe read error (0x%08lX).", iStatus);
    }

    return (iStatus);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Process Incoming Commands                                       */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void BMP280::ProcessCmdPipe()
{
    int32 iStatus             = CFE_SUCCESS;
    CFE_SB_Msg_t*   CmdMsgPtr = NULL;
    CFE_SB_MsgId_t  CmdMsgId  = 0;

    /* Process command messages until the pipe is empty */
    while (1)
    {
        iStatus = CFE_SB_RcvMsg(&CmdMsgPtr, CmdPipeId, CFE_SB_POLL);
        if(iStatus == CFE_SUCCESS)
        {
            CmdMsgId = CFE_SB_GetMsgId(CmdMsgPtr);
            switch (CmdMsgId)
            {
                case BMP280_CMD_MID:
                    ProcessAppCmds(CmdMsgPtr);
                    break;

                default:
                    /* Bump the command error counter for an unknown command.
                     * (This should only occur if it was subscribed to with this
                     *  pipe, but not handled in this switch-case.) */
                    m_HkTlm.usCmdErrCnt++;
                    (void) CFE_EVS_SendEvent(BMP280_MSGID_ERR_EID, CFE_EVS_ERROR,
                                "Recvd invalid CMD msgId (0x%04X)", (unsigned short)CmdMsgId);
                    break;
            }
        }
        else if (iStatus == CFE_SB_NO_MESSAGE)
        {
            break;
        }
        else
        {
            (void) CFE_EVS_SendEvent(BMP280_RCVMSG_ERR_EID, CFE_EVS_ERROR,
                  "CMD pipe read error (0x%08lX)", iStatus);
            break;
        }
    }
    return;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Process BMP280 Commands                                         */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void BMP280::ProcessAppCmds(CFE_SB_Msg_t* MsgPtr)
{
    uint32  uiCmdCode = 0;

    if (MsgPtr != NULL)
    {
        uint16 ExpectedLength = sizeof(BMP280_NoArgCmd_t);
        /* Length verification is then the same for all three commands */
        if (VerifyCmdLength(MsgPtr, ExpectedLength))
        {
            uiCmdCode = CFE_SB_GetCmdCode(MsgPtr);
            switch (uiCmdCode)
            {
                case BMP280_NOOP_CC:
                {
                    m_HkTlm.usCmdCnt++;
                    (void) CFE_EVS_SendEvent(BMP280_CMD_NOOP_EID, CFE_EVS_INFORMATION,
                        "Recvd NOOP. Version %d.%d.%d.%d",
                        BMP280_MAJOR_VERSION,
                        BMP280_MINOR_VERSION,
                        BMP280_REVISION,
                        BMP280_MISSION_REV);
                    break;
                }
                case BMP280_RESET_CC:
                {
                    m_HkTlm.usCmdCnt = 0;
                    m_HkTlm.usCmdErrCnt = 0;
                    break;
                }
                case BMP280_SEND_DIAG_TLM_CC:
                {
                    m_HkTlm.usCmdCnt++;
                    Reportm_Diagnostic();
                    break;
                }
                default:
                {
                    m_HkTlm.usCmdErrCnt++;
                    (void) CFE_EVS_SendEvent(BMP280_CC_ERR_EID, CFE_EVS_ERROR,
                                      "Recvd invalid command code (%u)", (unsigned int)uiCmdCode);
                    break;
                }
            }
        }
    }
    return;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Send BMP280 Housekeeping                                        */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void BMP280::ReportHousekeeping()
{
    CFE_SB_TimeStampMsg((CFE_SB_Msg_t*)&m_HkTlm);
    CFE_SB_SendMsg((CFE_SB_Msg_t*)&m_HkTlm);
    return;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Send BMP280 m_Diagnostic                                          */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void BMP280::Reportm_Diagnostic()
{
    CFE_SB_TimeStampMsg((CFE_SB_Msg_t*)&m_Diag);
    CFE_SB_SendMsg((CFE_SB_Msg_t*)&m_Diag);
    return;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Verify Command Length                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
boolean BMP280::VerifyCmdLength(CFE_SB_Msg_t* MsgPtr,
                           uint16 usExpectedLen)
{
    boolean bResult  = TRUE;
    uint16  usMsgLen = 0;

    if (MsgPtr != NULL)
    {
        usMsgLen = CFE_SB_GetTotalMsgLength(MsgPtr);

        if (usExpectedLen != usMsgLen)
        {
            bResult = FALSE;
            CFE_SB_MsgId_t MsgId = CFE_SB_GetMsgId(MsgPtr);
            uint16 usCmdCode = CFE_SB_GetCmdCode(MsgPtr);

            (void) CFE_EVS_SendEvent(BMP280_MSGLEN_ERR_EID, CFE_EVS_ERROR,
                              "Rcvd invalid msgLen: msgId=0x%08X, cmdCode=%d, "
                              "msgLen=%d, expectedLen=%d",
                              MsgId, usCmdCode, usMsgLen, usExpectedLen);
            m_HkTlm.usCmdErrCnt++;
        }
    }

    return (bResult);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* BMP280 Application C style main entry point.                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern "C" void BMP280_AppMain()
{
    oBMP280.AppMain();
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* BMP280 Application C++ style main entry point.                  */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void BMP280::AppMain()
{
    /* Register the application with Executive Services */
    uiRunStatus = CFE_ES_APP_RUN;

    int32 iStatus = CFE_ES_RegisterApp();
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_ES_WriteToSysLog("BMP280 - Failed to register the app (0x%08lX)\n", iStatus);
    }

    /* Start Performance Log entry */
    CFE_ES_PerfLogEntry(BMP280_MAIN_TASK_PERF_ID);

    /* Perform application initializations */
    if (iStatus == CFE_SUCCESS)
    {
        iStatus = InitApp();
    }

    if (iStatus == CFE_SUCCESS)
    {
        /* Do not perform performance monitoring on startup sync */
        CFE_ES_PerfLogExit(BMP280_MAIN_TASK_PERF_ID);
        CFE_ES_WaitForStartupSync(BMP280_STARTUP_TIMEOUT_MSEC);
        CFE_ES_PerfLogEntry(BMP280_MAIN_TASK_PERF_ID);
    }
    else
    {
        uiRunStatus = CFE_ES_APP_ERROR;
    }

    /* Application main loop */
    while (CFE_ES_RunLoop(&uiRunStatus) == TRUE)
    {
        RcvSchPipeMsg(BMP280_SCH_PIPE_PEND_TIME);
        iStatus = AcquireConfigPointers();
        if(iStatus != CFE_SUCCESS)
        {
            /* We apparently tried to load a new table but failed.  Terminate the application. */
            uiRunStatus = CFE_ES_APP_ERROR;
        }
    }

    /* Pre-normal exit cleanup */
    CleanupExit();

    /* Stop Performance Log entry */
    CFE_ES_PerfLogExit(BMP280_MAIN_TASK_PERF_ID);

    /* Exit the application */
    CFE_ES_ExitApp(uiRunStatus);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Read from the device and convert to altitude                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void BMP280::ReadDevice(void)
{
    uint32 rawPressure      = 0;
    uint32 rawTemperature   = 0;
    int32  fineTemperature  = 0;
    boolean returnBool  = TRUE;
    uint32 compPressure     = 0;

    returnBool = BMP280_Custom_Measure(&rawPressure, &rawTemperature);
    if (FALSE == returnBool)
    {
        (void) CFE_EVS_SendEvent(BMP280_READ_ERR_EID, CFE_EVS_ERROR,
                "BMP280 read failure, altitude not updated");
        goto end_of_function;
    }
    else
    {
        /* Stamp time */
        m_SensorBaro.Timestamp = PX4LIB_GetPX4TimeUs();
        /* Convert raw temperature to Celsius. */
        m_SensorBaro.Temperature = BMP280_Compensate_Temperature(rawTemperature,
                &fineTemperature, &m_calibration) / 100.0f;
        /* Convert raw pressure (lvalue is Pa). */
        compPressure = BMP280_Compensate_Pressure(rawPressure, 
                fineTemperature, &m_calibration) / 256.0f;
        /* Convert to millibar. */
        m_SensorBaro.Pressure = compPressure / 100.0f;
    
        /* tropospheric properties (0-11km) for standard atmosphere */
        /* temperature at base height in Kelvin */
        const double T1 = 15.0 + 273.15;
        /* temperature gradient in degrees per metre */
        const double a  = -6.5 / 1000;
        /* gravity constant in m/s/s */
        const double g  = 9.80665;
        /* ideal gas constant in J/kg/K */
        const double R  = 287.05;
        /* measured pressure in kPa */
        double p = static_cast<double>(compPressure) / 1000.0;

        /* current pressure at MSL in kPa double p1 = 101325 / 1000.0 */

        /*
         * Solve:
         *
         *     /        -(aR / g)     \
         *    | (p / p1)          . T1 | - T1
         *     \                      /
         * h = -------------------------------  + h1
         *                   a
         */
        m_SensorBaro.Altitude = (((pow((p / ConfigTblPtr->p1), (-(a * R) / g))) * T1) - T1) / a;
    
        /* Update diagnostic message */
        m_Diag.Pressure    = m_SensorBaro.Pressure;
        m_Diag.Temperature = m_SensorBaro.Temperature;
        m_Diag.Altitude    = m_SensorBaro.Altitude;
        
        /* Send the m_SensorBaro message */
        CFE_SB_TimeStampMsg((CFE_SB_Msg_t*)&m_SensorBaro);
        CFE_SB_SendMsg((CFE_SB_Msg_t*)&m_SensorBaro);
    }

end_of_function:
    return;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Normal application exit                                         */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void BMP280::CleanupExit(void)
{
    m_HkTlm.State = BMP280_UNINITIALIZED;
    if(BMP280_Custom_Uninit() != TRUE)
    {
        CFE_EVS_SendEvent(BMP280_UNINIT_ERR_EID, CFE_EVS_ERROR,"BMP280_Uninit failed");
        m_HkTlm.State = BMP280_INITIALIZED;
    }
    return;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Cleanup before exit                                             */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void BMP280_CleanupCallback(void)
{
    BMP280_Critical_Cleanup();
}


/************************/
/*  End of File Comment */
/************************/
