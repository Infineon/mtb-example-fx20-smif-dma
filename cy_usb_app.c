/***************************************************************************//**
 * \file cy_usb_app.c
 * \version 1.0
 *
 * Defines the interfaces used by the FX SMIF DMAC application.
 *
 *******************************************************************************
 * \copyright
 * (c) (2021-2025), Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/


#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usbss_cal_drv.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_usb_usbd.h"
#include "cy_usb_app.h"
#include "cy_debug.h"
#include "cy_qspi_mem.h"

__attribute__((section(".descSection"), used)) uint32_t SetSelDataBuffer[8] __attribute__((aligned(32)));

void UsbSSConnectionEnable (cy_stc_usb_app_ctxt_t *pAppCtxt);

#define CY_USB_TASK_MSG_SIZE (sizeof(cy_stc_usbd_app_msg_t))


/*
 * Function: Cy_USB_AppInit()
 * Description: This function Initializes application related data structures,
 *              register callback and creates queue and task for device
 *              function.
 * Parameter: cy_stc_usb_app_ctxt_t, cy_stc_usb_usbd_ctxt_t, DMAC_Type
 *            DW_Type, DW_Type, cy_stc_hbdma_mgr_context_t*
 * return: None.
 * Note: This function should be called after USBD_Init()
 */
void Cy_USB_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt,
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, DMAC_Type *pCpuDmacBase,
    DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base,
    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt)
{
  uint32_t index;
  BaseType_t status = pdFALSE;
  cy_stc_app_endp_dma_set_t *pEndpInDma;
  cy_stc_app_endp_dma_set_t *pEndpOutDma;

  pAppCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
  pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;
  pAppCtxt->devSpeed = CY_USBD_USB_DEV_FS;
  pAppCtxt->devAddr = 0x00;
  pAppCtxt->activeCfgNum = 0x00;
  pAppCtxt->prevAltSetting = 0x00;
  pAppCtxt->enumMethod = CY_USB_ENUM_METHOD_FAST;
  pAppCtxt->pHbDmaMgrCtxt = pHbDmaMgrCtxt;

  for (index=0x00; index < CY_USB_MAX_ENDP_NUMBER; index++) {
    pEndpInDma = &(pAppCtxt->endpInDma[index]);
    memset((void *)pEndpInDma, 0, sizeof(cy_stc_app_endp_dma_set_t));

    pEndpOutDma = &(pAppCtxt->endpOutDma[index]);
    memset((void *)pEndpOutDma, 0, sizeof(cy_stc_app_endp_dma_set_t));
  }

  pAppCtxt->pCpuDmacBase = pCpuDmacBase;
  pAppCtxt->pCpuDw0Base = pCpuDw0Base;
  pAppCtxt->pCpuDw1Base = pCpuDw1Base;
  pAppCtxt->pUsbdCtxt = pUsbdCtxt;

  /*
   * Callbacks registered with USBD layer. These callbacks will be called
   * based on appropriate event.
   */
  Cy_USB_AppRegisterCallback(pAppCtxt);

  if (!(pAppCtxt->firstInitDone))
  {
    /* Create queue and register it to kernel. */
    pAppCtxt->xQueue = xQueueCreate(24, CY_USB_TASK_MSG_SIZE);
    if (pAppCtxt->xQueue == NULL)
    {
      DBG_APP_ERR("QueuecreateFail\r\n");
      return;
    }

    DBG_APP_INFO("Created Queue\r\n");
    vQueueAddToRegistry(pAppCtxt->xQueue, "DeviceMsgQueue");

    /* Create task and check status to confirm task created properly. */
    status = xTaskCreate(Cy_USB_TaskHandler, "TaskHandler", 2048,
        (void *)pAppCtxt, 5, &(pAppCtxt->appTaskHandle));
    if (status != pdPASS)
    {
      DBG_APP_ERR("Taskcreate failed\r\n");
      return;
    }

    pAppCtxt->firstInitDone = 0x01;
  }

  return;
} /* end of function. */

/*
 * Function: Cy_USB_AppRegisterCallback()
 * Description: This function will register all calback with USBD layer.
 * Parameter: cy_stc_usb_app_ctxt_t.
 * return: None.
 */
void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt)
{
  cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = pAppCtxt->pUsbdCtxt;

  Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET,
      Cy_USB_AppBusResetCallback);
  Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET_DONE,
      Cy_USB_AppBusResetDoneCallback);
  Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_BUS_SPEED,
      Cy_USB_AppBusSpeedCallback);
  Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SETUP,
      Cy_USB_AppSetupCallback);
  Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SUSPEND,
      Cy_USB_AppSuspendCallback);
  Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESUME,
      Cy_USB_AppResumeCallback);
  Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_CONFIG,
      Cy_USB_AppSetCfgCallback);
  Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_INTF,
      Cy_USB_AppSetIntfCallback);
  Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_L1_SLEEP,
      Cy_USB_AppL1SleepCallback);
  Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_L1_RESUME,
      Cy_USB_AppL1ResumeCallback);
  Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_ZLP,
      Cy_USB_AppZlpCallback);
  Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SLP,
      Cy_USB_AppSlpCallback);
  return;
} /* end of function. */

/*
 * Function: Cy_USB_AppConfigureEndps()
 * Description: This Function is used by application to configure endpoints
 *              after set configuration.  This function should be used for
 *              all endpoints except endp0.
 * Parameter: cy_stc_usb_usbd_ctxt_t, pEndpDscr
 * return: void
 */
void Cy_USB_AppConfigureEndp(cy_stc_usb_usbd_ctxt_t * pUsbdCtxt, uint8_t * pEndpDscr)
{
  cy_stc_usb_endp_config_t endpConfig;
  cy_en_usb_endp_dir_t endpDirection;
  bool valid;
  uint32_t endpType;
  uint8_t endpInterval;
  uint32_t endpNumber, dir;
  uint16_t maxPktSize;
  uint32_t isoPkts = 0x00;
  uint8_t burstSize = 0x00;
  uint8_t maxStream = 0x00;
  uint8_t *pCompDscr = NULL;
  cy_en_usbd_ret_code_t usbdRetCode;

  /* If it is not endpoint descriptor then return */
  if (!Cy_USBD_EndpDscrValid(pEndpDscr))
  {
    return;
  }
  Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &dir);

  if (dir)
  {
    endpDirection = CY_USB_ENDP_DIR_IN;
  }
  else
  {
    endpDirection = CY_USB_ENDP_DIR_OUT;
  }

  Cy_USBD_GetEndpType(pEndpDscr, &endpType);
  Cy_USBD_GetEndpInterval(pEndpDscr, &endpInterval);
  if ((CY_USB_ENDP_TYPE_ISO == endpType) || (CY_USB_ENDP_TYPE_INTR == endpType))
  {
    /* The ISOINPKS setting in the USBHS register is the actual packets per microframe value. */
    isoPkts = (
        (*((uint8_t *)(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_MAX_PKT + 1)) & CY_USB_ENDP_ADDL_XN_MASK)
        >> CY_USB_ENDP_ADDL_XN_POS) + 1;

  }

  valid = 0x01;
  if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS)
  {
    /* Get companion descriptor and from there get burstSize. */
    pCompDscr = Cy_USBD_GetSsEndpCompDscr(pUsbdCtxt, pEndpDscr);
    Cy_USBD_GetEndpCompnMaxburst(pCompDscr, &burstSize);
    Cy_USBD_GetEndpCompnMaxStream(pCompDscr, &maxStream);
  }

  /* Prepare endpointConfig parameter. */
  endpConfig.endpType = (cy_en_usb_endp_type_t)endpType;
  endpConfig.endpDirection = endpDirection;
  endpConfig.valid = valid;
  endpConfig.endpNumber = endpNumber;
  endpConfig.maxPktSize = (uint32_t)maxPktSize;
  endpConfig.isoPkts = isoPkts;
  endpConfig.burstSize = burstSize;
  endpConfig.streamID = maxStream;
  endpConfig.allowNakTillDmaRdy = false;
  endpConfig.interval = endpInterval;

  usbdRetCode = Cy_USB_USBD_EndpConfig(pUsbdCtxt, endpConfig);

  /* Print status of the endpoint configuration to help debug. */
  DBG_APP_INFO("Endpoint:%d status:%d\r\n", endpNumber, usbdRetCode);
  return;
} /* end of function */

/*
 * Function: Cy_USB_AppSetCfgCallback()
 * Description: This Function will be called by USBD  layer when
 *              set configuration command successful. This function
 *              does sanity check and prepare device for function
 *              to take over.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void Cy_USB_AppSetCfgCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
    cy_stc_usb_cal_msg_t *pMsg)
{
  cy_stc_usb_app_ctxt_t *pUsbApp;
  uint8_t *pActiveCfg, *pIntfDscr, *pEndpDscr;
  uint8_t index, numOfIntf, numOfEndp;
  cy_en_usb_speed_t devSpeed;

  DBG_APP_INFO("AppSetCfgCbStart\r\n");

  pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
  pUsbApp->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);
  devSpeed = pUsbApp->devSpeed;

  /*
   * Based on type of application as well as how data flows,
   * data wire can be used so initialize datawire.
   */
  Cy_DMA_Enable(pUsbApp->pCpuDw0Base);
  Cy_DMA_Enable(pUsbApp->pCpuDw1Base);

  pActiveCfg = Cy_USB_USBD_GetActiveCfgDscr(pUsbdCtxt);
  if (!pActiveCfg)
  {
    /* Set config should be called when active config value > 0x00. */
    return;
  }
  numOfIntf = Cy_USBD_FindNumOfIntf(pActiveCfg);
  if (numOfIntf == 0x00)
  {
    return;
  }

  for (index = 0x00; index < numOfIntf; index++)
  {
    /* During Set Config command always altSetting 0 will be active. */
    pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, index, 0x00);
    if (pIntfDscr == NULL)
    {
      DBG_APP_INFO("pIntfDscrNull\r\n");
      return;
    }

    numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
    if (numOfEndp == 0x00)
    {
      DBG_APP_INFO("numOfEndp 0\r\n");
      continue;
    }

    pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
    while (numOfEndp != 0x00)
    {
      Cy_USB_AppConfigureEndp(pUsbdCtxt, pEndpDscr);
      numOfEndp--;
      if (devSpeed > CY_USBD_USB_DEV_HS)
      {
        pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)) + 6);
      }
      else
      {
        pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
      }
    }
  }

  pUsbApp->prevDevState = CY_USB_DEVICE_STATE_CONFIGURED;
  pUsbApp->devState = CY_USB_DEVICE_STATE_CONFIGURED;
  DBG_APP_INFO("AppSetCfgCbEnd Done\r\n");
  return;
} /* end of function */

/*
 * Function: Cy_USB_AppBusResetCallback()
 * Description: This Function will be called by USBD when bus detects RESET.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void Cy_USB_AppBusResetCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
    cy_stc_usb_cal_msg_t *pMsg)
{
  cy_stc_usb_app_ctxt_t *pUsbApp;
  uint8_t i;

  pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

  DBG_APP_INFO("AppBusResetCallback\r\n");

  for (i = 0; i < CY_USB_MAX_ENDP_NUMBER; i++) {
    if (pUsbApp->endpInDma[i].valid) {
      pUsbApp->endpInDma[i].valid = false;
    }

    if (pUsbApp->endpOutDma[i].valid) {
      pUsbApp->endpOutDma[i].valid = false;
    }
  }

  /*
   * USBD layer takes care of reseting its own data structure as well as
   * takes care of calling CAL reset APIs. Application needs to take care
   * of reseting its own data structure as well as "device function".
   */
  Cy_USB_AppInit(pUsbApp, pUsbdCtxt, pUsbApp->pCpuDmacBase,
      pUsbApp->pCpuDw0Base, pUsbApp->pCpuDw1Base,
      pUsbApp->pHbDmaMgrCtxt);
  pUsbApp->devState = CY_USB_DEVICE_STATE_RESET;
  pUsbApp->prevDevState = CY_USB_DEVICE_STATE_RESET;

  if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
    Cy_USBSS_Cal_LPMDisable(pUsbdCtxt->pSsCalCtxt);
  }

  return;
} /* end of function. */

/*
 * Function: Cy_USB_AppBusResetDoneCallback()
 * Description: This Function will be called by USBD  layer when
 *              set configuration command successful. This function
 *              does sanity check and prepare device for function
 *              to take over.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void Cy_USB_AppBusResetDoneCallback(void *pAppCtxt,
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
    cy_stc_usb_cal_msg_t *pMsg)
{
  cy_stc_usb_app_ctxt_t *pUsbApp;

  DBG_APP_INFO("ppBusResetDoneCallback\r\n");

  pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
  pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
  pUsbApp->prevDevState = pUsbApp->devState;
  return;
} /* end of function. */

/*
 * Function: Cy_USB_AppBusSpeedCallback()
 * Description: This Function will be called by USBD  layer when
 *              speed is identified or speed change is detected.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void Cy_USB_AppBusSpeedCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
    cy_stc_usb_cal_msg_t *pMsg)
{
  cy_stc_usb_app_ctxt_t *pUsbApp;

  pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
  pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
  pUsbApp->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);
  return;
} /* end of function. */

/*
 * Function: Cy_USB_AppSetupCallback()
 * Description: This Function will be called by USBD  layer when
 *              set configuration command successful. This function
 *              does sanity check and prepare device for function
 *              to take over.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void Cy_USB_AppSetupCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
    cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    uint8_t bRequest, bReqType;
    uint8_t bType, bTarget;
    uint16_t wValue, wIndex, wLength;
    bool isReqHandled = false;
    __attribute__ ((aligned(4))) uint8_t deviceBusy = 1;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    cy_en_usb_endp_dir_t epDir = CY_USB_ENDP_DIR_INVALID;

    uint32_t loopCnt = 1000;
    uint32_t spiAddress = 0;
    uint32_t sector = 0;
    cy_en_smif_status_t spiStatus = CY_SMIF_SUCCESS;
    uint8_t fx10FlashProg[] = {'F','X','1','0','P','R','O','G'};

    DBG_APP_TRACE("AppSetupCallback\r\n");
    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    /* Fast enumeration is used. Only requests addressed to the interface, class,
     * vendor and unknown control requests are received by this function. */

    /* Decode the fields from the setup request. */
    bReqType = pUsbdCtxt->setupReq.bmRequest;
    bType = ((bReqType & CY_USB_CTRL_REQ_TYPE_MASK) >> CY_USB_CTRL_REQ_TYPE_POS);
    bTarget = (bReqType & CY_USB_CTRL_REQ_RECIPENT_OTHERS);
    bRequest = pUsbdCtxt->setupReq.bRequest;
    wValue = pUsbdCtxt->setupReq.wValue;
    wIndex = pUsbdCtxt->setupReq.wIndex;
    wLength = pUsbdCtxt->setupReq.wLength;

    if (bType == CY_USB_CTRL_REQ_STD)
    {
        /* Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND)
         * requests here. It should be allowed to pass if the device is in configured
         * state and failed otherwise. */
#if USE_WINUSB
        /* Handle Microsoft OS String Descriptor request. */
        if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE) &&
                (bRequest == CY_USB_SC_GET_DESCRIPTOR) &&
                (wValue == ((CY_USB_STRING_DSCR << 8) | 0xEE))) {

            /* Make sure we do not send more data than requested. */
            if (wLength > glOsString[0]) {
                wLength = glOsString[0];
            }

            DBG_APP_INFO("OSString\r\n");
            retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)glOsString, wLength);
            if(retStatus == CY_USBD_STATUS_SUCCESS) {
                isReqHandled = true;
            }
        }
#endif

        if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && ((bRequest == CY_USB_SC_SET_FEATURE) || (bRequest == CY_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
        {
            if (pUsbApp->devState == CY_USB_DEVICE_STATE_CONFIGURED)
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
            else
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);

            isReqHandled = true;
        }

        if ((bRequest == CY_USB_SC_SET_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) &&
                (wValue == CY_USB_FEATURE_ENDP_HALT))
        {
            epDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) : (CY_USB_ENDP_DIR_OUT));

            Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL),
                    epDir, true);

            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
            isReqHandled = true;
        }

        if ((bRequest == CY_USB_SC_CLEAR_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) &&
                (wValue == CY_USB_FEATURE_ENDP_HALT))
        {

            epDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) : (CY_USB_ENDP_DIR_OUT));

            Cy_USBD_FlushEndp(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL), epDir);

            Cy_USBD_ResetEndp(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL), epDir, false);
            Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL),
                    epDir, false);

            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
            isReqHandled = true;
        }

        DBG_APP_INFO("CY_USB_CTRL_REQ_STD\r\n");
    }

    if (bType == CY_USB_CTRL_REQ_VENDOR) {
#if USE_WINUSB
        /* Handle OS Compatibility and OS Feature requests */
        if (bRequest == MS_VENDOR_CODE) {
            if (wIndex == 0x04) {

                if (wLength > *((uint16_t*)glOsCompatibilityId)) {
                    wLength = *((uint16_t*)glOsCompatibilityId);
                }

                DBG_APP_INFO("OSCompat\r\n");
                retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)glOsCompatibilityId, wLength);
                if(retStatus == CY_USBD_STATUS_SUCCESS) {
                    isReqHandled = true;
                }
            }
            else if (wIndex == 0x05) {

                if (wLength > *((uint16_t*)glOsFeature)) {
                    wLength = *((uint16_t*)glOsFeature);
                }

                DBG_APP_INFO("OSFeature\r\n");
                retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)glOsFeature, wLength);
                if(retStatus == CY_USBD_STATUS_SUCCESS) {
                    isReqHandled = true;
                }
            }
        }
#endif
        if (bRequest == FLASH_CMD_CHECK_FLASHPROG)
        {
            retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, fx10FlashProg, sizeof(fx10FlashProg));
            ASSERT_NON_BLOCK(CY_USBD_STATUS_SUCCESS == retStatus, retStatus);
            isReqHandled = !retStatus;
        }

        /* Check memory busy status */
        if (bRequest == FLASH_CMD_CHECK_MEMBUSY)
        {
            if(Cy_SPI_IsDeviceBusy() == CY_SMIF_SUCCESS)
            {
                deviceBusy = 0;
            }
            retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt,&deviceBusy, 1);
            ASSERT_NON_BLOCK(CY_USBD_STATUS_SUCCESS == retStatus, retStatus);
            isReqHandled = !retStatus;
            if(!retStatus)
            {
                DBG_APP_TRACE("Device Busy?:(%d)\r\n",deviceBusy);
            }
        }


        /* Write to SPI flash. This version of the code example supports writes in multiples of
         * 64 Bytes to prevent handling of short packets in USB HS
         */
        if ((bRequest == FLASH_CMD_FLASH_WRITE) && (wLength <= SMIF_DMA_MAX_BUFFER_SIZE) &&
                ((wLength % USBHS_MAXPKT_SIZE) == 0))
        {

            spiAddress = wIndex * SMIF_DMA_MAX_BUFFER_SIZE;
            retStatus = Cy_USB_USBD_RecvEndp0Data(pUsbdCtxt, pUsbApp->spiHbwWriteBuffer, wLength);
            ASSERT_NON_BLOCK(CY_USBD_STATUS_SUCCESS == retStatus, retStatus);

            /* Wait until receive DMA transfer has been completed. */
            while ((!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) && (loopCnt--)) {
                Cy_SysLib_Delay(1);
            }

            if (!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) {
                Cy_USB_USBD_RetireRecvEndp0Data(pUsbdCtxt);
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, true);
                return;
            }

            if (retStatus == CY_USBD_STATUS_SUCCESS) {
                spiStatus = Cy_SPI_WriteOperation(spiAddress, pUsbApp->spiHbwWriteBuffer, wLength);
                ASSERT_NON_BLOCK(CY_SMIF_SUCCESS == spiStatus, spiStatus);
                if(!spiStatus)
                {
                    DBG_APP_TRACE("Write Pass:Address range:0x%x -> 0x%x\r\n", spiAddress, spiAddress + wLength - 1);
                    isReqHandled = true;
                }
                else
                {
                    DBG_APP_ERR("Write Fail:Address range:0x%x -> 0x%x\r\n", spiAddress, spiAddress + wLength - 1);
                }
            }
        }

        /* Read from SPI flash */
        if ((bRequest == FLASH_CMD_FLASH_READ) && (wLength <= SMIF_DMA_MAX_BUFFER_SIZE))
        {
            spiAddress = wIndex * SMIF_DMA_MAX_BUFFER_SIZE;
            memset(pUsbApp->spiHbwReadBuffer, 0, SMIF_DMA_MAX_BUFFER_SIZE);
            spiStatus = Cy_SPI_ReadOperation(spiAddress, pUsbApp->spiHbwReadBuffer , wLength);
            if(!spiStatus)
            {
                DBG_APP_TRACE("Read Pass: Address Range:0x%x->0x%x done\r\n",spiAddress, spiAddress + wLength-1);
            }
            else
            {
                DBG_APP_ERR("Read Fail: Address Range:0x%x->0x%x done\r\n",spiAddress, spiAddress + wLength-1);
            }

            /* Transfer to USB after DMA completion event is received */
            pUsbApp->spiHbwRxCount = wLength;

            ASSERT_NON_BLOCK(CY_USBD_STATUS_SUCCESS == retStatus, retStatus);
            if (retStatus == CY_USBD_STATUS_SUCCESS)
            {
                isReqHandled = true;
            }
        }

        /* Sector Erase to SPI flash */
        if (bRequest == FLASH_CMD_FLASH_SECTOR_ERASE)
        {
            sector = wIndex;
            /* The external flash progam mode of USB control center assumes a sector size of CY_APP_SPI_FLASH_ERASE_SIZE
             * while estimating how many sectors to be erased for a given filesize.
             * Users can change this based on their host application.
             */
            spiAddress = sector * CY_APP_SPI_FLASH_ERASE_SIZE;
            if (Cy_SPI_EraseOperation(spiAddress, CY_APP_SPI_FLASH_ERASE_SIZE) == CY_SMIF_SUCCESS)
            {
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                DBG_APP_TRACE("Erase sector %d done\r\n", sector);
            }
            else
            {
                DBG_APP_ERR("Erase sector %d fail\r\n", sector);
            }
            isReqHandled = true;
        }
    }

    /* SET_SEL request is supposed to have an OUT data phase of 6 bytes. */
    if ((bRequest == CY_USB_SC_SET_SEL) && (wLength == 6))
    {
        /* SET_SEL request is only received in USBSS case and the Cy_USB_USBD_RecvEndp0Data is blocking. */
        retStatus = Cy_USB_USBD_RecvEndp0Data(pUsbdCtxt, (uint8_t *)SetSelDataBuffer, wLength);
        DBG_APP_INFO("SET_SEL: EP0 recv stat = %d, Data=%x:%x\r\n",
                retStatus, SetSelDataBuffer[0], SetSelDataBuffer[1]);
        isReqHandled = true;
    }

    /*
     * If Request is not handled by the callback, Stall the command.
     */
    if (!isReqHandled)
    {
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00,
                CY_USB_ENDP_DIR_IN, TRUE);
    }
} /* end of function. */

/*
 * Function: Cy_USB_AppSuspendCallback()
 * Description: This Function will be called by USBD  layer when
 *              Suspend signal/message is detected.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppSuspendCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
    cy_stc_usb_cal_msg_t *pMsg)
{
  cy_stc_usb_app_ctxt_t *pUsbApp;

  pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
  pUsbApp->prevDevState = pUsbApp->devState;
  pUsbApp->devState = CY_USB_DEVICE_STATE_SUSPEND;
} /* end of function. */

/*
 * Function: Cy_USB_AppResumeCallback()
 * Description: This Function will be called by USBD  layer when
 *              Resume signal/message is detected.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppResumeCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
    cy_stc_usb_cal_msg_t *pMsg)
{
  cy_stc_usb_app_ctxt_t *pUsbApp;
  cy_en_usb_device_state_t tempState;

  pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

  tempState = pUsbApp->devState;
  pUsbApp->devState = pUsbApp->prevDevState;
  pUsbApp->prevDevState = tempState;
  return;
} /* end of function. */

/*
 * Function: Cy_USB_AppSetIntfCallback()
 * Description: This Function will be called by USBD  layer when
 *              set interface is called.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppSetIntfCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
    cy_stc_usb_cal_msg_t *pMsg)
{
  cy_stc_usb_setup_req_t *pSetupReq;
  uint8_t intfNum, altSetting;
  int8_t numOfEndp;
  uint8_t *pIntfDscr, *pEndpDscr;
  uint32_t endpNumber;
  cy_en_usb_endp_dir_t endpDirection;
  cy_stc_usb_app_ctxt_t *pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

  DBG_APP_INFO("AppSetIntfCallback Start\r\n");
  pSetupReq = &(pUsbdCtxt->setupReq);
  /*
   * Get interface and alt setting info. If new setting same as previous
   * then return.
   * If new alt setting came then first Unconfigure previous settings
   * and then configure new settings.
   */
  intfNum = pSetupReq->wIndex;
  altSetting = pSetupReq->wValue;

  if (altSetting == pUsbApp->prevAltSetting)
  {
    DBG_APP_INFO("SameAltSetting\r\n");
  }

  /* New altSetting is different than previous one so unconfigure previous. */
  pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, intfNum, pUsbApp->prevAltSetting);
  DBG_APP_INFO("unconfigPrevAltSet\r\n");
  if (pIntfDscr == NULL)
  {
    DBG_APP_INFO("pIntfDscrNull\r\n");
    return;
  }
  numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
  DBG_APP_INFO("NUM_OF_EP %X\r\n", numOfEndp);
  if (numOfEndp == 0x00)
  {
    DBG_APP_INFO("SetIntf:prevNumEp 0\r\n");
  }
  else
  {
    pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
    while (numOfEndp != 0x00)
    {
      if (*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS) & 0x80)
      {
        endpDirection = CY_USB_ENDP_DIR_IN;
      }
      else
      {
        endpDirection = CY_USB_ENDP_DIR_OUT;
      }
      endpNumber = (uint32_t)((*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS)) & 0x7F);

      /* Unconfigure any previous settings. */
      Cy_USBD_EnableEndp(pUsbdCtxt, endpNumber, endpDirection, FALSE);

      numOfEndp--;
      pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
    }
  }

  /* Now take care of different config with new alt setting. */
  pUsbApp->prevAltSetting = altSetting;
  pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, intfNum, altSetting);
  if (pIntfDscr == NULL)
  {
    DBG_APP_INFO("pIntfDscrNull\r\n");
    return;
  }

  numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
  if (numOfEndp == 0x00)
  {
    DBG_APP_INFO("SetIntf:numEp 0\r\n");
  }
  else
  {
    pUsbApp->prevAltSetting = altSetting;
    pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
    while (numOfEndp != 0x00)
    {
      Cy_USB_AppConfigureEndp(pUsbdCtxt, pEndpDscr);
      numOfEndp--;
      pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
    }
  }

  DBG_APP_INFO("AppSetIntfCallback done\r\n");
  return;
} /* end of function. */

/*
 * Function: Cy_USB_AppZlpCallback()
 * Description: This Function will be called by USBD layer when
 *              ZLP message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppZlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
    cy_stc_usb_cal_msg_t *pMsg)
{
  return;
} /* end of function. */

/*
 * Function: Cy_USB_AppL1SleepCallback()
 * Description: This Function will be called by USBD layer when
 *              L1 Sleep message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppL1SleepCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
    cy_stc_usb_cal_msg_t *pMsg)
{
  DBG_APP_INFO("AppL1SleepCb\r\n");
  return;
} /* end of function. */

/*
 * Function: Cy_USB_AppL1ResumeCallback()
 * Description: This Function will be called by USBD layer when
 *              L1 Resume message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppL1ResumeCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
    cy_stc_usb_cal_msg_t *pMsg)
{
  DBG_APP_INFO("AppL1ResumeCb\r\n");
  return;
} /* end of function. */

/*
 * Function: Cy_USB_AppSlpCallback()
 * Description: This Function will be called by USBD layer when
 *              SLP message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppSlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
    cy_stc_usb_cal_msg_t *pMsg)
{
  DBG_APP_INFO("AppSlpCb\r\n");
  return;
} /* end of function. */

/*
 * Function: Cy_USB_AppSetFeatureCallback()
 * Description: This Function will be called by USBD layer when
 *              set feature message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppSetFeatureCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
    cy_stc_usb_cal_msg_t *pMsg)
{
  DBG_APP_INFO("AppSetFeatureCb\r\n");
  return;
} /* end of function. */

/*
 * Function: Cy_USB_AppClearFeatureCallback()
 * Description: This Function will be called by USBD layer when
 *              clear feature message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppClearFeatureCallback(void *pUsbApp,
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
    cy_stc_usb_cal_msg_t *pMsg)
{
  DBG_APP_INFO("AppClearFeatureCb\r\n");
  return;
} /* end of function. */


void Cy_USB_TaskHandler(void *pTaskParam)
{
  cy_stc_usbd_app_msg_t queueMsg;
  BaseType_t xStatus;
  cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
  cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pTaskParam;

  /* If VBus is present, enable the USB data connection. */
  pAppCtxt->vbusPresent = (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);
  if (pAppCtxt->vbusPresent) {
    UsbSSConnectionEnable(pAppCtxt);
  }

  do
  {
    /*
     * Wait until some data is received from the queue.
     * Timeout after 100 ms.
     */
    xStatus = xQueueReceive(pAppCtxt->xQueue, &queueMsg, 100);
    if (xStatus != pdPASS)
    {
      continue;
    }

    switch (queueMsg.type)
    {
      case CY_SMIF_READ_COMPLETE:
        DBG_APP_TRACE(" SMIF READ_COMPLETE\r\n");
        /*Data transfer from DMA to read buffer complete. Send to USB*/
        retStatus = Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt, pAppCtxt->spiHbwReadBuffer, pAppCtxt->spiHbwRxCount);
        ASSERT_NON_BLOCK(CY_USBD_STATUS_SUCCESS == retStatus, retStatus);
        break;

      default:
        DBG_APP_INFO("Default\r\n");
        break;
    } /* end of switch() */

  } while (1);
}

/*[]*/

