/***************************************************************************//**
 * \file cy_usb_app.h
 * \version 1.0
 *
 * Defines the interfaces used by the FX10 SMIF DMAC application.
 *
 *******************************************************************************
 * \copyright
 * (c) (2021-2023), Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef _CY_USB_APP_H_
#define _CY_USB_APP_H_

#include "cy_pdl.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "event_groups.h"
#include "cy_debug.h"
#include "cy_hbdma_mgr.h"
#include "cy_usb_common.h"
#include "cy_usb_usbd.h"
#include "cy_usbhs_dw_wrapper.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define USB3_DESC_ATTRIBUTES __attribute__ ((section(".descSection"), used)) __attribute__ ((aligned (32)))

  /* P4.0 is used for VBus detect functionality. */
#define VBUS_DETECT_GPIO_PORT           (P4_0_PORT)
#define VBUS_DETECT_GPIO_PIN            (P4_0_PIN)
#define VBUS_DETECT_GPIO_INTR           (ioss_interrupts_gpio_dpslp_4_IRQn)
#define VBUS_DETECT_STATE               (0u)


#define PERIPHERAL_USB_BUFFER_SIZE  (1024)
#define PERIPHERAL_USB_BUFFER_COUNT (2)

#define PDM_APP_BUFFER_CNT    (0x04u)           /* Number of DMA buffers used. */
#define CY_SMIF_READ_COMPLETE (0x04u)
#define USBHS_MAXPKT_SIZE     (0x40u)

  extern uint8_t glOsString[];
  extern uint8_t glOsCompatibilityId[];
  extern uint8_t glOsFeature[];

  #define MS_VENDOR_CODE         (0xF0)
  typedef struct cy_stc_usb_app_ctxt_ cy_stc_usb_app_ctxt_t;
  void Cy_USB_TaskHandler (void *pTaskParam);
  void Cy_USB_PDMDmaReadCompletion(void *pApp);
  void Cy_USB_App_PDM_Read (uint8_t *pHbSramBuffer, uint16_t dataSize);
  void Cy_USB_AppClearPDMDmaInterrupt(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t channel);
  void PDM0_RX_ISR(void);
  void PDM1_RX_ISR(void);

  /* USBD layer return code shared between USBD layer and Application layer. */
  typedef enum cy_en_usb_app_ret_code_ {
    CY_USB_APP_STATUS_SUCCESS=0,
    CY_USB_APP_STATUS_FAILURE,
  } cy_en_usb_app_ret_code_t;

  /*
   * USB application data structure which is bridge between USB system and device
   * functionality.
   * It maintains some usb system information which comes from USBD and it also
   * maintains info about functionality.
   */
  struct cy_stc_usb_app_ctxt_
  {
    uint8_t                  firstInitDone;             /** Whether APP-INIT has been completed. */
    bool                     usbConnected;              /** Whether USB connection has been enabled. */
    bool                     vbusPresent;               /** Whether VBus supply is present. */

    cy_en_usb_device_state_t devState;                  /** Current state of USB connection. */
    cy_en_usb_device_state_t prevDevState;              /** Previous state of USB connection. */
    cy_en_usb_speed_t        devSpeed;                  /** Active USB connection speed. */
    uint8_t                  devAddr;                   /** USB address assigned to the device. */
    uint8_t                  activeCfgNum;              /** Active configuration index. */
    cy_en_usb_enum_method_t  enumMethod;                /** Enumeration method used. */
    uint8_t                  prevAltSetting;            /** Previous alternate setting index. */

    cy_stc_app_endp_dma_set_t endpInDma[CY_USB_MAX_ENDP_NUMBER];        /** DMA state container for IN endpoints. */
    cy_stc_app_endp_dma_set_t endpOutDma[CY_USB_MAX_ENDP_NUMBER];       /** DMA state container for OUT endpoints. */

    DMAC_Type                  *pCpuDmacBase;           /** DMAC IP control block. */
    DW_Type                    *pCpuDw0Base;            /** DW0 IP control block. */
    DW_Type                    *pCpuDw1Base;            /** DW1 IP control block. */
    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt;          /** High BandWidth DMA manager context structure. */
    cy_stc_usb_usbd_ctxt_t     *pUsbdCtxt;              /** USB stack context. */

    TaskHandle_t  appTaskHandle;                        /** Handle to the application task. */
    QueueHandle_t xQueue;                               /** Handle to application message queue. */

    uint8_t *spiHbwWriteBuffer;                         /** HBDMA buffer for storing data to be written to SPI (SMIF) */
    uint8_t *spiHbwReadBuffer;                          /** HBDMA buffer for storing data to be read from SPI (SMIF) */
    uint32_t spiHbwRxCount;                             /** Count of data to be read from SPI (SMIF) */
  };



  void Cy_USB_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt,
      cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
      DMAC_Type *pCpuDmacBase,
      DW_Type *pCpuDw0Base,
      DW_Type *pCpuDw1Base,
      cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt);

  void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt);

  void Cy_USB_AppSetCfgCallback(void *pAppCtxt,
      cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
      cy_stc_usb_cal_msg_t *pMsg);
  void Cy_USB_AppBusResetCallback(void *pAppCtxt,
      cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
      cy_stc_usb_cal_msg_t *pMsg);
  void Cy_USB_AppBusResetDoneCallback(void *pAppCtxt,
      cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
      cy_stc_usb_cal_msg_t *pMsg);
  void Cy_USB_AppBusSpeedCallback(void *pAppCtxt,
      cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
      cy_stc_usb_cal_msg_t *pMsg);
  void Cy_USB_AppSetupCallback(void *pAppCtxt,
      cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
      cy_stc_usb_cal_msg_t *pMsg);
  void Cy_USB_AppSuspendCallback(void *pAppCtxt,
      cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
      cy_stc_usb_cal_msg_t *pMsg);
  void Cy_USB_AppResumeCallback (void *pAppCtxt,
      cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
      cy_stc_usb_cal_msg_t *pMsg);
  void Cy_USB_AppSetIntfCallback(void *pAppCtxt,
      cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
      cy_stc_usb_cal_msg_t *pMsg);
  void Cy_USB_AppL1SleepCallback(void *pUsbApp,
      cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
      cy_stc_usb_cal_msg_t *pMsg);
  void Cy_USB_AppL1ResumeCallback(void *pUsbApp,
      cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
      cy_stc_usb_cal_msg_t *pMsg);
  void Cy_USB_AppZlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
      cy_stc_usb_cal_msg_t *pMsg);
  void Cy_USB_AppSlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
      cy_stc_usb_cal_msg_t *pMsg);
  void Cy_USB_AppSetFeatureCallback(void *pUsbApp,
      cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
      cy_stc_usb_cal_msg_t *pMsg);
  void Cy_USB_AppClearFeatureCallback(void *pUsbApp,
      cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
      cy_stc_usb_cal_msg_t *pMsg);
#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_APP_H_ */

/* End of File */

