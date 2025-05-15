/***************************************************************************//**
 * \file cy_qspi_mem.h
 * \version 1.0
 *
 * File with declarations for FX SMIF interface
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

#ifndef _QSPI_MEM_H_
#define _QSPI_MEM_H_

#include "cy_pdl.h"
#include "cy_debug.h"
#include "cy_device.h"
#include "cycfg_qspi_memslot.h"


#define ASSERT(condition, value)            Cy_App_CheckStatus(__func__, __LINE__, condition, value, true);
#define ASSERT_NON_BLOCK(condition, value)  Cy_App_CheckStatus(__func__, __LINE__, condition, value, false);


#define SMIF_HW                 (SMIF0)
#define CY_SYSCLK_SPI_CLK_HF1   (1)

#define SMIF_CLK_HSIOM                  (P6_0_SMIF_SPI_CLK)
#define SMIF_CLK_PORT                   (P6_0_PORT)
#define SMIF_CLK_PIN                    (P6_0_PIN)

#define SMIF_SELECT0_HSIOM              (P6_1_SMIF_SPI_SELECT0)
#define SMIF_SELECT0_PORT               (P6_1_PORT)
#define SMIF_SELECT0_PIN                (P6_1_PIN)

#define SMIF_SELECT1_HSIOM              (P6_2_SMIF_SPI_SELECT1)
#define SMIF_SELECT1_PORT               (P6_2_PORT)
#define SMIF_SELECT1_PIN                (P6_2_PIN)

#define SMIF_DATA0_HSIOM                (P7_0_SMIF_SPI_DATA0)
#define SMIF_DATA0_PORT                 (P7_0_PORT)
#define SMIF_DATA0_PIN                  (P7_0_PIN)

#define SMIF_DATA1_HSIOM                (P7_1_SMIF_SPI_DATA1)
#define SMIF_DATA1_PORT                 (P7_1_PORT)
#define SMIF_DATA1_PIN                  (P7_1_PIN)

#define SMIF_DATA2_HSIOM                (P7_2_SMIF_SPI_DATA2)
#define SMIF_DATA2_PORT                 (P7_2_PORT)
#define SMIF_DATA2_PIN                  (P7_2_PIN)

#define SMIF_DATA3_HSIOM                (P7_3_SMIF_SPI_DATA3)
#define SMIF_DATA3_PORT                 (P7_3_PORT)
#define SMIF_DATA3_PIN                  (P7_3_PIN)

#define SMIF_DATA4_HSIOM                (P7_4_SMIF_SPI_DATA4)
#define SMIF_DATA4_PORT                 (P7_4_PORT)
#define SMIF_DATA4_PIN                  (P7_4_PIN)

#define SMIF_DATA5_HSIOM                (P7_5_SMIF_SPI_DATA5)
#define SMIF_DATA5_PORT                 (P7_5_PORT)
#define SMIF_DATA5_PIN                  (P7_5_PIN)

#define SMIF_DATA6_HSIOM                (P7_6_SMIF_SPI_DATA6)
#define SMIF_DATA6_PORT                 (P7_6_PORT)
#define SMIF_DATA6_PIN                  (P7_6_PIN)

#define SMIF_DATA7_HSIOM                (P7_7_SMIF_SPI_DATA7)
#define SMIF_DATA7_PORT                 (P7_7_PORT)
#define SMIF_DATA7_PIN                  (P7_7_PIN)


#define SMIF_DW                             (DW0)
#define SMIF_TX_DW_CHANNEL                  (20)
#define SMIF_RX_DW_CHANNEL                  (21)

#define SMIF_DMAC                           (DMAC)
#define SMIF_TX_DMAC_CHANNEL                (2)
#define SMIF_RX_DMAC_CHANNEL                (3)

#define SMIF_TX_THRESHOLD_LEVEL             (4)  /*Trigger when TX FIFO has space for >= THRESHOLD elements*/
#define SMIF_RX_THRESHOLD_LEVEL             (4)  /* Trigger when RX FIFO has > THRESHOLD elements*/
#define SMIF_DMA_MAX_BUFFER_SIZE            (2048)


#define CY_SPI_MAX_NUM_ADDR_BYTES           (0x04)

#define CY_APP_SPI_WRITE_ANY_REGISTER_CMD   (0x71)
#define CY_APP_SPI_READ_ANY_REGISTER_CMD    (0x65)
#define CY_APP_CR1V_REG_ADDR                (0x800002)
#define CY_APP_QE_BITMASK                   (0x02)

/* Vendor commands sent by USB Host application (eg: control center)*/
typedef enum cy_en_flash_programmer_vendor_cmd_t
{
  FLASH_CMD_CHECK_FLASHPROG       = 0xB0,
  FLASH_CMD_FLASH_WRITE           = 0xB2,
  FLASH_CMD_FLASH_READ            = 0xB3,
  FLASH_CMD_FLASH_SECTOR_ERASE    = 0xB4,
  FLASH_CMD_CHECK_MEMBUSY         = 0xB5,
}cy_en_flash_programmer_vendor_cmd_t;


#define CY_APP_SPI_FLASH_ERASE_SIZE       (0x0010000)
#define CY_APP_SPI_PROGRAM_TIMEOUT_US     (2000)
#define CY_APP_SPI_PAGE_SIZE_DEFAULT      (0x100)
#define CY_APP_SPI_RESET_ENABLE_CMD       (0x66)
#define CY_APP_SPI_SW_RESET_CMD           (0x99)

extern cy_stc_smif_context_t qspiContext;

cy_en_smif_status_t Cy_SMIF_Start(void);
cy_en_smif_status_t Cy_SPI_WriteOperation(uint32_t address, uint8_t *txBuffer, uint32_t length);
cy_en_smif_status_t Cy_SPI_ReadOperation(uint32_t address, uint8_t *rxBuffer, uint32_t length);
cy_en_smif_status_t Cy_SPI_EraseOperation(uint32_t address, uint32_t eraseSize);
cy_en_smif_status_t Cy_SPI_IsDeviceBusy(void);
void Cy_App_CheckStatus(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking);

#endif /*_QSPI_MEM_H_*/
