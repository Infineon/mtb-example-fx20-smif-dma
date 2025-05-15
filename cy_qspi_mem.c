/***************************************************************************//**
 * \file cy_qspi_mem.c
 * \version 1.0
 *
 * Defines the QSPI APIs used in FX10 SMIF application
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


#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_qspi_mem.h"

cy_stc_smif_context_t qspiContext;

static const cy_stc_smif_config_t qspiConfig =
{
    .mode = (uint32_t)CY_SMIF_NORMAL,
    .deselectDelay = 0u, /*Minimum duration SPI_SS is held high between SPI transfers (Default: 0 -> 1 interface cycle)*/
    .rxClockSel = (uint32_t)CY_SMIF_SEL_INV_INTERNAL_CLK,/* Source selection for receiver clock. MISO is sampled on rising edge of this clock */
    .blockEvent = (uint32_t)CY_SMIF_BUS_ERROR
};

#if USE_SMIF_DMAC
static cy_stc_dmac_descriptor_t glSMIFWriteDmaDesc;
static cy_stc_dmac_descriptor_t glSMIFReadDmaDesc;
#else
static cy_stc_dma_descriptor_t glSMIFWriteDmaDesc;
static cy_stc_dma_descriptor_t glSMIFReadDmaDesc;
static cy_stc_dma_descriptor_t glSMIFReadDmaDescNext;
#endif


/*
Function     : AddessToArray ()
Description  : Change address value to byte-array
Parameters  :  uint32_t value, uint8_t *byteArray, uint8_t size
Return      :  cy_en_smif_status_t
*/
static cy_en_smif_status_t AddressToArray(uint32_t value, uint8_t *byteArray, uint8_t size)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    if(byteArray == NULL)
    {
        return CY_SMIF_BAD_PARAM;
    }

    do
    {
        size--;
        byteArray[size] = (uint8_t)(value & 0x000000FF);
        value >>= 8U; /* Shift to get the next byte */
    } while (size > 0U);

    return status;
}

/*
 * Function     :  Cy_SPI_FlashReset ()
 * Description  :  Issue soft reset to SPI Flash
 * Parameters   :  SMIF_Type *base, cy_stc_smif_mem_config_t const *memDevice
 * Return       :  cy_en_smif_status_t
 * */
static cy_en_smif_status_t Cy_SPI_FlashReset(SMIF_Type *base, cy_stc_smif_mem_config_t const *memDevice)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    status = Cy_SMIF_TransmitCommand(base,
            CY_APP_SPI_RESET_ENABLE_CMD,
            CY_SMIF_WIDTH_QUAD,
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            CY_SMIF_WIDTH_NA,
            memDevice->slaveSelect,
            CY_SMIF_TX_LAST_BYTE,
            &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    if(CY_SMIF_SUCCESS == status)
    {
        status = Cy_SMIF_TransmitCommand(base,
                CY_APP_SPI_SW_RESET_CMD,
                CY_SMIF_WIDTH_QUAD,
                NULL,
                CY_SMIF_CMD_WITHOUT_PARAM,
                CY_SMIF_WIDTH_NA,
                memDevice->slaveSelect,
                CY_SMIF_TX_LAST_BYTE,
                &qspiContext);
        ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

        /* Wait for tRPH (35us) for reset to complete. Sufficient margin added */
        Cy_SysLib_DelayUs(1000);
    }
    return status;
}



/*
 * Function     :  Cy_SPI_WriteAnyRegister ()
 * Description  :  Issue Write Any Register (WRAR) command to SPI Flash
 * Parameters   :  SMIF_Type *base, cy_stc_smif_mem_config_t const *memDevice, uint32_t regAddress, uint8_t value
 * Return       :  cy_en_smif_status_t
 * */

static cy_en_smif_status_t  Cy_SPI_WriteAnyRegister(SMIF_Type *base, cy_stc_smif_mem_config_t const *memDevice,
                                                    uint32_t regAddress, uint8_t value)
{
    cy_en_smif_status_t status = CY_SMIF_BAD_PARAM;
    uint8_t addrArray[CY_SPI_MAX_NUM_ADDR_BYTES] = {0};
    if(NULL != memDevice)
    {
        uint8_t numAddrBytes = memDevice->deviceCfg->numOfAddrBytes;

        DBG_APP_INFO("Write 0x%x to Register: 0x%x\r\n", value, regAddress);

        AddressToArray(regAddress, addrArray,numAddrBytes);

        status = Cy_SMIF_MemCmdWriteEnable(base, memDevice, &qspiContext);
        ASSERT_NON_BLOCK(CY_SMIF_SUCCESS == status, status);

        if(CY_SMIF_SUCCESS == status)
        {
            status = Cy_SMIF_TransmitCommand(base,
                    CY_APP_SPI_WRITE_ANY_REGISTER_CMD,
                    CY_SMIF_WIDTH_SINGLE,
                    addrArray,
                    numAddrBytes,
                    CY_SMIF_WIDTH_SINGLE,
                    memDevice->slaveSelect,
                    CY_SMIF_TX_NOT_LAST_BYTE,
                    &qspiContext);
            ASSERT_NON_BLOCK(CY_SMIF_SUCCESS == status, status);

            if(CY_SMIF_SUCCESS == status)
            {
                status = Cy_SMIF_TransmitDataBlocking(base, &value, CY_SMIF_WRITE_ONE_BYTE, CY_SMIF_WIDTH_SINGLE, &qspiContext);
                ASSERT_NON_BLOCK(CY_SMIF_SUCCESS == status, status);
            }
        }
    }
    return status;
}

/*
 * Function     :  Cy_SPI_ReadAnyRegister
 * Description  :  Issue Read Any register (RDAR) command to SPI Flash
 * Parameters   :  SMIF_Type *base, cy_stc_smif_mem_config_t const *memDevice, uint8_t *readValue, uint32_t regAddress
 * Return       :  uint8_t
 * */

static cy_en_smif_status_t Cy_SPI_ReadAnyRegister(SMIF_Type *base, cy_stc_smif_mem_config_t const *memDevice,
                                                  uint8_t *readValue, uint32_t regAddress)
{
    cy_en_smif_status_t status = CY_SMIF_BAD_PARAM;
    uint8_t addrArray[CY_SPI_MAX_NUM_ADDR_BYTES] = {0};
    uint8_t numAddrBytes = memDevice->deviceCfg->numOfAddrBytes;
    cy_stc_smif_mem_device_cfg_t *device = memDevice->deviceCfg;

    if((NULL != device) && (NULL != readValue))
    {
        cy_stc_smif_mem_cmd_t *cmdRead = device->readCmd;

        if(NULL != cmdRead)
        {
            AddressToArray(regAddress, addrArray,numAddrBytes);

            status = Cy_SMIF_MemCmdWriteEnable(base, memDevice, &qspiContext);
            ASSERT_NON_BLOCK(CY_SMIF_SUCCESS == status, status);

            if(CY_SMIF_SUCCESS == status)
            {
                status = Cy_SMIF_TransmitCommand(base,
                        CY_APP_SPI_READ_ANY_REGISTER_CMD,
                        CY_SMIF_WIDTH_SINGLE,
                        addrArray,
                        numAddrBytes,
                        CY_SMIF_WIDTH_SINGLE,
                        memDevice->slaveSelect,
                        CY_SMIF_TX_NOT_LAST_BYTE,
                        &qspiContext);
                ASSERT_NON_BLOCK(CY_SMIF_SUCCESS == status, status);

                if (CY_SMIF_SUCCESS == status)
                {
                    if(cmdRead->dummyCycles > 0)
                    {
                        Cy_SMIF_SendDummyCycles(SMIF_HW,cmdRead->dummyCycles);
                    }
                    status = Cy_SMIF_ReceiveDataBlocking(SMIF0, readValue, CY_SMIF_READ_ONE_BYTE, CY_SMIF_WIDTH_SINGLE, &qspiContext);
                    ASSERT_NON_BLOCK(CY_SMIF_SUCCESS == status, status);
                    DBG_APP_INFO("Read Register: 0x%x Read value = 0x%x\r\n",regAddress, *readValue);
                }
            }
        }
    }
    return status;
}

/*
 * Function     :  Cy_SPI_QuadEnableVolatile
 * Description  :  Enable Quad mode by writing to the volatile Quad Enable bit
 * Parameters   :  SMIF_Type *base, cy_stc_smif_mem_config_t const *memDevice, cy_stc_smif_context_t const *context
 * Return       :  cy_en_smif_status_t
 * */
cy_en_smif_status_t Cy_SPI_QuadEnableVolatile(SMIF_Type *base, cy_stc_smif_mem_config_t const *memDevice,
                                              cy_stc_smif_context_t const *context)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint8_t regValue = 0;

    /*Reset to clear the volatile SPI flash settings*/
    Cy_SPI_FlashReset(base, memDevice);
    status = Cy_SPI_ReadAnyRegister(base, memDevice, &regValue, CY_APP_CR1V_REG_ADDR);
    if(CY_SMIF_SUCCESS == status)
    {
        status = Cy_SPI_WriteAnyRegister(base, memDevice, CY_APP_CR1V_REG_ADDR, regValue | CY_APP_QE_BITMASK);
        ASSERT_NON_BLOCK(CY_SMIF_SUCCESS == status, status);
        if(CY_SMIF_SUCCESS == status)
        {
            DBG_APP_INFO("Quad mode enabled (volatile)\r\n");
        }
    }
    return status;
}




/*
Function     : Cy_SMIF_Start ()
Description :  Initialize SMIF clock and pins
Parameters  :  void
Return      :  cy_en_smif_status_t

*/

cy_en_smif_status_t Cy_SMIF_Start(void)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    status = Cy_SMIF_Init(SMIF_HW, &qspiConfig, 10000u, &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    Cy_SMIF_SetDataSelect(SMIF_HW, CY_SMIF_SLAVE_SELECT_0, CY_SMIF_DATA_SEL0);
    Cy_SMIF_SetDataSelect(SMIF_HW, CY_SMIF_SLAVE_SELECT_1, CY_SMIF_DATA_SEL2);

    Cy_SMIF_Enable(SMIF_HW, &qspiContext);

    status = Cy_SMIF_MemInit(SMIF_HW, &smifBlockConfig, &qspiContext);
    if(status == CY_SMIF_SUCCESS)
    {
        DBG_APP_INFO("SMIF Initialization Done\r\n");
    }
    else
    {
        DBG_APP_ERR("SMIF Initialization failed with error: 0x%x\r\n", status);
    }
    DBG_APP_TRACE("SMIF Clock = %d\r\n",Cy_SysClk_ClkHfGetFrequency(CY_SYSCLK_SPI_CLK_HF1));

    /* At this stage, flash is in 1S-1S-1S mode, enable quad mode
     * for exercising octal read mode in Dual-Quad mode */

    for (uint8_t deviceIdx = 0; deviceIdx < smifBlockConfig.memCount;  deviceIdx++)
    {
        status = Cy_SPI_QuadEnableVolatile(SMIF_HW, smifMemConfigs[deviceIdx], &qspiContext);
        ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    }
    return status;
}


cy_en_smif_status_t Cy_SPI_EraseOperation(uint32_t address, uint32_t eraseSize)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    for (uint8_t index = 0; index < smifBlockConfig.memCount; index++) {
        status |= Cy_SMIF_MemEraseSector(SMIF_HW, smifMemConfigs[index],address, eraseSize, &qspiContext);
        ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    }
    return status;
}



#if (!USE_SMIF_DMAC)
/*
Function     : Cy_DW_WriteToPeripheral ()
Description :  Initialize DW channel descriptors and enable the transfer of data from HBDMA buffer to SMIF TX FIFO
Parameters  :  uint8_t * pHbSramBuffer, uint16_t count
Return      :  void

*/

void Cy_DW_WriteToPeripheral(uint8_t * pHbSramBuffer, uint16_t count)
{
    cy_stc_dma_descriptor_config_t desc_cfg;
    cy_stc_dma_channel_config_t chan_cfg;
    cy_en_dma_status_t stat;

    /* If the DMA channel is already enabled, disable it. */
    Cy_DMA_Channel_Disable(SMIF_DW, SMIF_TX_DW_CHANNEL);

    /*
       SMIF_TX_THRESHOLD_LEVEL is set as 4 bytes (ie, trigger will be generated when there is space for 4 bytes in FIFO)
       For SMIF TX, data should be written from HBDMA Buffer to SMIF_TX_FIFO.

       Transfer is done as 2D DMA transfer:
       X Loop: Do SMIF_TX_THRESHOLD_LEVEL transfers from HBSRAM to SMIF_TX_FIFO on every input trigger
       Y Loop: Repeat till the required bytes are transferred (count parameter)

       8 bit from HBDMA transferred as 1 word to WR1
       HBDMA Buffer data size is 1 Byte.
       SMIF_TX_FIFO_WR1 is 4 Bytes (valid data is lower 1 byte).
       number of transfers after one input trigger (X_loop) = SMIF_TX_THRESHOLD_LEVEL
       srcYincrement (where to start the transfer after on X loop) : SMIF_TX_THRESHOLD_LEVEL
       */

    desc_cfg.retrigger       = CY_DMA_RETRIG_IM;
    desc_cfg.interruptType   = CY_DMA_DESCR_CHAIN;
    desc_cfg.triggerOutType  = CY_DMA_DESCR_CHAIN;
    desc_cfg.triggerInType   = CY_DMA_X_LOOP;
    desc_cfg.channelState    = CY_DMA_CHANNEL_DISABLED;
    desc_cfg.dataSize        = CY_DMA_BYTE;
    desc_cfg.srcTransferSize = CY_DMA_TRANSFER_SIZE_DATA;
    desc_cfg.dstTransferSize = CY_DMA_TRANSFER_SIZE_WORD;
    desc_cfg.descriptorType  = CY_DMA_2D_TRANSFER;
    desc_cfg.dstAddress      = (void *)(&(SMIF0->TX_DATA_FIFO_WR1));
    desc_cfg.srcAddress      = (void *)pHbSramBuffer;
    desc_cfg.srcXincrement   = 1u;
    desc_cfg.dstXincrement   = 0u;
    desc_cfg.xCount          = SMIF_TX_THRESHOLD_LEVEL;
    desc_cfg.srcYincrement   = SMIF_TX_THRESHOLD_LEVEL;
    desc_cfg.dstYincrement   = 0;
    desc_cfg.yCount          = (count / SMIF_TX_THRESHOLD_LEVEL);
    desc_cfg.nextDescriptor  = NULL;

    stat = Cy_DMA_Descriptor_Init(&glSMIFWriteDmaDesc, &desc_cfg);
    ASSERT_NON_BLOCK(stat == CY_DMA_SUCCESS, stat);

    chan_cfg.descriptor  = &glSMIFWriteDmaDesc;
    chan_cfg.preemptable = false;
    chan_cfg.priority    = 0;
    chan_cfg.enable      = false;
    chan_cfg.bufferable  = false;

    stat = Cy_DMA_Channel_Init(SMIF_DW, SMIF_TX_DW_CHANNEL, &chan_cfg);
    ASSERT_NON_BLOCK(stat == CY_DMA_SUCCESS, stat);

    /* Make sure SLOW AHB read cache is evicted before new transfer is started. */
    MAIN_REG->CTRL |= MAIN_REG_CTRL_EVICT_SLOW_AHB_RD_CACHE_Msk;
    while ((MAIN_REG->CTRL & MAIN_REG_CTRL_EVICT_SLOW_AHB_RD_CACHE_Msk) != 0);

    Cy_DMA_Channel_SetInterruptMask(SMIF_DW, SMIF_TX_DW_CHANNEL, CY_DMA_INTR_MASK);
    Cy_DMA_Channel_Enable(SMIF_DW, SMIF_TX_DW_CHANNEL);
}

#else
/*
Function     : Cy_DMAC_WriteToPeripheral ()
Description :  Initialize DMAC channel descriptors and enable the transfer of data from HBDMA buffer to SMIF TX FIFO
Parameters  :  uint8_t * pHbSramBuffer, uint16_t count
Return      :  void

*/

void Cy_DMAC_WriteToPeripheral(uint8_t * pHbSramBuffer, uint16_t count)
{
    cy_stc_dmac_descriptor_config_t desc_cfg;
    cy_stc_dmac_channel_config_t chan_cfg;
    cy_en_dmac_status_t stat;

    /* If the DMA channel is already enabled, disable it. */
    Cy_DMAC_Channel_Disable(SMIF_DMAC, SMIF_TX_DMAC_CHANNEL);

    /*
       SMIF_TX_THRESHOLD_LEVEL is set as 4 bytes (ie, trigger will be generated when there is space for 4 bytes in FIFO)
       For SMIF TX, data should be written from HBDMA Buffer to SMIF_TX_FIFO.

       Transfer is done as 2D DMA transfer:
       X Loop: Do SMIF_TX_THRESHOLD_LEVEL transfers from HBSRAM to SMIF_TX_FIFO on every input trigger
       Y Loop: Repeat till the required bytes are transferred (count parameter)

       4 bytes from HBDMA transferred to WR4 FIFO register (can hold 4 bytes)
       number of transfers after one input trigger (X_loop) = SMIF_TX_THRESHOLD_LEVEL/4
       srcYincrement (where to start the transfer after on X loop) : SMIF_TX_THRESHOLD_LEVEL/4
       */

    desc_cfg.retrigger       = CY_DMAC_RETRIG_IM;
    desc_cfg.interruptType   = CY_DMAC_DESCR_CHAIN;
    desc_cfg.triggerOutType  = CY_DMAC_DESCR_CHAIN;
    desc_cfg.triggerInType   = CY_DMAC_X_LOOP;
    desc_cfg.channelState    = CY_DMAC_CHANNEL_DISABLED;
    desc_cfg.dataSize        = CY_DMAC_WORD;
    desc_cfg.srcTransferSize = CY_DMAC_TRANSFER_SIZE_WORD;
    desc_cfg.dstTransferSize = CY_DMAC_TRANSFER_SIZE_WORD;
    desc_cfg.descriptorType  = CY_DMAC_2D_TRANSFER;
    desc_cfg.dstAddress      = (void *)(&(SMIF0->TX_DATA_FIFO_WR4));
    desc_cfg.srcAddress      = (void *)pHbSramBuffer;
    desc_cfg.srcXincrement   = 1u;
    desc_cfg.dstXincrement   = 0u;
    desc_cfg.xCount          = SMIF_TX_THRESHOLD_LEVEL/4;
    desc_cfg.srcYincrement   = SMIF_TX_THRESHOLD_LEVEL/4;
    desc_cfg.dstYincrement   = 0;
    desc_cfg.yCount          = (count / SMIF_TX_THRESHOLD_LEVEL);
    desc_cfg.nextDescriptor  = NULL;

    stat = Cy_DMAC_Descriptor_Init(&glSMIFWriteDmaDesc, &desc_cfg);
    ASSERT_NON_BLOCK(stat == CY_DMAC_SUCCESS, stat);

    chan_cfg.descriptor  = &glSMIFWriteDmaDesc;
    chan_cfg.priority    = 0;
    chan_cfg.enable      = false;
    chan_cfg.bufferable  = false;

    stat = Cy_DMAC_Channel_Init(SMIF_DMAC, SMIF_TX_DMAC_CHANNEL, &chan_cfg);
    ASSERT_NON_BLOCK(stat == CY_DMAC_SUCCESS, stat);

    /* Make sure SLOW AHB read cache is evicted before new transfer is started. */
    MAIN_REG->CTRL |= MAIN_REG_CTRL_EVICT_SLOW_AHB_RD_CACHE_Msk;
    while ((MAIN_REG->CTRL & MAIN_REG_CTRL_EVICT_SLOW_AHB_RD_CACHE_Msk) != 0);

    Cy_DMAC_Channel_SetInterruptMask(SMIF_DMAC, SMIF_TX_DMAC_CHANNEL, CY_DMAC_INTR_MASK);
    Cy_DMAC_Channel_Enable(SMIF_DMAC, SMIF_TX_DMAC_CHANNEL);
}
#endif


cy_en_smif_status_t Cy_SPI_MemProgram(SMIF_Type *base,
                                    cy_stc_smif_mem_config_t const *memDevice,
                                    uint8_t const *addr,
                                    uint8_t const *writeBuff,
                                    uint32_t size,
                                    cy_stc_smif_context_t *context)
{
    cy_en_smif_status_t result = CY_SMIF_SUCCESS;
    cy_en_smif_slave_select_t slaveSelected;

    cy_stc_smif_mem_device_cfg_t *device = memDevice->deviceCfg;
    cy_stc_smif_mem_cmd_t *cmdProg = device->programCmd;

    if(NULL == cmdProg)
    {
        result = CY_SMIF_CMD_NOT_FOUND;
    }
    else if ((NULL == addr) || (size > device->programSize))
    {
        result = CY_SMIF_BAD_PARAM;
    }
    else
    {
        slaveSelected = (0U == memDevice->dualQuadSlots)?  memDevice->slaveSelect :
            (cy_en_smif_slave_select_t)memDevice->dualQuadSlots;

        if(CY_SMIF_SUCCESS == result)
        {
            /* The page program command */
            result = Cy_SMIF_TransmitCommand( base, (uint8_t)cmdProg->command,
                    cmdProg->cmdWidth, addr, device->numOfAddrBytes,
                    cmdProg->addrWidth, slaveSelected, CY_SMIF_TX_NOT_LAST_BYTE,
                    context);

            if((CY_SMIF_SUCCESS == result) && (CY_SMIF_NO_COMMAND_OR_MODE != cmdProg->mode))
            {
                result = Cy_SMIF_TransmitCommand(base, (uint8_t)cmdProg->mode,
                        cmdProg->modeWidth, NULL,
                        CY_SMIF_CMD_WITHOUT_PARAM, CY_SMIF_WIDTH_NA,
                        (cy_en_smif_slave_select_t)slaveSelected,
                        CY_SMIF_TX_NOT_LAST_BYTE, context);
            }

            if((cmdProg->dummyCycles > 0U) && (CY_SMIF_SUCCESS == result))
            {
                result = Cy_SMIF_SendDummyCycles(base, cmdProg->dummyCycles);
            }

            if(CY_SMIF_SUCCESS == result)
            {
                result = Cy_SMIF_TransmitData( base, NULL, size,
                        cmdProg->dataWidth, NULL, context);
            }
        }
    }

    return(result);
}


static inline void Cy_SPI_EnableDMAWrite(uint8_t *txBuffer, uint32_t size)
{
#if USE_SMIF_DMAC
    Cy_DMAC_WriteToPeripheral(txBuffer, size);
#else
    Cy_DW_WriteToPeripheral(txBuffer, size);
#endif /*USE_SMIF_DMAC*/
}



/*
Function     : Cy_SPI_WritePage ()
Description :  Write to a SPI flash page (256 Bytes)
Parameters  :  uint32_t address, uint8_t *txBuffer, cy_en_flash_index_t flashIndex
Return      :  cy_en_smif_status_t
*/

cy_en_smif_status_t Cy_SPI_WritePage(uint32_t address, uint8_t *txBuffer, uint32_t size)
{
    uint8_t addrArray[CY_SPI_MAX_NUM_ADDR_BYTES] = {0};
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    const cy_stc_smif_mem_config_t *memDevice = smifMemConfigs[0];
    const cy_stc_smif_mem_config_t *memDeviceAlt = NULL;
    uint32_t programTimeout = memDevice->deviceCfg->programTime;
    uint8_t numAddrBytes = memDevice->deviceCfg->numOfAddrBytes;

    if(smifBlockConfig.memCount > 1)
    {
        memDeviceAlt = smifMemConfigs[1];
    }

    AddressToArray(address, addrArray,numAddrBytes);
    if(memDevice->dualQuadSlots)
    {
        status =  (Cy_SMIF_MemCmdWriteEnable(SMIF_HW, memDevice, &qspiContext) |
                Cy_SMIF_MemCmdWriteEnable(SMIF_HW, memDeviceAlt, &qspiContext));
    }
    else
    {
        status = Cy_SMIF_MemCmdWriteEnable(SMIF_HW, memDevice, &qspiContext);
    }
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    if(status == CY_SMIF_SUCCESS)
    {
        DBG_APP_TRACE("SMIF write: DMA Mode\r\n");
        status = Cy_SPI_MemProgram(SMIF_HW,
                memDevice,
                addrArray,
                txBuffer,
                size,
                &qspiContext);

        Cy_SPI_EnableDMAWrite(txBuffer, size);
    }
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    status = CY_SMIF_SUCCESS;
    while(Cy_SPI_IsDeviceBusy())
    {
        Cy_SysLib_DelayUs(1);
        programTimeout--;
        if(programTimeout == 0)
        {
            status = CY_SMIF_EXCEED_TIMEOUT;
            break;
        }
    }
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    return status;
}

cy_en_smif_status_t Cy_SPI_IsDeviceBusy(void)
{
    cy_en_smif_status_t status = CY_SMIF_EXCEED_TIMEOUT;
    bool isBusy = true;

    if(smifMemConfigs[0]->dualQuadSlots)
    {
        isBusy = (Cy_SMIF_MemIsBusy(SMIF_HW, smifMemConfigs[0], &qspiContext) |
                  Cy_SMIF_MemIsBusy(SMIF_HW, smifMemConfigs[1], &qspiContext));
    }
    else
    {
        isBusy =  Cy_SMIF_MemIsBusy(SMIF_HW, smifMemConfigs[0], &qspiContext) ;
    }

    status = ((isBusy) ? CY_SMIF_EXCEED_TIMEOUT : CY_SMIF_SUCCESS);
    return status;
}


cy_en_smif_status_t Cy_SPI_WriteOperation(uint32_t startingAddress, uint8_t *txBuffer, uint32_t length)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint32_t pageIndex = 0;
    uint32_t qspiAddress = startingAddress;
    uint32_t pageOffset = 0;
    uint32_t pageSize = CY_APP_SPI_PAGE_SIZE_DEFAULT;
    uint32_t numPages = (length / pageSize) + ((length % pageSize)? 1 : 0) ;

    DBG_APP_TRACE("Num Pages to be written to:%d\r\n",numPages);

    for(pageIndex = 0; pageIndex < numPages; pageIndex++)
    {
        pageOffset = pageSize * pageIndex;
        status = Cy_SPI_WritePage(qspiAddress, txBuffer + pageOffset, pageSize);
        qspiAddress += pageSize;
        ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
        if(!status)
        {
            DBG_APP_TRACE("Write %d done\r\n", qspiAddress);
        }
    }
    return status;
}

#if (!USE_SMIF_DMAC)
/*
Function     : Cy_SPI_InitDWRX ()
Description :  Initialize the DW channel for data transfer from SMIF RX FIFO to HBDMA buffer
Parameters  :  uint8_t * pHbSramBuffer, uint16_t count
Return      :  void

*/

void Cy_SPI_InitDWRX(uint8_t * pHbSramBuffer, uint16_t count)
{
    cy_stc_dma_descriptor_config_t desc_cfg;
    cy_stc_dma_channel_config_t chan_cfg;
    cy_en_dma_status_t stat;
    bool enableNextDscr = false;
    uint32_t remainingCount = count;

    uint16_t yCount  = count/SMIF_RX_THRESHOLD_LEVEL;

    if(yCount > CY_DMA_LOOP_COUNT_MAX)
    {
        yCount = CY_DMA_LOOP_COUNT_MAX;
        enableNextDscr = true;
        remainingCount = count - (CY_DMA_LOOP_COUNT_MAX * SMIF_RX_THRESHOLD_LEVEL);
    }


    /*
       RX trigger level is set as 3 bytes (ie, trigger will be generated when there are for 4 bytes in FIFO)
       For SMIF RX, data should be written from SMIF RX FIFO to HBDMA SRAM.

       Transfer is done as 2D DMA transfer:
       X Loop: Do SMIF_RX_THRESHOLD_LEVEL transfers on every input trigger
       Y Loop: Repeat till the required bytes are transferred (count parameter)

       4 bytes from RD1 transferred to HBDMA
       number of transfers after one input trigger (X_loop) = SMIF_RX_THRESHOLD_LEVEL
       srcYincrement (where to start the transfer after on X loop) : SMIF_RX_THRESHOLD_LEVEL
       */

    desc_cfg.retrigger       = CY_DMA_RETRIG_IM;
    desc_cfg.interruptType   = CY_DMA_DESCR_CHAIN;
    desc_cfg.triggerOutType  = CY_DMA_DESCR_CHAIN;
    desc_cfg.triggerInType   = CY_DMA_X_LOOP;
    desc_cfg.channelState    = CY_DMA_CHANNEL_ENABLED;
    desc_cfg.dataSize        = CY_DMA_BYTE;
    desc_cfg.srcTransferSize = CY_DMA_TRANSFER_SIZE_WORD;
    desc_cfg.dstTransferSize = CY_DMA_TRANSFER_SIZE_DATA;

    desc_cfg.descriptorType  = CY_DMA_2D_TRANSFER;
    desc_cfg.srcAddress      = (void *)(&(SMIF0->RX_DATA_FIFO_RD1));
    desc_cfg.dstAddress      = (void *)pHbSramBuffer;
    desc_cfg.srcXincrement   = 0u;
    desc_cfg.dstXincrement   = 1u;
    desc_cfg.xCount          = SMIF_RX_THRESHOLD_LEVEL;
    desc_cfg.dstYincrement   = SMIF_RX_THRESHOLD_LEVEL;
    desc_cfg.srcYincrement   = 0;
    desc_cfg.yCount          = yCount;
    desc_cfg.nextDescriptor  = (enableNextDscr)? &glSMIFReadDmaDescNext : NULL;

    stat = Cy_DMA_Descriptor_Init(&glSMIFReadDmaDesc, &desc_cfg);
    ASSERT_NON_BLOCK(stat == CY_DMA_SUCCESS, stat);

    if(enableNextDscr)
    {
        desc_cfg.retrigger       = CY_DMA_RETRIG_IM;
        desc_cfg.interruptType   = CY_DMA_DESCR_CHAIN;
        desc_cfg.triggerOutType  = CY_DMA_DESCR_CHAIN;
        desc_cfg.triggerInType   = CY_DMA_X_LOOP;
        desc_cfg.channelState    = CY_DMA_CHANNEL_DISABLED;
        desc_cfg.dataSize        = CY_DMA_BYTE;
        desc_cfg.srcTransferSize = CY_DMA_TRANSFER_SIZE_WORD;
        desc_cfg.dstTransferSize = CY_DMA_TRANSFER_SIZE_DATA;

        desc_cfg.descriptorType  = CY_DMA_2D_TRANSFER;
        desc_cfg.srcAddress      = (void *)(&(SMIF0->RX_DATA_FIFO_RD1));
        desc_cfg.dstAddress      = (void *)pHbSramBuffer + (CY_DMA_LOOP_COUNT_MAX * SMIF_RX_THRESHOLD_LEVEL);
        desc_cfg.srcXincrement   = 0u;
        desc_cfg.dstXincrement   = 1u;
        desc_cfg.xCount          = SMIF_RX_THRESHOLD_LEVEL;
        desc_cfg.dstYincrement   = SMIF_RX_THRESHOLD_LEVEL;
        desc_cfg.srcYincrement   = 0;
        desc_cfg.yCount          = (remainingCount / (SMIF_RX_THRESHOLD_LEVEL));
        desc_cfg.nextDescriptor  = NULL;

        stat = Cy_DMA_Descriptor_Init(&glSMIFReadDmaDescNext, &desc_cfg);
        ASSERT_NON_BLOCK(stat == CY_DMA_SUCCESS, stat);
    }

    chan_cfg.descriptor  = &glSMIFReadDmaDesc;
    chan_cfg.preemptable = false;
    chan_cfg.priority    = 0;
    chan_cfg.enable      = false;
    chan_cfg.bufferable  = false;

    stat = Cy_DMA_Channel_Init(SMIF_DW, SMIF_RX_DW_CHANNEL, &chan_cfg);
    ASSERT_NON_BLOCK(stat == CY_DMA_SUCCESS, stat);
    Cy_DMA_Channel_SetInterruptMask(SMIF_DW, SMIF_RX_DW_CHANNEL, CY_DMA_INTR_MASK);
}

#else
/*
Function     : Cy_SPI_InitDMACRX ()
Description :  Initialize DMAC receiver channels for SMIF RX FIFO to HBDMA buffer transfers.
Parameters  :  uint8_t * pHbSramBuffer, uint16_t count
Return      :  void

*/

void Cy_SPI_InitDMACRX(uint8_t * pHbSramBuffer, uint16_t count)
{
    cy_stc_dmac_descriptor_config_t desc_cfg;
    cy_stc_dmac_channel_config_t chan_cfg;
    cy_en_dmac_status_t stat;
    /*
       RX trigger level is set as 3 bytes (ie, trigger will be generated when there are for 4 bytes in FIFO)
       For SMIF RX, data should be written from SMIF RX FIFO to HBDMA SRAM.

       Transfer is done as 2D DMA transfer:
       X Loop: Do SMIF_RX_THRESHOLD_LEVEL transfers on every input trigger
       Y Loop: Repeat till the required bytes are transferred (count parameter)

       4 bytes from RD1 transferred to HBDMA
       number of transfers after one input trigger (X_loop) = SMIF_RX_THRESHOLD_LEVEL
       srcYincrement (where to start the transfer after on X loop) : SMIF_RX_THRESHOLD_LEVEL
       */

    desc_cfg.retrigger       = CY_DMAC_RETRIG_IM;
    desc_cfg.interruptType   = CY_DMAC_DESCR_CHAIN;
    desc_cfg.triggerOutType  = CY_DMAC_DESCR_CHAIN;
    desc_cfg.triggerInType   = CY_DMAC_X_LOOP;
    desc_cfg.channelState    = CY_DMAC_CHANNEL_DISABLED;
    desc_cfg.dataSize        = CY_DMAC_BYTE;
    desc_cfg.srcTransferSize = CY_DMAC_TRANSFER_SIZE_WORD;
    desc_cfg.dstTransferSize = CY_DMAC_TRANSFER_SIZE_DATA;

    desc_cfg.descriptorType  = CY_DMAC_2D_TRANSFER;
    desc_cfg.srcAddress      = (void *)(&(SMIF0->RX_DATA_FIFO_RD1));
    desc_cfg.dstAddress      = (void *)pHbSramBuffer;
    desc_cfg.srcXincrement   = 0u;
    desc_cfg.dstXincrement   = 1u;
    desc_cfg.xCount          = SMIF_RX_THRESHOLD_LEVEL;
    desc_cfg.dstYincrement   = SMIF_RX_THRESHOLD_LEVEL;
    desc_cfg.srcYincrement   = 0;
    desc_cfg.yCount          = (count / (SMIF_RX_THRESHOLD_LEVEL));
    desc_cfg.nextDescriptor  = NULL;

    stat = Cy_DMAC_Descriptor_Init(&glSMIFReadDmaDesc, &desc_cfg);
    ASSERT_NON_BLOCK(stat == CY_DMAC_SUCCESS, stat);

    chan_cfg.descriptor  = &glSMIFReadDmaDesc;
    chan_cfg.priority    = 0;
    chan_cfg.enable      = false;
    chan_cfg.bufferable  = false;

    stat = Cy_DMAC_Channel_Init(SMIF_DMAC, SMIF_RX_DMAC_CHANNEL, &chan_cfg);
    ASSERT_NON_BLOCK(stat == CY_DMAC_SUCCESS, stat);
    Cy_DMAC_Channel_SetInterruptMask(SMIF_DMAC, SMIF_RX_DMAC_CHANNEL, CY_DMAC_INTR_MASK);
}

#endif

static inline void Cy_SPI_InitReceiveDMA(uint8_t *buffer, uint32_t length)
{
#if USE_SMIF_DMAC
    Cy_SPI_InitDMACRX(buffer, length);
#else
    Cy_SPI_InitDWRX(buffer, length);
#endif /* USE_SMIF_DMAC */
}


static inline void Cy_SPI_EnableReceiveDMA(void)
{
#if USE_SMIF_DMAC
    Cy_DMAC_Channel_Enable(SMIF_DMAC, SMIF_RX_DMAC_CHANNEL);
#else
    Cy_DMA_Channel_Enable(SMIF_DW, SMIF_RX_DW_CHANNEL);
#endif /* USE_SMIF_DMAC */

}


/*
Function     : Cy_SPI_ReadOperation ()
Description :  Perform Read operation from the flash in Register mode.
Parameters  :  uint32_t address, uint8_t *rxBuffer, uint32_t length, cy_en_flash_index_t flashIndex
Return      :  cy_en_smif_status_t
*/

cy_en_smif_status_t Cy_SPI_ReadOperation(uint32_t address, uint8_t *rxBuffer, uint32_t length)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint8_t addrArray[CY_SPI_MAX_NUM_ADDR_BYTES] = {0};
    uint8_t numAddrBytes = smifMemConfigs[0]->deviceCfg->numOfAddrBytes;
    AddressToArray(address, addrArray,numAddrBytes);

    if(length < SMIF_RX_THRESHOLD_LEVEL)
    {
        DBG_APP_ERR("DMAC/DW currently uses word level transfers.Min len:%d\r\n",SMIF_RX_THRESHOLD_LEVEL);
        return CY_SMIF_BAD_PARAM;
    }

    Cy_SPI_InitReceiveDMA(rxBuffer, length);
    status = Cy_SMIF_MemCmdRead(SMIF_HW,smifMemConfigs[0],
            addrArray,
            NULL,
            length,
            NULL,
            &qspiContext);
    Cy_SPI_EnableReceiveDMA();
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    return status;
}

/*
Function     : Cy_App_CheckStatus ()
Description :  Function is used for checking ASSERT conditions in blocking or non-blockin mode.
Parameters  :  const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking
Return      :  void

*/

void Cy_App_CheckStatus(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking)
{
    if (!condition)
    {
        /* Application failed with the error code status */
#if DEBUG_INFRA_EN
        Cy_Debug_AddToLog(1, "Function %s failed at line %d with status = 0x%x\r\n", function, line, value);
#endif
        if (isBlocking)
        {
            /* Loop indefinitely */
            for (;;)
            {
            }
        }
    }
}

