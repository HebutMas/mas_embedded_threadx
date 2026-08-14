#include "bsp_flash.h"
#include "tx_api.h"
#include <string.h>

#if defined(STM32F407xx)

#include "stm32f4xx_hal.h"

/* 用户Flash区域：第11扇区，地址0x080E0000，大小128KB */
#define USER_FLASH_SECTOR      FLASH_SECTOR_11
#define USER_FLASH_SECTOR_ADDR 0x080E0000U
#define USER_FLASH_SECTOR_SIZE (128U * 1024U)

/**
 * @description: 擦除用户Flash扇区
 * @return {uint8_t} 0成功，1失败
 */
static uint8_t BSP_FLASH_Erase_Sector(void)
{
    /* 清除残留错误标志，防止上游代码遗留的 PGSERR/PGPERR 导致 HAL 返回错误 */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef erase = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
        .Sector       = USER_FLASH_SECTOR,
        .NbSectors    = 1,
    };
    uint32_t page_error = 0;

    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

uint8_t BSP_FLASH_Write_Buffer(uint8_t *buffer, uint32_t length)
{
    if (buffer == NULL || length == 0 || length > USER_FLASH_SECTOR_SIZE)
    {
        return 1;
    }

    HAL_FLASH_Unlock();

    /* 擦除扇区（Erase_Sector 内部会清除残留错误标志） */
    if (BSP_FLASH_Erase_Sector() != 0)
    {
        HAL_FLASH_Lock();
        return 1;
    }

    /* 按半字(16bit)写入，提高写入效率 */
    uint32_t addr = USER_FLASH_SECTOR_ADDR;
    uint32_t end  = addr + length;
    uint8_t *src  = buffer;
    uint8_t  ret  = 0;

    while (addr < end)
    {
        uint32_t remaining = end - addr;

        if (remaining >= 2)
        {
            /* 半字写入 */
            uint16_t data = (uint16_t)src[0] | ((uint16_t)src[1] << 8);
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, data) != HAL_OK)
            {
                ret = 1;
                break;
            }
            addr += 2;
            src += 2;
        }
        else
        {
            /* 末尾奇数字节 */
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr, *src) != HAL_OK)
            {
                ret = 1;
                break;
            }
            addr += 1;
            src += 1;
        }
    }

    HAL_FLASH_Lock();
    return ret;
}

uint8_t BSP_FLASH_Read_Buffer(uint8_t *buffer, uint32_t length)
{
    if (buffer == NULL || length == 0 || length > USER_FLASH_SECTOR_SIZE)
    {
        return 1;
    }

    memcpy(buffer, (uint8_t *)USER_FLASH_SECTOR_ADDR, length);
    return 0;
}

#elif defined(STM32H723xx)

#include "octospi.h"

#define LOG_TAG "bsp_flash"
#define LOG_LVL LOG_LVL_WARNING
#include "ulog_def.h"

/* W25Q64 命令与参数 */
#define W25Qxx_CMD_EnableReset          0x66
#define W25Qxx_CMD_ResetDevice          0x99
#define W25Qxx_CMD_JedecID              0x9F
#define W25Qxx_CMD_WriteEnable          0x06
#define W25Qxx_CMD_SectorErase          0x20
#define W25Qxx_CMD_BlockErase_64K       0xD8
#define W25Qxx_CMD_ChipErase            0xC7
#define W25Qxx_CMD_QuadInputPageProgram 0x32
#define W25Qxx_CMD_FastReadQuad_IO      0xEB
#define W25Qxx_CMD_ReadStatus_REG1      0x05
#define W25Qxx_Status_REG1_BUSY         0x01
#define W25Qxx_Status_REG1_WEL          0x02

#define W25Qxx_PageSize                 256
#define W25Qxx_FlashSize                0x800000 /* 8MB */
#define W25Qxx_FLASH_ID                 0xEF4017
#define W25Qxx_ChipErase_TIMEOUT_MAX    100000U

/* OSPI 句柄 */
extern OSPI_HandleTypeDef hospi2;

/* 初始化标志 */
static uint8_t w25qxx_inited = 0;

/**
 * @description: 自动轮询等待W25Qxx空闲
 */
static uint8_t OSPI_W25Qxx_AutoPollingMemReady(void)
{
    OSPI_RegularCmdTypeDef  cmd;
    OSPI_AutoPollingTypeDef poll;

    memset(&cmd, 0, sizeof(cmd));
    cmd.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId            = HAL_OSPI_FLASH_ID_1;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.Instruction        = W25Qxx_CMD_ReadStatus_REG1;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_NONE;
    cmd.DataMode           = HAL_OSPI_DATA_1_LINE;
    cmd.NbData             = 1;
    cmd.DummyCycles        = 0;
    cmd.DQSMode            = HAL_OSPI_DQS_DISABLE;
    cmd.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    if (HAL_OSPI_Command(&hospi2, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return 1;
    }

    memset(&poll, 0, sizeof(poll));
    poll.Match         = 0;
    poll.Mask          = W25Qxx_Status_REG1_BUSY;
    poll.MatchMode     = HAL_OSPI_MATCH_MODE_AND;
    poll.Interval      = 0x10;
    poll.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE;

    if (HAL_OSPI_AutoPolling(&hospi2, &poll, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

/**
 * @description: 读取W25Q64 JEDEC ID
 */
static uint32_t W25Qxx_ReadID(void)
{
    OSPI_RegularCmdTypeDef cmd;
    uint8_t                buf[3];

    memset(&cmd, 0, sizeof(cmd));
    cmd.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId            = HAL_OSPI_FLASH_ID_1;
    cmd.Instruction        = W25Qxx_CMD_JedecID;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_NONE;
    cmd.DataMode           = HAL_OSPI_DATA_1_LINE;
    cmd.NbData             = 3;
    cmd.DummyCycles        = 0;
    cmd.DQSMode            = HAL_OSPI_DQS_DISABLE;
    cmd.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    HAL_OSPI_Command(&hospi2, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
    HAL_OSPI_Receive(&hospi2, buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
}

/**
 * @description: 写使能
 */
static uint8_t W25Qxx_WriteEnable(void)
{
    OSPI_RegularCmdTypeDef  cmd;
    OSPI_AutoPollingTypeDef poll;

    /* 发送写使能命令 */
    memset(&cmd, 0, sizeof(cmd));
    cmd.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId            = HAL_OSPI_FLASH_ID_1;
    cmd.Instruction        = W25Qxx_CMD_WriteEnable;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_NONE;
    cmd.DataMode           = HAL_OSPI_DATA_NONE;
    cmd.DQSMode            = HAL_OSPI_DQS_DISABLE;
    cmd.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    if (HAL_OSPI_Command(&hospi2, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return 1;
    }

    /* 轮询等待 WEL 置位 */
    memset(&cmd, 0, sizeof(cmd));
    cmd.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId            = HAL_OSPI_FLASH_ID_1;
    cmd.Instruction        = W25Qxx_CMD_ReadStatus_REG1;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_NONE;
    cmd.DataMode           = HAL_OSPI_DATA_1_LINE;
    cmd.NbData             = 1;
    cmd.DummyCycles        = 0;
    cmd.DQSMode            = HAL_OSPI_DQS_DISABLE;
    cmd.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    if (HAL_OSPI_Command(&hospi2, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return 1;
    }

    memset(&poll, 0, sizeof(poll));
    poll.Match         = 0x02;
    poll.Mask          = W25Qxx_Status_REG1_WEL;
    poll.MatchMode     = HAL_OSPI_MATCH_MODE_AND;
    poll.Interval      = 0x10;
    poll.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE;

    if (HAL_OSPI_AutoPolling(&hospi2, &poll, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

/**
 * @description: 64KB块擦除
 */
static uint8_t W25Qxx_BlockErase_64K(uint32_t sector_addr)
{
    OSPI_RegularCmdTypeDef cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId            = HAL_OSPI_FLASH_ID_1;
    cmd.Instruction        = W25Qxx_CMD_BlockErase_64K;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.Address            = sector_addr;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
    cmd.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
    cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode           = HAL_OSPI_DATA_NONE;
    cmd.DummyCycles        = 0;
    cmd.DQSMode            = HAL_OSPI_DQS_DISABLE;
    cmd.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    if (W25Qxx_WriteEnable() != 0)
    {
        return 1;
    }
    if (HAL_OSPI_Command(&hospi2, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return 1;
    }
    if (OSPI_W25Qxx_AutoPollingMemReady() != 0)
    {
        return 1;
    }
    return 0;
}

/**
 * @description: 按页写入，最大256字节
 */
static uint8_t W25Qxx_WritePage(uint8_t *data, uint32_t addr, uint16_t len)
{
    OSPI_RegularCmdTypeDef cmd;

    if (len > W25Qxx_PageSize)
    {
        len = W25Qxx_PageSize;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId            = HAL_OSPI_FLASH_ID_1;
    cmd.Instruction        = W25Qxx_CMD_QuadInputPageProgram;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.Address            = addr;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
    cmd.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
    cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode           = HAL_OSPI_DATA_4_LINES;
    cmd.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
    cmd.NbData             = len;
    cmd.DummyCycles        = 0;
    cmd.DQSMode            = HAL_OSPI_DQS_DISABLE;
    cmd.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    if (W25Qxx_WriteEnable() != 0)
    {
        return 1;
    }
    if (HAL_OSPI_Command(&hospi2, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return 1;
    }
    if (HAL_OSPI_Transmit(&hospi2, data, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return 1;
    }
    if (OSPI_W25Qxx_AutoPollingMemReady() != 0)
    {
        return 1;
    }
    return 0;
}

/**
 * @description: 初始化W25Q64（内部调用）
 */
static uint8_t W25Qxx_Init(void)
{
    uint32_t id = W25Qxx_ReadID();
    if (id != W25Qxx_FLASH_ID)
    {
        LOG_E("W25Q64 init failed, ID=0x%06X", id);
        return 1;
    }
    LOG_I("W25Q64 init success, ID=0x%06X", id);
    return 0;
}

/* 外部接口 */

uint8_t BSP_FLASH_Write_Buffer(uint8_t *buffer, uint32_t length)
{
    if (buffer == NULL || length == 0 || length > W25Qxx_FlashSize)
    {
        return 1;
    }

    /* 首次使用时自动初始化 */
    if (!w25qxx_inited)
    {
        if (W25Qxx_Init() != 0)
        {
            return 1;
        }
        w25qxx_inited = 1;
    }

    /* 擦除从地址0开始的所需64KB块 */
    uint32_t end_addr   = length;
    uint32_t num_blocks = (end_addr + (64U * 1024U) - 1) / (64U * 1024U);
    for (uint32_t i = 0; i < num_blocks; i++)
    {
        if (W25Qxx_BlockErase_64K(i * 64U * 1024U) != 0)
        {
            LOG_E("BlockErase failed at block %lu", i);
            return 1;
        }
    }

    /* 按页写入 */
    uint32_t addr      = 0;
    uint32_t remaining = length;
    uint8_t *src       = buffer;

    while (remaining > 0)
    {
        uint16_t chunk = (remaining > W25Qxx_PageSize) ? W25Qxx_PageSize : (uint16_t)remaining;
        if (W25Qxx_WritePage(src, addr, chunk) != 0)
        {
            LOG_E("Page write failed at addr 0x%08lX", addr);
            return 1;
        }
        addr += chunk;
        src += chunk;
        remaining -= chunk;
    }

    return 0;
}

uint8_t BSP_FLASH_Read_Buffer(uint8_t *buffer, uint32_t length)
{
    if (buffer == NULL || length == 0 || length > W25Qxx_FlashSize)
    {
        return 1;
    }

    /* 首次使用时自动初始化 */
    if (!w25qxx_inited)
    {
        if (W25Qxx_Init() != 0)
        {
            return 1;
        }
        w25qxx_inited = 1;
    }

    OSPI_RegularCmdTypeDef cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId            = HAL_OSPI_FLASH_ID_1;
    cmd.Instruction        = W25Qxx_CMD_FastReadQuad_IO;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.Address            = 0;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_4_LINES;
    cmd.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
    cmd.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
    cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode           = HAL_OSPI_DATA_4_LINES;
    cmd.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
    cmd.NbData             = length;
    cmd.DummyCycles        = 6;
    cmd.DQSMode            = HAL_OSPI_DQS_DISABLE;
    cmd.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    if (HAL_OSPI_Command(&hospi2, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return 1;
    }
    if (HAL_OSPI_Receive(&hospi2, buffer, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return 1;
    }
    if (OSPI_W25Qxx_AutoPollingMemReady() != 0)
    {
        return 1;
    }
    return 0;
}

#endif
