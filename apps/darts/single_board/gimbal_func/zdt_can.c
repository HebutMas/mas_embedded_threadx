/*
 * @Description: 张大头 ZDT 闭环步进 CAN 直发驱动 (darts 本地, 仅 dji_c 编译)
 *               协议: X42S 手册 4.2 章 — 扩展帧, ID = (电机地址<<8)|包序号,
 *               命令去掉地址字节后拆 8 字节包, 每包首字节重复功能码
 */
#include "zdt_can.h"
#include "bsp_can.h"

#define LOG_TAG "app_zdt_can"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define ZDT_CMD_ENABLE   0xF3
#define ZDT_CMD_POSITION 0xFD
#define ZDT_CMD_STOP     0xFE
#define ZDT_CHECKSUM     0x6B
#define ZDT_REPLY_ID     0x100 /* (addr=1)<<8 | 包号, 回包与命令同 ID */

static CAN_HandleTypeDef *zdt_hcan;
static uint8_t            zdt_addr;
static bool               zdt_started;
static bool               zdt_enabled;

/* Ozone 调试观察: 模块回包 (bsp 分发只认标准帧, 扩展帧回包由本地轮询 FIFO1 读取) */
uint8_t  zdt_dbg_reply_status = 0xFF; /* 最近回包状态字节: 02=接受 EE=格式错 E2=参数错, 0xFF=还没收到 */
uint32_t zdt_dbg_reply_count  = 0;    /* 收到的回包数 */
uint32_t zdt_dbg_can_tsr      = 0;    /* hcan2 TSR: bit28/27/26=TME2/1/0 邮箱空闲, 0x1C000000=全空闲, 全0=堵死 */
uint32_t zdt_dbg_can_esr      = 0;    /* hcan2 ESR: bit23..16=TEC 发送错误计数, 0=模块在ACK, 在涨=无ACK */

static bool zdt_can_send_packet(uint8_t packet_idx, const uint8_t *data, uint8_t len)
{
    if (!zdt_started) return false;

    CAN_TxHeaderTypeDef header = {
        .ExtId = ((uint32_t)zdt_addr << 8) | packet_idx,
        .IDE   = CAN_ID_EXT,
        .RTR   = CAN_RTR_DATA,
        .DLC   = len,
    };
    uint32_t mailbox;
    /* 返回入队结果，失能命令失败时由下个控制周期重试。 */
    if (HAL_CAN_AddTxMessage(zdt_hcan, &header, (uint8_t *)data, &mailbox) != HAL_OK)
    {
        LOG_E("zdt can tx busy");
        return false;
    }
    return true;
}

void zdt_can_init(CAN_HandleTypeDef *hcan, uint8_t addr)
{
    zdt_hcan    = hcan;
    zdt_addr    = addr;
    zdt_enabled = false;
    zdt_started = false;

    /* CAN 总线由 BSP 统一启动，CAN2 可能已经被超级电容模块启动。
     * 注意: 模块回包是扩展帧, bsp 的分发只认标准帧 → 回包由 zdt_can_poll_reply 本地读 */
    Can_Device_Init_Config_s rx_cfg = {
        .hcan        = hcan,
        .tx_id       = ZDT_REPLY_ID,
        .rx_id       = ZDT_REPLY_ID,
        .rx_callback = NULL,
        .user_arg    = NULL,
    };
    Can_Device *rx_dev = BSP_CAN_Device_Init(&rx_cfg);
    if (rx_dev == NULL)
    {
        LOG_E("zdt rx device init failed");
        return;
    }
    zdt_started = (HAL_CAN_GetState(hcan) == HAL_CAN_STATE_LISTENING);

    /* 回包进 FIFO1 (bsp 按 rx_id 奇偶分配), 关掉它的中断改本地轮询,
     * 否则 bsp 的 IRQ 会把扩展帧读走再按标准帧 ID 丢掉 */
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

    /* bsp 注册设备的过滤器是 16 位, 只比较 ID[28:18]+ID[17:15], 匹配不了扩展帧 0x100/0x101。
     * 将本设备分配到的过滤器改为 32 位: 只放行扩展帧 0x100/0x101。
     * SlaveStartFilterBank 必须 14, 和 bsp 一致, 否则会改写 CAN1/CAN2 的过滤器组分配 */
    CAN_FilterTypeDef rx_filter = {
        .FilterIdHigh         = 0x0000,
        .FilterIdLow          = (uint16_t)((ZDT_REPLY_ID << 3) | CAN_ID_EXT), /* FR1: ID<<3 | IDE<<2 | RTR<<1 */
        .FilterMaskIdHigh     = 0xFFFF,
        .FilterMaskIdLow      = 0xFFF7, /* ID bit0 不关心: 0x100 和 0x101 都放行 */
        .FilterFIFOAssignment = CAN_FILTER_FIFO1,
        .FilterBank           = rx_dev->filter_bank_index,
        .FilterMode           = CAN_FILTERMODE_IDMASK,
        .FilterScale          = CAN_FILTERSCALE_32BIT,
        .FilterActivation     = CAN_FILTER_ENABLE,
        .SlaveStartFilterBank = 14,
    };
    if (HAL_CAN_ConfigFilter(hcan, &rx_filter) != HAL_OK) LOG_E("zdt rx filter config failed");

    /* 上电先失能，遥控器在线后由云台控制路径使能。 */
    zdt_can_set_enable(false);

    if (!zdt_started) LOG_E("zdt can start failed");
    LOG_I("zdt can init (addr=%d)", addr);
}

/* 每周期轮询 FIFO1 读回包, 更新 Ozone 观察变量 */
void zdt_can_poll_reply(void)
{
    if (!zdt_started) return;

    zdt_dbg_can_tsr = zdt_hcan->Instance->TSR;
    zdt_dbg_can_esr = zdt_hcan->Instance->ESR;

    while (HAL_CAN_GetRxFifoFillLevel(zdt_hcan, CAN_RX_FIFO1) > 0)
    {
        CAN_RxHeaderTypeDef header;
        uint8_t             data[8];
        if (HAL_CAN_GetRxMessage(zdt_hcan, CAN_RX_FIFO1, &header, data) != HAL_OK) break;
        zdt_dbg_reply_count++;
        if (header.DLC >= 2) zdt_dbg_reply_status = data[1];
    }
}

void zdt_can_set_enable(bool enable)
{
    /* 失能帧持续重发，避免邮箱忙或模块重启后仍保持使能。 */
    if (enable && zdt_enabled) return;

    if (enable)
    {
        /* 沿用 ZDT 驱动的立即停止指令，先取消旧运动目标再恢复使能。 */
        uint8_t stop[] = {ZDT_CMD_STOP, 0x98, 0x00, ZDT_CHECKSUM};
        if (!zdt_can_send_packet(0, stop, sizeof(stop))) return;
    }

    /* 串口帧 01 F3 AB 01 00 6B 去掉地址 → 单包 */
    uint8_t data[] = {ZDT_CMD_ENABLE, 0xAB, enable ? 0x01 : 0x00, 0x00, ZDT_CHECKSUM};
    if (zdt_can_send_packet(0, data, sizeof(data))) zdt_enabled = enable;
}

void zdt_can_send_position(int32_t pulses, uint16_t speed_rpm)
{
    if (!zdt_enabled) return;

    uint8_t  dir = (pulses >= 0) ? 0x00 : 0x01;
    uint32_t abs = (uint32_t)((pulses >= 0) ? pulses : -pulses);

    /* 串口帧 01 FD dir spdH spdL accel pos0..3 mode sync 6B 去掉地址 = 12 字节 → 拆两包 */
    uint8_t pack0[8] = {
        ZDT_CMD_POSITION,
        dir,
        (uint8_t)(speed_rpm >> 8),
        (uint8_t)(speed_rpm & 0xFF),
        0, /* 加速度档位 */
        (uint8_t)(abs >> 24),
        (uint8_t)(abs >> 16),
        (uint8_t)(abs >> 8),
    };
    uint8_t pack1[5] = {
        ZDT_CMD_POSITION,
        (uint8_t)(abs & 0xFF),
        0, /* 运动模式: 相对上一目标位置 */
        0, /* 同步标志: 立即执行 */
        ZDT_CHECKSUM,
    };

    zdt_can_send_packet(0, pack0, sizeof(pack0));
    zdt_can_send_packet(1, pack1, sizeof(pack1));
}
