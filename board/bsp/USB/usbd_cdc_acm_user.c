#include "usbd_core.h"
#include "usbd_cdc_acm.h"
#include "usbd_cdc_acm_user.h"
#include "cmsis_gcc.h"
#include "kfifo.h"
#include "tx_api.h"

/* 事件标志位定义 */
#define BSP_USB_EVENT_RX   ((ULONG)0x01) /* RX 数据就绪 */
#define BSP_USB_EVENT_TX   ((ULONG)0x02) /* TX 发送完成 */

/* 端点地址 */
#define CDC_IN_EP          0x81
#define CDC_OUT_EP         0x02
#define CDC_INT_EP         0x83

/* USB 设备标识 */
#define USBD_VID           0xFFFF
#define USBD_PID           0xFFFF
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

/* 配置描述符总长度 */
#define USB_CONFIG_SIZE    (9 + CDC_ACM_DESCRIPTOR_LEN)

/* 批量传输最大包长 */
#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif

/* RX kfifo 缓冲区大小（2 的幂，≥ CDC_MAX_MPS） */
#define CDC_RX_FIFO_SIZE 1024

/* USB 描述符 */

static const uint8_t device_descriptor[] = {USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01)};

static const uint8_t config_descriptor[] = {USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
                                            CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS, 0x02)};

static const uint8_t device_quality_descriptor[] = {
    /* 设备限定描述符 */
    0x0a, USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
};

static const char *string_descriptors[] = {
    (const char[]){0x09, 0x04}, /* 语言 ID */
    "mas_embedded",             /* 制造商 */
    "CDC ACM Device",           /* 产品名称 */
    "000000000001",             /* 序列号 */
};

/* 描述符回调 */

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return config_descriptor;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;
    if (index >= (sizeof(string_descriptors) / sizeof(char *)))
    {
        return NULL;
    }
    return string_descriptors[index];
}

static const struct usb_descriptor cdc_descriptor = {
    .device_descriptor_callback         = device_descriptor_callback,
    .config_descriptor_callback         = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .other_speed_descriptor_callback    = NULL,
    .string_descriptor_callback         = string_descriptor_callback,
    .msosv1_descriptor                  = NULL,
    .msosv2_descriptor                  = NULL,
    .webusb_url_descriptor              = NULL,
    .bos_descriptor                     = NULL,
};

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_read_buffer[CDC_RX_FIFO_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_write_buffer[CDC_MAX_MPS];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_out_ep_buffer[CDC_MAX_MPS];

static struct kfifo         cdc_rx_fifo;
static TX_EVENT_FLAGS_GROUP usb_event_flags;

/* USB 设备事件回调 */

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    (void)busid;
    switch (event)
    {
    case USBD_EVENT_RESET:
        break;
    case USBD_EVENT_CONNECTED:
        break;
    case USBD_EVENT_DISCONNECTED:
        break;
    case USBD_EVENT_RESUME:
        break;
    case USBD_EVENT_SUSPEND:
        break;
    case USBD_EVENT_CONFIGURED:
        /* 初始 TX 就绪标志，启动首个 OUT 端点读传输 */
        tx_event_flags_set(&usb_event_flags, BSP_USB_EVENT_TX, TX_OR);
        usbd_ep_start_read(busid, CDC_OUT_EP, cdc_out_ep_buffer, CDC_MAX_MPS);
        break;
    case USBD_EVENT_SET_REMOTE_WAKEUP:
        break;
    case USBD_EVENT_CLR_REMOTE_WAKEUP:
        break;
    default:
        break;
    }
}

/* CherryUSB 端点回调 */

/**
 * @brief 批量 OUT 端点回调（接收完成），推进 kfifo 写入索引。
 */
void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)ep;
    /* 将接收到的数据写入 kfifo */
    if (nbytes)
    {
        kfifo_in(&cdc_rx_fifo, cdc_out_ep_buffer, nbytes);
        /* 通知等待读取的任务有新数据 */
        tx_event_flags_set(&usb_event_flags, BSP_USB_EVENT_RX, TX_OR);
    }
    usbd_ep_start_read(busid, CDC_OUT_EP, cdc_out_ep_buffer, CDC_MAX_MPS);
    
}

/**
 * @brief 批量 IN 端点回调（发送完成）
 */
void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;

    if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes)
    {
        /* 发送 0 长度包（ZLP） */
        usbd_ep_start_write(busid, CDC_IN_EP, NULL, 0);
    }
    else
    {
        tx_event_flags_set(&usb_event_flags, BSP_USB_EVENT_TX, TX_OR);
    }
}

/* 端点 / 接口 注册结构体 */

static struct usbd_endpoint cdc_out_ep = {.ep_addr = CDC_OUT_EP, .ep_cb = usbd_cdc_acm_bulk_out};

static struct usbd_endpoint cdc_in_ep = {.ep_addr = CDC_IN_EP, .ep_cb = usbd_cdc_acm_bulk_in};

static struct usbd_interface intf0;
static struct usbd_interface intf1;

void cdc_acm_init(uint8_t busid, uintptr_t reg_base)
{
    kfifo_init(&cdc_rx_fifo, cdc_read_buffer, CDC_RX_FIFO_SIZE, 1);
    tx_event_flags_create(&usb_event_flags, "usb_evt");

    usbd_desc_register(busid, &cdc_descriptor);

    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf0));
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf1));
    usbd_add_endpoint(busid, &cdc_out_ep);
    usbd_add_endpoint(busid, &cdc_in_ep);

    usbd_initialize(busid, reg_base, usbd_event_handler);
}

int cdc_acm_send(const uint8_t *data, uint32_t len, uint32_t timeout)
{
    if (!data || len == 0 || len > CDC_MAX_MPS)
    {
        return -1;
    }

    /* 等待上次发送完成 */
    ULONG actual_flags = 0;
    if (tx_event_flags_get(&usb_event_flags, BSP_USB_EVENT_TX, TX_OR_CLEAR, &actual_flags, timeout) != TX_SUCCESS)
    {
        return -1;
    }

    memcpy(cdc_write_buffer, data, len);
    usbd_ep_start_write(0, CDC_IN_EP, cdc_write_buffer, len);
    return (int)len;
}

int cdc_acm_recv(uint8_t *data, uint32_t buf_size, uint32_t *rx_len, uint32_t timeout)
{
    if (!data || buf_size == 0)
    {
        if (rx_len) *rx_len = 0;
        return -1;
    }

    /* 等待 RX 数据就绪 */
    ULONG actual_flags = 0;
    if (tx_event_flags_get(&usb_event_flags, BSP_USB_EVENT_RX, TX_OR_CLEAR, &actual_flags, timeout) != TX_SUCCESS)
    {
        if (rx_len) *rx_len = 0;
        return -1;
    }

    /* 从 kfifo 读出数据 (不超过 buf_size) */
    uint32_t avail   = kfifo_len(&cdc_rx_fifo);
    uint32_t to_read = (avail <= buf_size) ? avail : buf_size;
    uint32_t actual  = kfifo_out(&cdc_rx_fifo, data, to_read);
    if (rx_len) *rx_len = actual;
    return (int)actual;
}
