/*
 * Copyright (c) 2022-2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * USB0 设备 - CDC ACM 虚拟串口 (CherryUSB 协议栈)
 *
 * 【文件结构理解】
 *   本文件是"USB 设备层", 上接协议栈(CherryUSB), 下接 BSP(bsp_usb.c)。
 *   三块内容:
 *     1) 描述符(device/config/string...) —— 告诉主机"我是谁、我怎么用",
 *        等价于 STM32 里的 usbd_desc.c / USBD_DescriptorsTypeDef;
 *     2) 事件与端点回调        —— 处理枚举过程、收发完成,
 *        等价于 STM32 的 usbd_cdc_if.c 里的收发回调;
 *     3) cdc_acm_init()        —— 注册描述符/接口/端点并启动,
 *        等价于 STM32 的 MX_USB_DEVICE_Init()。
 *
 * 【端点(Endpoint)概念, 与 STM32 一致】
 *   USB 通信靠端点。EP 地址最高位表示方向: 0x80 = IN(设备->主机),
 *   0x00 = OUT(主机->设备)。
 *     0x81 = IN  端点1 (设备发给主机, Bulk, 数据)
 *     0x01 = OUT 端点1 (主机发给设备, Bulk, 数据)
 *     0x83 = IN  端点3 (中断端点, 用于 CDC 的线路状态通知)
 */

#include "usbd_core.h"
#include "usbd_cdc_acm.h"
#include "bsp_usb.h"

/*!< endpoint address 端点地址(0x8x = IN 方向, 0x0x = OUT 方向) */
#define CDC_IN_EP  0x81     /* 设备 -> 主机 数据端点 */
#define CDC_OUT_EP 0x01     /* 主机 -> 设备 数据端点 */
#define CDC_INT_EP 0x83     /* 中断端点(CDC 通知) */

/*!< config descriptor size 配置描述符总长度 = 配置头(9) + CDC+数据接口 */
#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)

/* 设备描述符: 声明 VID/PID、USB 版本、设备类(0xEF/0x02/0x01 = IAD 复合设备) */
static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01)
};

static const uint8_t config_descriptor_hs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x02),
};

static const uint8_t config_descriptor_fs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_FS, 0x02),
};

static const uint8_t device_quality_descriptor[] = {
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, 0x01),
};

static const uint8_t other_speed_config_descriptor_hs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_FS, 0x02),
};

static const uint8_t other_speed_config_descriptor_fs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x02),
};

/* 字符串描述符: 索引 0 = 语言 ID, 1 = 厂商, 2 = 产品, 3 = 序列号。
 * PC 设备管理器里看到的"厂商/产品名"就来自这里。 */
static const char *string_descriptors[] = {
    (const char[]){ 0x09, 0x04 }, /* Langid: 0x0409 = 美式英语 */
    "HPMicro",                    /* Manufacturer 厂商 */
    "HPMicro CDC DEMO",           /* Product 产品名 */
    "2024051702",                 /* Serial Number 序列号 */
};

/**
 * @brief  返回设备描述符
 * @param  speed 当前 USB 速率(未使用)
 * @return 设备描述符指针
 * @note   协议栈在收到 GET_DESCRIPTOR(DEVICE) 时调用。
 *         对照 STM32: 相当于 USBD_GetDeviceDescriptor()。
 */
static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;

    return device_descriptor;
}

/**
 * @brief  按当前速率返回配置描述符
 * @param  speed USB_SPEED_HIGH / USB_SPEED_FULL
 * @return 对应速率的配置描述符; 未知速率返回 NULL
 * @note   高速和全速的端点包大小(Bulk MPS)不同, 所以准备了两份。
 */
static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    if (speed == USB_SPEED_HIGH) {
        return config_descriptor_hs;
    } else if (speed == USB_SPEED_FULL) {
        return config_descriptor_fs;
    } else {
        return NULL;
    }
}

/**
 * @brief  返回设备限定描述符(高速设备用)
 * @param  speed 当前速率(未使用)
 * @return 设备限定描述符指针
 */
static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void)speed;

    return device_quality_descriptor;
}

/**
 * @brief  返回"其他速率配置描述符"
 * @param  speed 当前速率
 * @return 另一种速率的配置描述符
 * @note   USB2.0 设备需在高速下也能被全速主机正确识别, 故提供此描述符。
 */
static const uint8_t *other_speed_config_descriptor_callback(uint8_t speed)
{
    if (speed == USB_SPEED_HIGH) {
        return other_speed_config_descriptor_hs;
    } else if (speed == USB_SPEED_FULL) {
        return other_speed_config_descriptor_fs;
    } else {
        return NULL;
    }
}

/**
 * @brief  按索引返回字符串描述符
 * @param  speed 当前速率(未使用)
 * @param  index 字符串索引(0=语言, 1=厂商, 2=产品, 3=序列号)
 * @return 字符串指针; 索引越界返回 NULL
 */
static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;

    if (index >= (sizeof(string_descriptors) / sizeof(char *))) {
        return NULL;
    }
    return string_descriptors[index];
}

/* 把上面 5 个回调打包注册给协议栈(类似 STM32 的 USBD_RegisterClass 时提供的描述符表) */
const struct usb_descriptor cdc_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .other_speed_descriptor_callback = other_speed_config_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
};

/* 收发缓冲区。
 * USB 使用 DMA(或需要与 Cache 一致的内存), 因此必须放在
 *   - NOCACHE_RAM_SECTION: 非缓存段, 防止 Cache 与 DMA 数据不一致
 *   - MEM_ALIGNX          : 对齐到 USB 要求的字节边界
 * 对照 STM32: 相当于 __attribute__((section(".nocache"))) 或 MPU 配 noncacheable。
 *
 * read_buffer[2][512]: 双缓冲 —— 读缓冲 0 正在被协议栈/协议处理时,
 *                      读缓冲 1 可以接收下一包, 防止覆盖。
 */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_buffer[2][512];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t write_buffer[2048];   /* 大包测试用缓冲 */
volatile uint8_t read_buffer_index;   /* 当前正在使用哪个读缓冲(0/1) */
volatile bool dtr_enable;             /* 主机是否打开串口(DTR), 被 bsp_usb_connected() 读取 */
volatile bool ep_tx_busy_flag;        /* 发送忙标志(测试函数用) */

/**
 * @brief  USB 设备事件处理(枚举过程的状态机入口)
 * @param  busid  总线编号
 * @param  event  事件类型
 *
 * 【原理】
 *   USB 枚举是一系列控制传输(复位 -> 取描述符 -> 设置地址 -> 设置配置 ...)。
 *   协议栈把关键节点以事件形式回调给应用。
 *   这里只在 USBD_EVENT_CONFIGURED(主机下发了 SET_CONFIGURATION, 配置完成,
 *   此时设备才算真正"可用")时, 发起第一次 OUT 端点读请求 —— 从此开始,
 *   主机发来的数据就能被接收。
 *
 *   注意: USB 是主机主导的总线, 设备必须"预先提交"接收请求,
 *         所以每个 OUT 传输完成后都要重新 usbd_ep_start_read(),
 *         这一点在 STM32 的 CDC_Receive_FS 里也是一样的(需要重新 ARM)。
 */
static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    switch (event) {
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
        /* 配置完成: 挂上第一笔 OUT 读请求, 准备接收主机数据 */
        read_buffer_index = 0;
        usbd_ep_start_read(busid, CDC_OUT_EP, &read_buffer[0][0], usbd_get_ep_mps(busid, CDC_OUT_EP));
        break;
    case USBD_EVENT_SET_REMOTE_WAKEUP:
        break;
    case USBD_EVENT_CLR_REMOTE_WAKEUP:
        break;

    default:
        break;
    }
}

/**
 * @brief  OUT 端点(主机 -> 设备)接收完成回调
 * @param  busid   总线编号
 * @param  ep      端点地址(CDC_OUT_EP)
 * @param  nbytes  本次收到的字节数
 *
 * 【做了什么】
 *   1. 切换读缓冲索引(双缓冲);
 *   2. 把刚收到的数据"原样回显"给主机(loopback, 方便用串口助手验证链路);
 *   3. 立刻再挂一笔读请求, 保证能继续收下一包;
 *   4. 调用 BSP 用户回调 bsp_usb_rx_callback() 交给应用处理。
 *
 * 对照 STM32: 相当于 CDC_Receive_FS() 回调。
 */
void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    uint8_t index = read_buffer_index;
    USB_LOG_RAW("actual out len:%d\r\n", nbytes);

    read_buffer_index = (index == 0) ? 1 : 0;                              /* 切到另一个读缓冲 */
    usbd_ep_start_write(busid, CDC_IN_EP, &read_buffer[index][0], nbytes); /* 回显 */
    usbd_ep_start_read(busid, ep, &read_buffer[read_buffer_index][0], usbd_get_ep_mps(busid, ep)); /* 重新挂读 */

    /* 交给 BSP 用户回调处理主机下发的数据 */
    bsp_usb_rx_callback(&read_buffer[index][0], nbytes);
}

/**
 * @brief  向主机发送数据 (供 BSP 调用的统一入口)
 * @param  busid  总线编号
 * @param  data   数据指针(须为非缓存区/静态数据, 因发送是异步的)
 * @param  len    字节数
 *
 * 对照 STM32: 相当于 CDC_Transmit_FS()。是"提交请求"语义, 立即返回。
 */
void cdc_acm_send(uint8_t busid, const uint8_t *data, uint32_t len)
{
    if (len == 0U) {
        return;
    }
    usbd_ep_start_write(busid, CDC_IN_EP, data, len);   /* 提交 IN 端点发送 */
}

/**
 * @brief  IN 端点(设备 -> 主机)发送完成回调
 * @param  busid   总线编号
 * @param  ep      端点地址
 * @param  nbytes  本次实际发出的字节数
 *
 * 【为什么会有 ZLP?】
 *   USB Bulk 传输用"短包"表示一次传输结束。若发送长度正好是端点包大小的整数倍
 *   (例如 512 字节且 MPS=512), 主机会认为"还没结束, 后面还有", 会一直等。
 *   所以必须再发一个长度为 0 的包(ZLP, Zero Length Packet)来收尾。
 *   这是 USB 协议细节, 与 STM32 上使用 HAL_PCD_EP_Transmit 时遇到的情况相同。
 */
void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("actual in len:%d\r\n", nbytes);

    if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes) {
        /* 长度是最大包长整数倍 -> 补发 ZLP 表示传输结束 */
        usbd_ep_start_write(busid, ep, NULL, 0);
    } else {
        ep_tx_busy_flag = false;    /* 传输真正结束, 清忙标志 */
    }
}

/* 端点注册表: 把端点地址和对应回调绑定。
 * 对照 STM32: 相当于 USBD_LL_OpenEP + 在 CDC 类里注册 IN/OUT 回调。 */
struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out      /* 收到数据时回调 */
};

struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in       /* 发完数据时回调 */
};

/* CDC 是"两个接口"的设备: 接口0 = 控制(ACM), 接口1 = 数据(Data) */
static struct usbd_interface intf0;
static struct usbd_interface intf1;

/* function ------------------------------------------------------------------*/

/**
 * @brief  初始化 CDC ACM 设备 (注册描述符/接口/端点并启动协议栈)
 * @param  busid     总线编号(单 USB 设备固定为 0)
 * @param  reg_base  USB 控制器寄存器基址
 *
 * 流程(对照 STM32 MX_USB_DEVICE_Init()):
 *   1. usbd_desc_register()  注册描述符回调表(设备/配置/字符串...)
 *   2. usbd_add_interface()  加入 CDC 控制接口 + 数据接口
 *   3. usbd_add_endpoint()   加入数据 OUT/IN 端点及其回调
 *   4. usbd_initialize()     初始化控制器、挂中断、连接总线(开始枚举)
 *
 * 之后主机就会开始枚举, 枚举完成后触发 USBD_EVENT_CONFIGURED。
 */
void cdc_acm_init(uint8_t busid, uint32_t reg_base)
{
    usbd_desc_register(busid, &cdc_descriptor);
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf0));
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf1));
    usbd_add_endpoint(busid, &cdc_out_ep);
    usbd_add_endpoint(busid, &cdc_in_ep);
    usbd_initialize(busid, reg_base, usbd_event_handler);
}

/**
 * @brief  CDC 类请求回调: 主机设置/清除 DTR
 * @param  busid  总线编号
 * @param  intf   接口号
 * @param  dtr    true = 主机打开串口, false = 关闭
 *
 * 【用途】
 *   DTR 是"Data Terminal Ready", 上位机打开串口时置 1。BSP 用它来判断
 *   "主机是否在听", 从而决定要不要发数据(bsp_usb_connected())。
 *
 * @note   这里只处理了 dtr==true 的情况(置位后不再清零), 如需感知关闭串口,
 *         可补上 else 分支 dtr_enable = 0。
 */
void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr)
{
    (void)busid;
    (void)intf;

    if (dtr) {
        dtr_enable = 1;
    }
}

/**
 * @brief  USB 发送自测: 发送 2048 字节 'a' 并阻塞等待发完
 * @param  busid  总线编号
 * @note   演示"整包 + ZLP"机制: 2048 正好是高速端点的整数倍, 所以会补发 ZLP。
 *         通过 while(ep_tx_busy_flag) 等待, 是阻塞式写法, 仅用于自测。
 */
void cdc_acm_data_send_with_dtr_test(uint8_t busid)
{
    ep_tx_busy_flag = true;
    write_buffer[0] = 'h';
    write_buffer[1] = 'p';
    write_buffer[2] = 'm';
    write_buffer[3] = 'i';
    write_buffer[4] = 'c';
    write_buffer[5] = 'r';
    write_buffer[6] = 'o';
    write_buffer[7] = 0x0D;     /* '\r' */
    write_buffer[8] = 0x0A;     /* '\n' */
    memset(&write_buffer[9], 'a', 2037);   /* 中间填 'a' */
    write_buffer[2046] = 0x0D;
    write_buffer[2047] = 0x0A;
    usbd_ep_start_write(busid, CDC_IN_EP, &write_buffer[0], 2048);
    while (ep_tx_busy_flag) {   /* 阻塞等待发送完成 */
    }
}