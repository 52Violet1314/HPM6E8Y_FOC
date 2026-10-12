/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 校准数据的 Flash 持久化实现
 *
 * 存储位置: NOR 尾部 64KB 区的首扇区 0x803F0000 (实测 Flash = 4MB),
 * 应用镜像只占头部 ~128KB, 相隔约 3.8MB, 擦写这里不会碰程序。
 * 布局 (小端):
 *   [0..3]    magic  0x3154554C ("LTU1")
 *   [4..7]    version = 1
 *   [8..11]   pole pairs
 *   [12..15]  offset_mrad (标定偏置, int32)
 *   [16..23]  reserved
 *   [24..27]  crc32 (覆盖 0..23 与 28..2075)
 *   [28..2075] LUT[1024] int16
 */
#include "app_nv.h"

#include <string.h>

#include "board.h"
#include "hpm_csr_drv.h"
#include "hpm_l1c_drv.h"
#include "hpm_romapi.h"

#include "app_diag.h"
#include "foc.h"

/* 【实测】本板 NOR 只有 4MB, board.h 的 BOARD_FLASH_SIZE=16MB 是错的:
 * OpenOCD 用 SFDP 探到的扇区表是 1024 × 4KB, 最后一个扇区偏移 0x3FF000。
 * 之前按 16MB 取地址 0x80FF0000 存表 -> 读越界 -> mcause=5 (load access
 * fault) -> 默认 trap 死循环, 表现就是启动打一半卡死。
 * 现在用 4MB 空间里的最后一个 64KB 区的第一扇区: 0x803F0000。 */
#define APP_NV_FLASH_ADDR    (BOARD_FLASH_BASE_ADDRESS + 0x003F0000UL)
#define APP_NV_SECTOR_BYTES  (4096U)
#define APP_NV_MAGIC         (0x3154554CUL)   /* "LTU1" */
/* 换电机后作废旧电机的 LUT/偏置存档。 */
#define APP_NV_VERSION       (2UL)
#define APP_NV_HEADER_BYTES  (28U)
#define APP_NV_TABLE_OFFSET  (APP_NV_HEADER_BYTES)

static xpi_nor_config_t s_nor;
static bool             s_ready;
static uint8_t          s_buf[APP_NV_SECTOR_BYTES];

static uint32_t nv_crc32_update(uint32_t crc, const uint8_t *p, uint32_t n)
{
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < n; i++) {
        crc ^= (uint32_t)p[i];
        for (j = 0U; j < 8U; j++) {
            if ((crc & 1U) != 0U) {
                crc = (crc >> 1U) ^ 0xEDB88320U;
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

static bool nv_header_ok(const uint8_t *b)
{
    uint32_t magic;
    uint32_t ver;
    uint32_t crc;
    uint32_t calc;

    (void)memcpy(&magic, b + 0U, 4U);
    (void)memcpy(&ver,   b + 4U, 4U);
    (void)memcpy(&crc,   b + 24U, 4U);
    if ((magic != APP_NV_MAGIC) || (ver != APP_NV_VERSION)) {
        return false;
    }

    calc  = nv_crc32_update(0xFFFFFFFFU, b, 24U);
    calc  = nv_crc32_update(calc, b + APP_NV_TABLE_OFFSET, ENC_LUT_BINS * 2U);
    calc ^= 0xFFFFFFFFU;

    return (calc == crc);
}

ATTR_RAMFUNC
void app_nv_init(void)
{
    xpi_nor_config_option_t opt;

    if (s_ready) {
        return;
    }

    /* 【必须与启动 ROM 用的配置完全一致】本板 board.c 的 .nor_cfg_option
     * 段 = { 0xfcf90002, 0x00000007, 0x1000, 0 } (SFDP SDR, 133MHz)。
     * board.h 里的 BOARD_APP_XPI_NOR_CFG_OPT_OPT0=0x5 是 100MHz, 和正在
     * 跑的 XIP 状态不一致, 一重配置 CPU 就取不到指令、卡死在启动。 */
    opt.header.U  = 0xfcf90002U;
    opt.option0.U = 0x00000007U;
    opt.option1.U = 0x00001000U;

    s_ready = (rom_xpi_nor_auto_config(HPM_XPI0, &s_nor, &opt) == status_success);
}

bool app_nv_load(int16_t *table_out, int32_t *offset_mrad_out)
{
    /* 只读: 直接走 XIP 映射, 不调 app_nv_init() —— 那会重配置 XPI NOR,
     * 而上电时 CPU 正在这片 flash 上取指, 重配置时钟/模式会把启动流程
     * 卡死 (症状: 打印完 BUILD 后无输出)。 */

    /* 【只作废 Flash 这一段, 绝不能用 l1c_dc_invalidate_all()】
     * 本芯片上电时 D-cache 是开着的, 程序刚写的变量(日志后端表 s_backends、
     * 编码器扫描结果、LUT...)全都在 cache 里还是"脏行"; 整片作废会把脏行
     * 直接丢掉, 于是它们立刻回到旧值(0) —— 实测症状就是【BUILD 之后所有
     * 日志无声无息地消失】, 而程序其实还在正常跑 (串口只在复位后打印一屏,
     * 扫完编码器就没了)。
     * 正确做法: 只作废要从 Flash 重读的这一段。 */
    if (l1c_dc_is_enabled()) {
        l1c_dc_invalidate(APP_NV_FLASH_ADDR, sizeof(s_buf));
    }
    memcpy(s_buf, (const void *)APP_NV_FLASH_ADDR, sizeof(s_buf));

    if (!nv_header_ok(s_buf)) {
        return false;
    }

    (void)memcpy(table_out, s_buf + APP_NV_TABLE_OFFSET, ENC_LUT_BINS * 2U);
    (void)memcpy(offset_mrad_out, s_buf + 12U, 4U);
    return true;
}

ATTR_RAMFUNC
bool app_nv_save(const int16_t *table, int32_t offset_mrad)
{
    uint32_t   off = APP_NV_FLASH_ADDR - BOARD_FLASH_BASE_ADDRESS;
    uint32_t  *w   = (uint32_t *)s_buf;
    uint32_t   crc;
    hpm_stat_t st;

    app_nv_init();
    if (!s_ready) {
        return false;
    }

    memset(s_buf, 0xFF, sizeof(s_buf));
    w[0] = APP_NV_MAGIC;
    w[1] = APP_NV_VERSION;
    w[2] = (uint32_t)FOC_MOTOR_POLE_PAIRS;
    w[3] = (uint32_t)offset_mrad;
    w[4] = 0U;
    w[5] = 0U;
    (void)memcpy(s_buf + APP_NV_TABLE_OFFSET, table, ENC_LUT_BINS * 2U);

    crc  = nv_crc32_update(0xFFFFFFFFU, s_buf, 24U);
    crc  = nv_crc32_update(crc, s_buf + APP_NV_TABLE_OFFSET, ENC_LUT_BINS * 2U);
    w[6] = crc ^ 0xFFFFFFFFU;

    /* 擦写的是"正在执行代码"的同一片 NOR: 关全局中断, 避免 ISR 在 Flash busy
     * 期间去取指令挂死。驱动本体在 ROM 里跑, 不依赖 XIP。 */
    disable_global_irq(CSR_MSTATUS_MIE_MASK);
    st = rom_xpi_nor_erase(HPM_XPI0, xpi_xfer_channel_auto, &s_nor, off, APP_NV_SECTOR_BYTES);
    if (st == status_success) {
        st = rom_xpi_nor_program(HPM_XPI0, xpi_xfer_channel_auto, &s_nor,
                                 (const uint32_t *)s_buf, off, APP_NV_SECTOR_BYTES);
    }
    enable_global_irq(CSR_MSTATUS_MIE_MASK);

    /* 同上: 只作废写过的这一段的缓存副本, 不要整片作废 */
    if (l1c_dc_is_enabled()) {
        l1c_dc_invalidate(APP_NV_FLASH_ADDR, APP_NV_SECTOR_BYTES);
    }

    return (st == status_success);
}
