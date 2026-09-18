/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 诊断状态模块实现
 */

#include <stdio.h>
#include <string.h>

#include "app_diag.h"

volatile app_diag_t g_diag;

/* ---- 阶段 ---- */
void diag_set_stage(uint32_t stage)
{
    g_diag.stage = stage;
}

/* ---- 心跳 ---- */
void diag_tick(void)
{
    g_diag.loop_cnt++;
}

/* ---- 采样量 ---- */
void diag_set_vbus_mv(uint32_t vbus_mv)
{
    g_diag.vbus_mv = vbus_mv;
}

void diag_set_currents_ma(int32_t iu, int32_t iv, int32_t iw)
{
    g_diag.iu_ma = iu;
    g_diag.iv_ma = iv;
    g_diag.iw_ma = iw;
}

void diag_set_enc(uint32_t angle, uint32_t frames, uint32_t errors)
{
    g_diag.enc_angle = angle;
    g_diag.enc_frames = frames;
    g_diag.enc_errors = errors;
}

/* ---- 控制输出 ---- */
void diag_set_control(uint32_t trip,
                      uint32_t ol_voltage_mv,
                      uint32_t ol_elec_angle_mrad,
                      uint32_t duty_u_m, uint32_t duty_v_m, uint32_t duty_w_m)
{
    g_diag.trip = trip;
    g_diag.ol_voltage_mv = ol_voltage_mv;
    g_diag.ol_elec_angle_mrad = ol_elec_angle_mrad;
    g_diag.duty_u_m = duty_u_m;
    g_diag.duty_v_m = duty_v_m;
    g_diag.duty_w_m = duty_w_m;
}

/* ---- 遥测格式化 ---- */
int diag_telemetry(char *buf, uint32_t size, float elec_hz, float mech_rpm,
                   uint32_t vbus_mv)
{
    /* 电频率拆成整数 + 1 位小数, 全部用整数打印避免 %f */
    int elec_hz_x10 = (int)(elec_hz * 10.0f);
    int e_int = elec_hz_x10 / 10;
    int e_dec = elec_hz_x10 % 10;

    if (e_dec < 0) {
        e_dec = -e_dec;
    }

    return snprintf(buf, size,
                    "OL F=%d.%dHz V=%umV I=%ld/%ld/%ldmA VBUS=%u E=%u RPM=%ld A=%umrad TRIP=%u CNT=%u ERR=%u\r\n",
                    e_int, e_dec,
                    (unsigned)g_diag.ol_voltage_mv,
                    (long)g_diag.iu_ma, (long)g_diag.iv_ma, (long)g_diag.iw_ma,
                    (unsigned)vbus_mv,
                    (unsigned)g_diag.enc_angle,
                    (long)mech_rpm,
                    (unsigned)g_diag.ol_elec_angle_mrad,
                    (unsigned)g_diag.trip,
                    (unsigned)g_diag.enc_frames,
                    (unsigned)g_diag.enc_errors);
}
