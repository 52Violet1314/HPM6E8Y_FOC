/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FOC: 磁场定向控制实现
 */

#include "foc.h"

/* --------------------------------------------------------------------------
 * 通用 PID 已抽到 pid.c / pid.h (电流环、速度环、位置环共用同一实现),
 * 这里不再自带一份 PI —— 原来那份的积分限幅被硬编成 ±out_limit、且
 * "饱和就完全不积分", 与其它两环的规则不一致。
 * -------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------
 * FOC
 * -------------------------------------------------------------------------- */
void foc_init(foc_t *foc)
{
    if (foc == NULL) {
        return;
    }

    foc_math_init();    /* 初始化 sin/cos 查表 (FOC_USE_HPM_MATH=1 时有效) */

    foc->id_ref = 0.0f;
    foc->iq_ref = 0.0f;
    foc->id = 0.0f;
    foc->iq = 0.0f;
    foc->ud = 0.0f;
    foc->uq = 0.0f;
    foc->duty_u = 0.5f;
    foc->duty_v = 0.5f;
    foc->duty_w = 0.5f;
    foc->elec_angle = 0.0f;
    foc->vdc = 0.0f;
    foc->ts = 1.0f / (float)FOC_CURRENT_LOOP_FREQ_HZ;
    foc->v_limit = 0.0f;
    foc->enable = false;

    /* 电流环 PID: 用 pid.c 的通用实现 (语义与 F405_M3508_FOC 的 PID.c 一致)。
     * 这里给的输出/积分限幅只是"额定母线"下的初值, 真正每拍由实测 vdc 算出的
     * vmax 覆盖(见 foc_run 里)。 */
    PID_InitSym(&foc->id_pi, FOC_DEFAULT_CURRENT_KP, FOC_DEFAULT_CURRENT_KI,
                 foc->ts, FOC_VOLTAGE_LIMIT_V, FOC_VOLTAGE_LIMIT_V);
    PID_InitSym(&foc->iq_pi, FOC_DEFAULT_CURRENT_KP, FOC_DEFAULT_CURRENT_KI,
                 foc->ts, FOC_VOLTAGE_LIMIT_V, FOC_VOLTAGE_LIMIT_V);
}

void foc_reset(foc_t *foc)
{
    if (foc == NULL) {
        return;
    }
    PID_Reset(&foc->id_pi);
    PID_Reset(&foc->iq_pi);
    foc->ud = 0.0f;
    foc->uq = 0.0f;
}

void foc_set_current_ref(foc_t *foc, float id_ref, float iq_ref)
{
    if (foc == NULL) {
        return;
    }
    foc->id_ref = foc_clampf(id_ref, -FOC_CURRENT_LIMIT_A, FOC_CURRENT_LIMIT_A);
    foc->iq_ref = foc_clampf(iq_ref, -FOC_CURRENT_LIMIT_A, FOC_CURRENT_LIMIT_A);
}

void foc_set_gains(foc_t *foc, float kp, float ki)
{
    if (foc == NULL) {
        return;
    }
    PID_SetGains(&foc->id_pi, kp, ki);
    PID_SetGains(&foc->iq_pi, kp, ki);
}

void foc_enable(foc_t *foc, bool enable)
{
    if (foc == NULL) {
        return;
    }
    if (!enable) {
        foc_reset(foc);
        foc->duty_u = 0.5f;
        foc->duty_v = 0.5f;
        foc->duty_w = 0.5f;
    }
    foc->enable = enable;
}

void foc_update(foc_t *foc, float ia, float ib, float ic, float elec_angle, float vdc)
{
    float ialpha, ibeta;
    float sin_t, cos_t;
    float ualpha, ubeta;
    float vmax;

    if (foc == NULL) {
        return;
    }

    foc->elec_angle = elec_angle;
    foc->vdc = vdc;

    /* SVPWM 线性区最大相电压 = vdc/√3。
     *
     * 【为什么只取 85%, 而不是 95%】这 15% 不是给电压留的, 是给【电流采样】留的。
     *   低边采样只能在"三相下桥同时导通"的窗口里做, 该窗口以 PWM 计数器回绕点为
     *   中心, 半宽 = reload/2×(1-0.98×duty_max)。占空比越极端窗口越窄:
     *       向量=0.85×线性极限 -> duty 约 0.075~0.925 -> 窗口半宽约 465 counts = 2.3us ✓
     *       向量=0.95×线性极限 -> duty 约 0.03 ~0.97  -> 窗口半宽约 190 counts = 0.95us
     *   后者再扣掉死区, 采样点几乎贴在开关沿上 —— 实测就是这样采到过 10A 的假值
     *   (12V+3.1Ω 下即使相间短路也只有 1.9A, 物理上不可能), 把整个运行打断。
     *   标准做法就是留"调制余量", 代价只是最高电压少 10%。 */
    vmax = vdc * FOC_ONE_BY_SQRT3 * 0.85f;
    foc->v_limit = vmax;

    /* 每拍按实测母线刷新限幅:
     *   - 输出限幅 = ±vmax (单轴), 下方还会做矢量限幅;
     *   - 积分限幅单独给, 用比例 FOC_CURRENT_INT_LIMIT_RATIO 收窄 (见 foc_config.h);
     *   - dt 同步: 控制器 dt 若被改过(foc_controller_set_dt), 积分步长要跟上。 */
    {
        float ilim = vmax * FOC_CURRENT_INT_LIMIT_RATIO;

        foc->id_pi.out_min = -vmax;
        foc->id_pi.out_max =  vmax;
        foc->iq_pi.out_min = -vmax;
        foc->iq_pi.out_max =  vmax;
        foc->id_pi.int_min = -ilim;
        foc->id_pi.int_max =  ilim;
        foc->iq_pi.int_min = -ilim;
        foc->iq_pi.int_max =  ilim;
        foc->id_pi.dt = foc->ts;
        foc->iq_pi.dt = foc->ts;
    }

    if (!foc->enable || vdc < 1.0f) {
        foc->duty_u = 0.5f;
        foc->duty_v = 0.5f;
        foc->duty_w = 0.5f;
        return;
    }

    /* 1) 相电流一阶低通 (FOC_CURRENT_LPF_ALPHA=1.0 时等同于直通)
     *    目的: 削掉开关瞬间耦合到采样上的尖峰(共模 dv/dt 经输入电容进差分端),
     *    以及 ADC 的 ±1LSB 抖动。截止频率按 20kHz 采样算:
     *        τ  = ts/α,   fc = α/(2π·ts)
     *        α=0.5 -> fc≈1.6kHz (群延迟 ≈ 1/(2πfc) ≈ 100us)
     *        α=0.3 -> fc≈1.0kHz
     *    对这种"数据本来就有毛刺"的场合, 低通比继续降 P 更有效 —— 降 P 只是让
     *    环路对噪声不敏感, 噪声本身还在; 低通是直接从源头上削。
     *    【代价】相位滞后: 在 100Hz 电流环(BW 628rad/s)下, 1.6kHz 低通的附加
     *    相位滞后 < 10°, 可以接受; 若把带宽提回 300Hz 要重新评估。 */
    if (FOC_CURRENT_LPF_ALPHA < 0.999f) {
        foc->ia_f += FOC_CURRENT_LPF_ALPHA * (ia - foc->ia_f);
        foc->ib_f += FOC_CURRENT_LPF_ALPHA * (ib - foc->ib_f);
        foc->ic_f += FOC_CURRENT_LPF_ALPHA * (ic - foc->ic_f);
        ia = foc->ia_f;
        ib = foc->ib_f;
        ic = foc->ic_f;
    } else {
        /* 直通时也要跟住状态, 免得切换系数时出现阶跃 */
        foc->ia_f = ia;
        foc->ib_f = ib;
        foc->ic_f = ic;
    }

    /* 2) Clarke + Park */
    foc_clarke(ia, ib, ic, &ialpha, &ibeta);
    foc_sin_cos(elec_angle, &sin_t, &cos_t);
    foc_park(ialpha, ibeta, sin_t, cos_t, &foc->id, &foc->iq);

    /* 2) 电流环 PID (通用 PID_Step, dt 已在上面同步) */
    foc->ud = PID_Step(&foc->id_pi, foc->id_ref - foc->id);
    foc->uq = PID_Step(&foc->iq_pi, foc->iq_ref - foc->iq);

    /* 3) 电压矢量限幅 (保证在 SVPWM 线性区) + 【积分器矢量限幅】抗饱和
     *
     * 【为什么必须在这里退积分】PID_Step() 里的饱和判定看的是【单轴】输出限幅
     * (±vmax), 而真正的饱和发生在下面这个【矢量】限幅: 两轴各自都没超、合成
     * 幅值超了。旧实现没有这一步, 于是 PID_Step 永远认为"没饱和", 积分器一路
     * 涨到 int_max 被钉死, 输出恒在 vmax 那个圆上。
     *
     * 【2026-10-18 加强: 光按比例退回不够, 还要按【矢量】压住积分器】
     * 实测(电流环隔离实验: 只给 d 轴 0.3A, 外环全关): idpp≈2100mA、iqpp≈1500mA
     * 在 0.3A 给定下剧烈自激, 而 sqrt(ud²+uq²) 恒等于 5960mV(=vmax) 还在转 ——
     * 这就是积分器钉在限幅圆上的极限环。
     * 根因: int_min/int_max 是【按轴】限的(各 ±ilim), 两轴同时到限时合成还有
     * √2·ilim, 于是 PI 的积分项永远能把矢量顶满, Kp 项一点余量都没有, 积分器
     * 只能等误差反号才退 —— 而它退的过程中又把电流推过头, 形成 50Hz 极限环。
     * 这里改成按【合成矢量】压: 让 |(id_int, iq_int)| ≤ vmax×RATIO,
     * Kp 项始终留有余量, 积分器能自己退出来。 */
    {
        float mag = sqrtf(foc->ud * foc->ud + foc->uq * foc->uq);

        if (mag > vmax && mag > 1e-6f) {
            float scale = vmax / mag;

            foc->ud *= scale;
            foc->uq *= scale;
            /* 比例退回(back-calculation): 让积分器跟着输出一起退出饱和 */
            foc->id_pi.integrator *= scale;
            foc->iq_pi.integrator *= scale;
        }

        /* 积分器的【矢量】限幅 (关键补丁, 见上) */
        {
            float ilim = vmax * FOC_CURRENT_INT_LIMIT_RATIO;
            float imag = sqrtf(foc->id_pi.integrator * foc->id_pi.integrator
                             + foc->iq_pi.integrator * foc->iq_pi.integrator);

            if (imag > ilim && imag > 1e-6f) {
                float k = ilim / imag;

                foc->id_pi.integrator *= k;
                foc->iq_pi.integrator *= k;
            }
        }
    }

    /* 4) 反 Park + SVPWM */
    foc_inv_park(foc->ud, foc->uq, sin_t, cos_t, &ualpha, &ubeta);
    foc_svpwm(ualpha, ubeta, vdc, &foc->duty_u, &foc->duty_v, &foc->duty_w);
}

void foc_get_duty(const foc_t *foc, float *du, float *dv, float *dw)
{
    if (foc == NULL) {
        return;
    }
    if (du != NULL) {
        *du = foc->duty_u;
    }
    if (dv != NULL) {
        *dv = foc->duty_v;
    }
    if (dw != NULL) {
        *dw = foc->duty_w;
    }
}

void foc_open_loop(float ud, float uq, float elec_angle, float vdc,
                   float *du, float *dv, float *dw)
{
    float sin_t, cos_t;
    float ualpha, ubeta;

    foc_sin_cos(elec_angle, &sin_t, &cos_t);
    foc_inv_park(ud, uq, sin_t, cos_t, &ualpha, &ubeta);
    foc_svpwm(ualpha, ubeta, vdc, du, dv, dw);
}

/* --------------------------------------------------------------------------
 * 速度估算
 * -------------------------------------------------------------------------- */
void foc_speed_est_init(foc_speed_est_t *est, float ts, float lpf_alpha)
{
    if (est == NULL) {
        return;
    }
    est->prev_angle = 0.0f;
    est->speed = 0.0f;
    est->ts = (ts > 0.0f) ? ts : 1e-3f;
    est->lpf_alpha = foc_clampf(lpf_alpha, 0.001f, 1.0f);
    est->started = false;
}

float foc_speed_est_update(foc_speed_est_t *est, float mech_angle)
{
    float delta;
    float raw;

    if (est == NULL) {
        return 0.0f;
    }

    if (!est->started) {
        est->prev_angle = mech_angle;
        est->started = true;
        return 0.0f;
    }

    /* 角度差分, 做 ±π 过零处理 */
    delta = foc_normalize_angle_pm_pi(mech_angle - est->prev_angle);
    est->prev_angle = mech_angle;

    raw = delta / est->ts;
    est->speed += est->lpf_alpha * (raw - est->speed);

    return est->speed;
}

void foc_speed_est_set_ts(foc_speed_est_t *est, float ts)
{
    if ((est != NULL) && (ts > 0.0f)) {
        est->ts = ts;
    }
}
