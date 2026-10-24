/**
  * @file    transform.c
  * @brief   坐标变换实现(Clarke / Park / 逆变换)
  */
#include "transform.h"
#include <math.h>

/* ==========================================================================
 * 三角函数查表（512 点 + 线性插值）
 * --------------------------------------------------------------------------
 * 动机见 transform.h。关键数字(cal21 [PROF] 实测):
 *   静止 27.26us / 运动 35.99us(周期 50us)
 * 而每拍最多 12 次库 sinf/cosf ⇒ 仅这一项就吃掉 10~14us。
 * 换成查表后每次约 20 周期, 整项降到 ~0.5us。
 *
 * 负数/大角度的处理: 用 (int32_t) 向零截断, 小数部分 f 与 i 同号,
 * 直接线性插值就能得到正确结果(负角度向下插), 不需要 fmodf。
 * ========================================================================== */
#define TBL_N      512
#define TBL_MASK   (TBL_N - 1)
#define TBL_K      ((float)TBL_N / 6.283185307179586f)   /* N/(2π) */

static float   s_tbl[TBL_N + 1];   /* 最后多一格给线性插值 */
static uint8_t s_tbl_ok = 0U;

void FOC_Math_Init(void)
{
    int32_t i;
    for (i = 0; i < TBL_N; i++) {
        s_tbl[i] = sinf(6.283185307179586f * (float)i / (float)TBL_N);
    }
    s_tbl[TBL_N] = s_tbl[0];
    s_tbl_ok = 1U;
}

void FOC_SinCos(float th, float *s, float *c)
{
    float x, f, a0, a1, b0, b1;
    int32_t i, j;

    if (s_tbl_ok == 0U) { FOC_Math_Init(); }   /* 兜底: 忘了初始化也不会算错 */

    x = th * TBL_K;
    if (x >  1.0e6f) { x = 0.0f; }             /* 防 NaN/Inf 把 int 转换搞爆 */
    if (x < -1.0e6f) { x = 0.0f; }
    i = (int32_t)x;                            /* 向零截断 */
    f = x - (float)i;                          /* 小数(与 i 同号) */
    i &= TBL_MASK;
    j = (i + (TBL_N / 4)) & TBL_MASK;          /* cos = sin(θ+90°) */

    a0 = s_tbl[i]; a1 = s_tbl[i + 1];
    if (s != 0) { *s = a0 + f * (a1 - a0); }
    if (c != 0) {
        b0 = s_tbl[j]; b1 = s_tbl[j + 1];
        *c = b0 + f * (b1 - b0);
    }
}

float FOC_Sin(float th)
{
    float s;
    FOC_SinCos(th, &s, 0);
    return s;
}

float FOC_Cos(float th)
{
    float c;
    FOC_SinCos(th, 0, &c);
    return c;
}

void Clarke_Transform(float iu, float iv, float iw, float *ialpha, float *ibeta)
{
    (void)iw;  // ????????????
    *ialpha = iu;
    *ibeta = (iu + 2.0f * iv) * 0.57735027f;  // 1/v3
}

void Park_Transform(float ialpha, float ibeta, float theta,
                    float *id, float *iq)
{
    /* ★ 查表代替 sinf/cosf: 每拍 2 次 → 约 40 周期 (原 ~400 周期) */
    float s, c;
    FOC_SinCos(theta, &s, &c);
    *id = ialpha * c + ibeta * s;
    *iq = -ialpha * s + ibeta * c;
}

void InvPark_Transform(float vd, float vq, float theta,
                       float *valpha, float *vbeta)
{
    float s, c;
    FOC_SinCos(theta, &s, &c);
    *valpha = vd * c - vq * s;
    *vbeta = vd * s + vq * c;
}

void InvClarke_Transform(float valpha, float vbeta, float *va, float *vb, float *vc)
{
    const float k = 0.8660254f;  // v3/2
    *va = valpha;
    *vb = -0.5f * valpha + k * vbeta;
    *vc = -0.5f * valpha - k * vbeta;
}

