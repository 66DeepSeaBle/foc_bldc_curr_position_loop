/**
  * @file    transform.c
  * @brief   坐标变换实现(Clarke / Park / 逆变换)
  */
#include "transform.h"
#include <math.h>

void Clarke_Transform(float iu, float iv, float iw, float *ialpha, float *ibeta)
{
    (void)iw;  // ????????????
    *ialpha = iu;
    *ibeta = (iu + 2.0f * iv) * 0.57735027f;  // 1/v3
}

void Park_Transform(float ialpha, float ibeta, float theta,
                    float *id, float *iq)
{
    float s = sinf(theta);
    float c = cosf(theta);
    *id = ialpha * c + ibeta * s;
    *iq = -ialpha * s + ibeta * c;
}

void InvPark_Transform(float vd, float vq, float theta,
                       float *valpha, float *vbeta)
{
    float s = sinf(theta);
    float c = cosf(theta);
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

