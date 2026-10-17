/**
  * @file    transform.h
  * @brief   Clarke/Park/?Park?? ??
  */
#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "foc_types.h"

/* ---- Clarke??: Iu,Iv,Iw -> Ialpha,Ibeta ---- */
void Clarke_Transform(float iu, float iv, float iw, float *ialpha, float *ibeta);

/* ---- Park??: Ialpha,Ibeta,theta -> Id,Iq ---- */
void Park_Transform(float ialpha, float ibeta, float theta,
                    float *id, float *iq);

/* ---- ?Park??: Vd,Vq,theta -> Valpha,Vbeta ---- */
void InvPark_Transform(float vd, float vq, float theta,
                       float *valpha, float *vbeta);

/* ---- ????: Valpha,Vbeta -> Va,Vb,Vc ---- */
void InvClarke_Transform(float valpha, float vbeta, float *va, float *vb, float *vc);

#endif

