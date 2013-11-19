#ifndef _KALMAN_H_
#define _KALMAN_H_

#include "stm32f10x.h"

typedef void (*timeUpdateFunc)(float*,void*,void*);
typedef void (*EstiMeasureFunc)(float*,void*,void*);
typedef void (*getA_function)(float*,void*,void*,void*);
typedef void (*getH_function)(float*,void*,void*);

typedef struct
{
	u8 state_dim;
	u8 measure_dim;
	float *x;
	float *y;
	float *P;
	float *A;
	float *H;
	float *Q;
	float *R;

	getA_function GetA;
	getH_function GetH;
	timeUpdateFunc aFunc;
	EstiMeasureFunc hFunc;
}ekf_filter_t;

typedef ekf_filter_t* ekf_filter;

ekf_filter ekf_filter_new(u8 state_dim,
			u8 measure_dim,
			const float *Q,
			const float *R,
			getA_function GetA,
			getH_function GetH,
			timeUpdateFunc aFunc,
			EstiMeasureFunc hFunc);
/*
 * delete a ekf filter
 */
void ekf_filter_delete(ekf_filter filter);
void EKF_predict(ekf_filter filter,void *para1,void *para2,void *para3,void *para4,void *para5);
void EKF_update(ekf_filter filter,float *measure, void *para1,void *para2, void *para3, void *para4);

#endif

