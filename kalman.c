#include "kalman.h"
#include "OSConfig.h"
#include <arm_math.h>
#include <string.h>
#include "axisTrans.h"
#include "uart.h"

ekf_filter ekf_filter_new(u8 state_dim,
			u8 measure_dim,
			const float *Q,
			const float *R,
			getA_function GetA,
			getH_function GetH,
			timeUpdateFunc aFunc,
			EstiMeasureFunc hFunc)
{
	ekf_filter filter;
	unsigned err = 0;
	// nothing to do if no state or measurement !
	if(state_dim == 0 || measure_dim == 0)
        return 0;
    // alloc new structure
    filter = pvPortMalloc(sizeof(ekf_filter_t));
    // returns 0 if allocation fails
    if(filter == 0)
    	return 0;
    // fills the structure
    filter->state_dim = state_dim;
    filter->measure_dim = measure_dim;
    filter->GetA = GetA;
    filter->GetH = GetH;
	filter->aFunc = aFunc;
	filter->hFunc = hFunc;

    filter->x = pvPortMalloc(state_dim * sizeof(float));
    err |= (filter->x == 0);

    filter->y = pvPortMalloc(measure_dim * sizeof(float));
    err |= (filter->y == 0);

    filter->P = pvPortMalloc(state_dim*state_dim * sizeof(float));
    err |= (filter->P == 0);

    filter->A = pvPortMalloc(state_dim*state_dim * sizeof(float));
    err |= (filter->A == 0);

    filter->H = pvPortMalloc(measure_dim*state_dim * sizeof(float));
    err |= (filter->H == 0);

	if(err != 0)
	{
		vPortFree(filter);
		return 0;
	}

    filter->Q = pvPortMalloc(state_dim*state_dim * sizeof(float));
	if(filter->Q == 0)
	{
		vPortFree(filter);
		return 0;
	}
	memcpy(filter->Q,Q,state_dim*state_dim*sizeof(float));

	filter->R = pvPortMalloc(measure_dim*measure_dim*sizeof(float));
	if(filter->R == 0)
	{
		vPortFree(filter);
		return 0;
	}
	memcpy(filter->R,R,measure_dim*measure_dim*sizeof(float));

    // returns the newly allocated structure
    return filter;	
}

void ekf_filter_delete(ekf_filter filter)
{
	vPortFree(filter->x);
	vPortFree(filter->y);
	vPortFree(filter->P);
	vPortFree(filter->A);
	vPortFree(filter->H);
	vPortFree(filter->Q);
	vPortFree(filter->R);

	vPortFree(filter);
}

/*
filter: EKF filter structure
para1~5: option parameter
*/
void EKF_predict(ekf_filter filter,void *para1,void *para2,void *para3,void *para4,void *para5)
{
	arm_matrix_instance_f32 AMat,PMat,QMat,APMat,trAMat,APtrAMat;

	size_t ss_size=filter->state_dim*filter->state_dim*sizeof(float);

	filter->GetA(filter->A,para1,para2,para3);
	filter->aFunc(filter->x,para4,para5);

	AMat.numRows=filter->state_dim;
	AMat.numCols=filter->state_dim;
	AMat.pData = filter->A;

	PMat.numRows=filter->state_dim;
	PMat.numCols=filter->state_dim;
	PMat.pData = filter->P;

	QMat.numRows=filter->state_dim;
	QMat.numCols=filter->state_dim;
	QMat.pData = filter->Q;

	APMat.numRows=filter->state_dim;
	APMat.numCols=filter->state_dim;
	APMat.pData =pvPortMalloc(ss_size);

	trAMat.numRows=filter->state_dim;
	trAMat.numCols=filter->state_dim;
	trAMat.pData =pvPortMalloc(ss_size);

	APtrAMat.numRows=filter->state_dim;
	APtrAMat.numCols=filter->state_dim;
	APtrAMat.pData =pvPortMalloc(ss_size);

	arm_mat_trans_f32(&AMat,&trAMat);
	arm_mat_mult_f32(&AMat,&PMat,&APMat);
	arm_mat_mult_f32(&APMat,&trAMat,&APtrAMat);

	vPortFree(trAMat.pData);
	vPortFree(APMat.pData);

	arm_mat_add_f32(&APtrAMat,&QMat,&PMat);
	
	vPortFree(APtrAMat.pData);
}

void EKF_update(ekf_filter filter,float *measure,void *para1,void *para2, void *para3, void *para4)
{
	u8 i;
	arm_matrix_instance_f32 xMat,hxMat,zMat,z_hxMat
				,PMat,HMat,RMat
				,trHMat,PtrHMat,HPtrHMat
				,HPtrH_RMat,invHPtrH_RMat
				,KMat,KresiMat,newxMat
				,IMat,KHMat,I_KHMat,newPMat;

	size_t s_size=filter->state_dim*sizeof(float);
	size_t m_size=filter->measure_dim*sizeof(float);
	size_t ss_size=filter->state_dim*filter->state_dim*sizeof(float);
	size_t sm_size=filter->state_dim*filter->measure_dim*sizeof(float);
	size_t mm_size=filter->measure_dim*filter->measure_dim*sizeof(float);

	filter->GetH(filter->H,para1,para2);
//	printMat(filter->H,filter->measure_dim,filter->state_dim);

	PMat.numRows=filter->state_dim;
	PMat.numCols=filter->state_dim;
	PMat.pData=filter->P;
//	printMat(PMat.pData,filter->state_dim,filter->state_dim);

	HMat.numRows=filter->measure_dim;
	HMat.numCols=filter->state_dim;
	HMat.pData=filter->H;

	RMat.numRows=filter->measure_dim;
	RMat.numCols=filter->measure_dim;
	RMat.pData=filter->R;


	trHMat.numRows=filter->state_dim;
	trHMat.numCols=filter->measure_dim;
	trHMat.pData=pvPortMalloc(sm_size);

	PtrHMat.numRows=filter->state_dim;
	PtrHMat.numCols=filter->measure_dim;
	PtrHMat.pData=pvPortMalloc(sm_size);

	arm_mat_trans_f32(&HMat,&trHMat);
//	printMat(trHMat.pData,filter->state_dim,filter->measure_dim);

	arm_mat_mult_f32(&PMat,&trHMat,&PtrHMat);

	vPortFree(trHMat.pData);

	HPtrHMat.numRows=filter->measure_dim;
	HPtrHMat.numCols=filter->measure_dim;
	HPtrHMat.pData=pvPortMalloc(mm_size);

	HPtrH_RMat.numRows=filter->measure_dim;
	HPtrH_RMat.numCols=filter->measure_dim;
	HPtrH_RMat.pData=pvPortMalloc(mm_size);

	invHPtrH_RMat.numRows=filter->measure_dim;
	invHPtrH_RMat.numCols=filter->measure_dim;
	invHPtrH_RMat.pData=pvPortMalloc(mm_size);

	arm_mat_mult_f32(&HMat,&PtrHMat,&HPtrHMat);
	arm_mat_add_f32(&HPtrHMat,&RMat,&HPtrH_RMat);
//	printMat(HPtrH_RMat.pData,filter->measure_dim,filter->measure_dim);

	vPortFree(HPtrHMat.pData);

	arm_mat_inverse_f32(&HPtrH_RMat,&invHPtrH_RMat);
//	printMat(invHPtrH_RMat.pData,filter->measure_dim,filter->measure_dim);
	vPortFree(HPtrH_RMat.pData);
	
	KMat.numRows=filter->state_dim;
	KMat.numCols=filter->measure_dim;
	KMat.pData=pvPortMalloc(sm_size);

	arm_mat_mult_f32(&PtrHMat,&invHPtrH_RMat,&KMat);
//	printMat(KMat.pData,filter->state_dim,filter->measure_dim);
	vPortFree(invHPtrH_RMat.pData);
	vPortFree(PtrHMat.pData);

	hxMat.numRows=filter->measure_dim;
	hxMat.numCols=1;
	hxMat.pData=pvPortMalloc(m_size);

	filter->hFunc(hxMat.pData,para3,para4);
//	printMat(pvParameters1,1,filter->measure_dim);
//	printMat(hxMat.pData,1,filter->measure_dim);

	zMat.numRows=filter->measure_dim;
	zMat.numCols=1;
	zMat.pData=pvPortMalloc(m_size);

	memcpy(zMat.pData,measure,m_size);
//	printMat(zMat.pData,filter->measure_dim,1);
	
	z_hxMat.numRows=filter->measure_dim;
	z_hxMat.numCols=1;
	z_hxMat.pData=pvPortMalloc(m_size);
	
	arm_mat_sub_f32(&zMat,&hxMat,&z_hxMat);
	vPortFree(zMat.pData);
	vPortFree(hxMat.pData);

	KresiMat.numRows=filter->state_dim;
	KresiMat.numCols=1;
	KresiMat.pData=pvPortMalloc(s_size);

	arm_mat_mult_f32(&KMat,&z_hxMat,&KresiMat);
	vPortFree(z_hxMat.pData);

	xMat.numRows=filter->state_dim;
	xMat.numCols=1;
	xMat.pData=filter->x;

	newxMat.numRows=filter->state_dim;
	newxMat.numCols=1;
	newxMat.pData=pvPortMalloc(s_size); 
	arm_mat_add_f32(&xMat,&KresiMat,&newxMat);

	memcpy(xMat.pData,newxMat.pData,s_size);
//	printMat(filter->x,filter->state_dim,1);
	vPortFree(KresiMat.pData);
	vPortFree(newxMat.pData);

	IMat.numRows=filter->state_dim;
	IMat.numCols=filter->state_dim;
	IMat.pData=pvPortMalloc(ss_size);
	memset(IMat.pData,0,ss_size);

	for(i=0;i<filter->state_dim;i++)
	{
		IMat.pData[i*(filter->state_dim+1)]=1.0;
	}

	KHMat.numRows=filter->state_dim;
	KHMat.numCols=filter->state_dim;
	KHMat.pData=pvPortMalloc(ss_size);

	arm_mat_mult_f32(&KMat,&HMat,&KHMat);
	vPortFree(KMat.pData);

	I_KHMat.numRows=filter->state_dim;
	I_KHMat.numCols=filter->state_dim;
	I_KHMat.pData=pvPortMalloc(ss_size);

	arm_mat_sub_f32(&IMat,&KHMat,&I_KHMat);
	vPortFree(IMat.pData);
	vPortFree(KHMat.pData);

	newPMat.numRows=filter->state_dim;
	newPMat.numCols=filter->state_dim;
	newPMat.pData=pvPortMalloc(ss_size);

	arm_mat_mult_f32(&I_KHMat,&PMat,&newPMat);
	memcpy(PMat.pData,newPMat.pData,ss_size);
	
	vPortFree(I_KHMat.pData);
	vPortFree(newPMat.pData);
}


