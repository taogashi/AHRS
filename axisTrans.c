#include "axisTrans.h"
#include <arm_math.h>

void Quat2Angle(float *angle,float *quat)
{
	float c32;
	float c33;
	float c31;
	float c11;
	float c21;

	c32=2*(quat[2]*quat[3]+quat[0]*quat[1]);
	c33=quat[0]*quat[0]-quat[1]*quat[1]-quat[2]*quat[2]+quat[3]*quat[3];
	c31=2*(quat[1]*quat[3]-quat[0]*quat[2]);
	c11=quat[0]*quat[0]+quat[1]*quat[1]-quat[2]*quat[2]-quat[3]*quat[3];
	c21=2*(quat[1]*quat[2]+quat[0]*quat[3]);
	
	angle[2]=atan2(c21,c11);  //-PI~PI
	angle[1]=asin(-c31);	 //-PI/2~PI/2
	angle[0]=atan2(c32,c33);	//-PI~PI

	if(angle[2]<0) angle[2]+=2*PI;
}

void Angle2Quat(float *quat,float *angle)
{
	float cosFai=arm_cos_f32(angle[0]/2);
	float sinFai=arm_sin_f32(angle[0]/2);
	float cosTheta=arm_cos_f32(angle[1]/2);
	float sinTheta=arm_sin_f32(angle[1]/2);
	float cosPsi=arm_cos_f32(angle[2]/2);
	float sinPsi=arm_sin_f32(angle[2]/2);

	quat[0]=cosFai*cosTheta*cosPsi+sinFai*sinTheta*sinPsi;
	quat[1]=sinFai*cosTheta*cosPsi-cosFai*sinTheta*sinPsi;
	quat[2]=cosFai*sinTheta*cosPsi+sinFai*cosTheta*sinPsi;
	quat[3]=cosFai*cosTheta*sinPsi-sinFai*sinTheta*cosPsi;
	QuatNormalize(quat);	
}

void Quat2dcm(float *DCM,float *quat)
{
	float a2=quat[0]*quat[0];
	float b2=quat[1]*quat[1];
	float c2=quat[2]*quat[2];
	float d2=quat[3]*quat[3];
	float ab=quat[0]*quat[1];
	float ac=quat[0]*quat[2];
	float ad=quat[0]*quat[3];
	float bc=quat[1]*quat[2];
	float bd=quat[1]*quat[3];
	float cd=quat[2]*quat[3];

	DCM[0]=a2+b2-c2-d2;	DCM[1]=2*(bc-ad);	DCM[2]=2*(bd+ac);
	DCM[3]=2*(bc+ad);	DCM[4]=a2-b2+c2-d2;	DCM[5]=2*(cd-ab);
	DCM[6]=2*(bd-ac);	DCM[7]=2*(cd+ab);	DCM[8]=a2-b2-c2+d2;
}

void QuatNormalize(float *quat)//ËÄÔªÊý¹éÒ»»¯
{
	float abs=0;
	unsigned char i;
	for(i=0;i<4;i++)
	{
		abs+=quat[i]*quat[i];
	}

	for(i=0;i<4;i++)
	{
		quat[i]/=abs;
	}	
}

void Angle2dcm(float *DCM,float *angle)
{
	float tempQuat[4];
	Angle2Quat(tempQuat,angle);
	Quat2dcm(DCM,tempQuat);
}

void MeasureAngle(float *acc,s16 *mag,float *angle,float *refangle,u8 use_ref)
{
	float totalAcc;
	float Hx,Hy; 
		 
	if(use_ref==0)
	{
		arm_sqrt_f32(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2],&totalAcc);
		angle[1]=asin(acc[0]/totalAcc);	//¸©Ñö½Ç   
		angle[0]=atan2(-acc[1],-acc[2]);		//¹ö×ª½Ç	
		Hx=mag[0]*arm_cos_f32(angle[1])+mag[1]*arm_sin_f32(angle[1])+mag[2]*arm_cos_f32(angle[0])*arm_sin_f32(angle[1]);
		Hy=mag[1]*arm_cos_f32(angle[0])-mag[2]*arm_sin_f32(angle[0]);
	}
	else
	{
		Hx=mag[0]*arm_cos_f32(refangle[1])+mag[1]*arm_sin_f32(refangle[1])+mag[2]*arm_cos_f32(refangle[0])*arm_sin_f32(refangle[1]);
		Hy=mag[1]*arm_cos_f32(refangle[0])-mag[2]*arm_sin_f32(refangle[0]);
		arm_sqrt_f32(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2],&totalAcc);
		angle[1]=asin(acc[0]/totalAcc);	//¸©Ñö½Ç   
		angle[0]=atan2(-acc[1],-acc[2]);		//¹ö×ª½Ç
	}

	if(fabs(Hy)<2)
	{
		if(Hx>0) angle[2]=0;
		else angle[2]=PI;
	}
	else
	{
		angle[2]=atan(Hx/Hy);
		if(Hy<0) angle[2]+=PI/2;
		else
		{
			angle[2]+=PI*3/2;
		}
	}

	angle[2]=angle[2]-0.11461;
	if(angle[2]<0.0) angle[2] += 2*PI;			//0~2*Pi
}

void EularAngleRestrict(float *angle)
{
	if(angle[0]<-PI) angle[0]+=2*PI;
	if(angle[0]>PI)	angle[0]-=2*PI;

	//restrict to -PI~PI
	if(angle[1]<-PI) angle[1]+=2*PI;
	if(angle[1]>PI)	angle[1]-=2*PI;

	//restrict to 0~2PI
	if(angle[2]<0) angle[2]+=2*PI;
	if(angle[2]>2*PI)	angle[2]-=2*PI;	
}

