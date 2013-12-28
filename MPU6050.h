#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "stm32f10x.h"

#define MPU6050_ADDRESS_AD0_LOW     ((u8)0xD0) // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    ((u8)0xD2) // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#define MPU6050_RA_XG_OFFS_TC       ((u8)0x00) //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       ((u8)0x01) //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       ((u8)0x02) //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      ((u8)0x03) //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      ((u8)0x04) //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      ((u8)0x05) //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        ((u8)0x06) //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     ((u8)0x07)
#define MPU6050_RA_YA_OFFS_H        ((u8)0x08) //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     ((u8)0x09)
#define MPU6050_RA_ZA_OFFS_H        ((u8)0x0A) //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     ((u8)0x0B)
#define MPU6050_RA_XG_OFFS_USRH     ((u8)0x13) //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     ((u8)0x14)
#define MPU6050_RA_YG_OFFS_USRH     ((u8)0x15) //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     ((u8)0x16)
#define MPU6050_RA_ZG_OFFS_USRH     ((u8)0x17) //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     ((u8)0x18)

//Register address
#define MPU6050_RA_SMPLRT_DIV       ((u8)0x19)
#define MPU6050_RA_CONFIG           ((u8)0x1A)
#define MPU6050_RA_GYRO_CONFIG      ((u8)0x1B)
#define MPU6050_RA_ACCEL_CONFIG     ((u8)0x1C)
#define MPU6050_RA_FF_THR           ((u8)0x1D)
#define MPU6050_RA_FF_DUR           ((u8)0x1E)
#define MPU6050_RA_MOT_THR          ((u8)0x1F)
#define MPU6050_RA_MOT_DUR          ((u8)0x20)
#define MPU6050_RA_ZRMOT_THR        ((u8)0x21)
#define MPU6050_RA_ZRMOT_DUR        ((u8)0x22)
#define MPU6050_RA_FIFO_EN          ((u8)0x23)
#define MPU6050_RA_I2C_MST_CTRL     ((u8)0x24)
#define MPU6050_RA_I2C_SLV0_ADDR    ((u8)0x25)
#define MPU6050_RA_I2C_SLV0_REG     ((u8)0x26)
#define MPU6050_RA_I2C_SLV0_CTRL    ((u8)0x27)
#define MPU6050_RA_I2C_SLV1_ADDR    ((u8)0x28)
#define MPU6050_RA_I2C_SLV1_REG     ((u8)0x29)
#define MPU6050_RA_I2C_SLV1_CTRL    ((u8)0x2A)
#define MPU6050_RA_I2C_SLV2_ADDR    ((u8)0x2B)
#define MPU6050_RA_I2C_SLV2_REG     ((u8)0x2C)
#define MPU6050_RA_I2C_SLV2_CTRL    ((u8)0x2D)
#define MPU6050_RA_I2C_SLV3_ADDR    ((u8)0x2E)
#define MPU6050_RA_I2C_SLV3_REG     ((u8)0x2F)
#define MPU6050_RA_I2C_SLV3_CTRL    ((u8)0x30)
#define MPU6050_RA_I2C_SLV4_ADDR    ((u8)0x31)
#define MPU6050_RA_I2C_SLV4_REG     ((u8)0x32)
#define MPU6050_RA_I2C_SLV4_DO      ((u8)0x33)
#define MPU6050_RA_I2C_SLV4_CTRL    ((u8)0x34)
#define MPU6050_RA_I2C_SLV4_DI      ((u8)0x35)
#define MPU6050_RA_I2C_MST_STATUS   ((u8)0x36)
#define MPU6050_RA_INT_PIN_CFG      ((u8)0x37)
#define MPU6050_RA_INT_ENABLE       ((u8)0x38)
#define MPU6050_RA_DMP_INT_STATUS   ((u8)0x39)
#define MPU6050_RA_INT_STATUS       ((u8)0x3A)
#define MPU6050_RA_ACCEL_XOUT_H     ((u8)0x3B)
#define MPU6050_RA_ACCEL_XOUT_L     ((u8)0x3C)
#define MPU6050_RA_ACCEL_YOUT_H     ((u8)0x3D)
#define MPU6050_RA_ACCEL_YOUT_L     ((u8)0x3E)
#define MPU6050_RA_ACCEL_ZOUT_H     ((u8)0x3F)
#define MPU6050_RA_ACCEL_ZOUT_L     ((u8)0x40)
#define MPU6050_RA_TEMP_OUT_H       ((u8)0x41)
#define MPU6050_RA_TEMP_OUT_L       ((u8)0x42)
#define MPU6050_RA_GYRO_XOUT_H      ((u8)0x43)
#define MPU6050_RA_GYRO_XOUT_L      ((u8)0x44)
#define MPU6050_RA_GYRO_YOUT_H      ((u8)0x45)
#define MPU6050_RA_GYRO_YOUT_L      ((u8)0x46)
#define MPU6050_RA_GYRO_ZOUT_H      ((u8)0x47)
#define MPU6050_RA_GYRO_ZOUT_L      ((u8)0x48)
#define MPU6050_RA_EXT_SENS_DATA_00 ((u8)0x49)
#define MPU6050_RA_EXT_SENS_DATA_01 ((u8)0x4A)
#define MPU6050_RA_EXT_SENS_DATA_02 ((u8)0x4B)
#define MPU6050_RA_EXT_SENS_DATA_03 ((u8)0x4C)
#define MPU6050_RA_EXT_SENS_DATA_04 ((u8)0x4D)
#define MPU6050_RA_EXT_SENS_DATA_05 ((u8)0x4E)
#define MPU6050_RA_EXT_SENS_DATA_06 ((u8)0x4F)
#define MPU6050_RA_EXT_SENS_DATA_07 ((u8)0x50)
#define MPU6050_RA_EXT_SENS_DATA_08 ((u8)0x51)
#define MPU6050_RA_EXT_SENS_DATA_09 ((u8)0x52)
#define MPU6050_RA_EXT_SENS_DATA_10 ((u8)0x53)
#define MPU6050_RA_EXT_SENS_DATA_11 ((u8)0x54)
#define MPU6050_RA_EXT_SENS_DATA_12 ((u8)0x55)
#define MPU6050_RA_EXT_SENS_DATA_13 ((u8)0x56)
#define MPU6050_RA_EXT_SENS_DATA_14 ((u8)0x57)
#define MPU6050_RA_EXT_SENS_DATA_15 ((u8)0x58)
#define MPU6050_RA_EXT_SENS_DATA_16 ((u8)0x59)
#define MPU6050_RA_EXT_SENS_DATA_17 ((u8)0x5A)
#define MPU6050_RA_EXT_SENS_DATA_18 ((u8)0x5B)
#define MPU6050_RA_EXT_SENS_DATA_19 ((u8)0x5C)
#define MPU6050_RA_EXT_SENS_DATA_20 ((u8)0x5D)
#define MPU6050_RA_EXT_SENS_DATA_21 ((u8)0x5E)
#define MPU6050_RA_EXT_SENS_DATA_22 ((u8)0x5F)
#define MPU6050_RA_EXT_SENS_DATA_23 ((u8)0x60)
#define MPU6050_RA_MOT_DETECT_STATUS    ((u8)0x61)
#define MPU6050_RA_I2C_SLV0_DO      ((u8)0x63)
#define MPU6050_RA_I2C_SLV1_DO      ((u8)0x64)
#define MPU6050_RA_I2C_SLV2_DO      ((u8)0x65)
#define MPU6050_RA_I2C_SLV3_DO      ((u8)0x66)
#define MPU6050_RA_I2C_MST_DELAY_CTRL   ((u8)0x67)
#define MPU6050_RA_SIGNAL_PATH_RESET    ((u8)0x68)
#define MPU6050_RA_MOT_DETECT_CTRL      ((u8)0x69)
#define MPU6050_RA_USER_CTRL        ((u8)0x6A)
#define MPU6050_RA_PWR_MGMT_1       ((u8)0x6B)
#define MPU6050_RA_PWR_MGMT_2       ((u8)0x6C)
#define MPU6050_RA_BANK_SEL         ((u8)0x6D)
#define MPU6050_RA_MEM_START_ADDR   ((u8)0x6E)
#define MPU6050_RA_MEM_R_W          ((u8)0x6F)
#define MPU6050_RA_DMP_CFG_1        ((u8)0x70)
#define MPU6050_RA_DMP_CFG_2        ((u8)0x71)
#define MPU6050_RA_FIFO_COUNTH      ((u8)0x72)
#define MPU6050_RA_FIFO_COUNTL      ((u8)0x73)
#define MPU6050_RA_FIFO_R_W         ((u8)0x74)
#define MPU6050_RA_WHO_AM_I         ((u8)0x75)		// x110100x
//
/*
需要配置的传感器
0x19 SMPLRT_DIV
0x1A EXT_SYNC_SET/DLPF
0x1B gyro_selftest/FS
0x1C acc_selftest/FS
0x23 FIFO_Enable disable all
0x37 INT_Pin
0x38 INT_ENABLE
0x6A USER_CTRL
0x6B PWR_MGMT_1
0x6C PWR_MGMT_2
*/

#define MPU6050_SR_200Hz	200
#define MPU6050_SR_400Hz	400
#define MPU6050_SR_600Hz	600
#define MPU6050_SR_800Hz	800
#define MPU6050_SR_1000Hz	1000

#define MPU6050_Ext_Sync_Disable 	((u8)0x00)	//不接收外部中断信号
#define MPU6050_Ext_Sync_TempL0		((u8)0x08)	//外部中断信号锁存在温度寄存器的最低位
#define MPU6050_Ext_Sync_GyrXL0		((u8)0x10)
#define MPU6050_Ext_Sync_GyrYL0		((u8)0x18)
#define MPU6050_Ext_Sync_GyrZL0		((u8)0x20)
#define MPU6050_Ext_Sync_AccXL0		((u8)0x28)
#define MPU6050_Ext_Sync_AccYL0		((u8)0x30)
#define MPU6050_Ext_Sync_AccZL0		((u8)0x38)

#define MPU6050_DLPF_Bandwidth_A260_G256	((u8)0x00)	  //数字低通滤波器会造成延时，不建议使用
#define MPU6050_DLPF_Bandwidth_A184_G188	((u8)0x01)
#define MPU6050_DLPF_Bandwidth_A94_G98		((u8)0x02)
#define MPU6050_DLPF_Bandwidth_A44_G42		((u8)0x03)
#define MPU6050_DLPF_Bandwidth_A21_G20		((u8)0x04)
#define MPU6050_DLPF_Bandwidth_A10_G10		((u8)0x05)
#define MPU6050_DLPF_Bandwidth_A5_G5		((u8)0x06)

#define MPU6050_Gyro_AxisX_Selftest_Enable	((u8)0x80)	 //陀螺仪配置
#define MPU6050_Gyro_AxisX_Selftest_Disable	((u8)0x00)
#define MPU6050_Gyro_AxisY_Selftest_Enable	((u8)0x40)
#define MPU6050_Gyro_AxisY_Selftest_Disable	((u8)0x00)
#define MPU6050_Gyro_AxisZ_Selftest_Enable	((u8)0x20)
#define MPU6050_Gyro_AxisZ_Selftest_Disable	((u8)0x00)

#define MPU6050_Gyro_FS_250			((u8)0x00)
#define MPU6050_Gyro_FS_500			((u8)0x08)
#define MPU6050_Gyro_FS_1000		((u8)0x10)
#define MPU6050_Gyro_FS_2000		((u8)0x18)

#define MPU6050_Acc_AxisX_Selftest_Enable	((u8)0x80)	  //加速度计配置
#define MPU6050_Acc_AxisX_Selftest_Disable	((u8)0x00)	  
#define MPU6050_Acc_AxisY_Selftest_Enable	((u8)0x40)
#define MPU6050_Acc_AxisY_Selftest_Disable	((u8)0x00)
#define MPU6050_Acc_AxisZ_Selftest_Enable	((u8)0x20)
#define MPU6050_Acc_AxisZ_Selftest_Disable	((u8)0x00)

#define MPU6050_Acc_FS_2g			((u8)0x00)
#define MPU6050_Acc_FS_4g			((u8)0x08)
#define MPU6050_Acc_FS_8g			((u8)0x10)
#define MPU6050_Acc_FS_16g			((u8)0x18)

#define MPU6050_Acc_HPF_Disable		((u8)0x00)		 //加速度计高通滤波器
#define MPU6050_Acc_HPF_5Hz			((u8)0x01)
#define MPU6050_Acc_HPF_2_5Hz		((u8)0x02)
#define MPU6050_Acc_HPF_1_25Hz		((u8)0x03)
#define MPU6050_Acc_HPF_0_63Hz		((u8)0x04)
#define MPU6050_Acc_HPF_Hold		((u8)0x07)

#define MPU6050_FIFO_Disable_All	((u8)0x00)

#define MPU6050_Int_Level_High		((u8)0x00)	   //int pin 配置
#define MPU6050_Int_Level_Low		((u8)0x80)
#define MPU6050_Int_Pin_PP			((u8)0x00)
#define MPU6050_Int_Pin_OD			((u8)0x40)
#define MPU6050_Int_Latch_Enable	((u8)0x20)
#define MPU6050_Int_Latch_Disable	((u8)0x00)
#define MPU6050_Int_Clear_StatusRD	((u8)0x00)
#define MPU6050_Int_Clear_AnyRD		((u8)0x10)

#define MPU6050_FSYNC_Int_Level_High	((u8)0x00)
#define MPU6050_FSYNC_Int_Level_Low		((u8)0x08)
#define MPU6050_FSYNC_Enable			((u8)0x04)
#define MPU6050_FSYNC_Disable			((u8)0x00)
#define MPU6050_CLKOUT_Enable			((u8)0x01)
#define MPU6050_CLKOUT_Disable			((u8)0x00)

#define MPU6050_DRDY_Int_Enable			((u8)0x01)
#define MPU6050_DRDY_Int_Disable		((u8)0x00)

#define MPU6050_FIFO_Enable			((u8)0x40)
#define MPU6050_FIFO_Disable		((u8)0x00)
#define MPU6050_I2C_MST_Enable		((u8)0x20)
#define MPU6050_I2C_MST_Disable		((u8)0x00)
#define MPU6050_I2C_Choose			((u8)0x00)

#define MPU6050_Reset_Command		((u8)0x80)
#define MPU6050_Sleep_Command		((u8)0x40)
#define MPU6050_Temp_Sensor_Disable	((u8)0x08)
#define MPU6050_CLKSEL_Internal_8M	((u8)0x00)
#define MPU6050_CLKSEL_Pll_GyrX		((u8)0x01)
#define MPU6050_CLKSEL_Pll_GyrY		((u8)0x02)
#define MPU6050_CLKSEL_Pll_GyrZ		((u8)0x03)

typedef struct
{
	u16 SamRate;			  //采样率
	u8 LPFBandwidth;	  //低通滤波器带宽
	u8 GyroSelfTestStatus;		//自测
	u8 GyroFS;					//陀螺仪量程
	u8 AccSelfTestStatus;		//加速度计自测
	u8 AccFS;					//加速度计量程
	u8 AccHPF;					//加速度计高通滤波器
	u8 FIFOConfig;				//FIFO配置
	u8 IntPinLevel;				//中断管脚上有效电平
	u8 IntPinType;				//中断管脚的模式推挽or开漏
	u8 IntLatchMode;			//中断状态锁存方式
	u8 IntClearMode;			//中断状态清楚方式
	u8 FSYNCIntLevel;			//外部中断喜好有效电平
	u8 FSYNCIntMode;			//外部中断开启状态
	u8 CLKOUTMode;				//输出时钟开启状态
	u8 DRDYIntMode;				//数据准备好管脚开启状态
	u8 FIFOMode;				//FIFO使能
	u8 I2CMasterStatus;			//I2C主设备使能状态
	u8 CLKSel;					//时钟源选择

}MPU6050_ConfigTypeDef;

void MPU6050_I2C_Init(void);	//初始化I2C总线
u8 MPU6050_I2C_ByteWrite(u8 slAddr, u8* pBuffer, u8 WriteAddr);
u8 MPU6050_I2C_BufferRead(u8 slAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
u8 MPU6050_Init(MPU6050_ConfigTypeDef *MPU6050_Config_Struct);

u8 MPU6050_Read_PID(u8* pid);	//读设备ID
u8 MPU6050_Read_GyroRaw(s16* out);   //读陀螺仪原始数据
u8 MPU6050_Read_GyroRate(float* out);	 //读陀螺仪角速率
u8 MPU6050_Read_AccRaw(s16* out);	   //读加速度计原始数据
u8 MPU6050_Read_Acc(float* out);	 //读加速度
u8 MPU6050_Read_Raw(s16* gyr,s16* acc);		//读取所有数据
u8 MPU6050_Read_GyroAcc(float* gyr,float* acc);
u8 MPU6050_Read_TempRaw(s16* temp); //读温度原始数据
u8 MPU6050_read_Temp(s16* temp);//读温度值，精确到0.1℃

#endif /* _MPU6050_H_ */

