#ifndef _HAL_MPU6050_H_
#define _HAL_MPU6050_H_

#define MPU6050_I2C                  I2C2
#define MPU6050_I2C_RCC_Periph       RCC_APB1Periph_I2C2
#define MPU6050_I2C_Port             GPIOB
#define MPU6050_I2C_SCL_Pin          GPIO_Pin_10
#define MPU6050_I2C_SDA_Pin          GPIO_Pin_11
#define MPU6050_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define MPU6050_I2C_Speed            400000

/**
*@}
*/ /* end of group MPU60503200_I2C_Define */ 

/**
* @addtogroup Gyro_DRDY_Pin_Define
* @{
*/

//#ifdef  MPU6050_DRDY_ENABLE
#define MPU6050_DRDY_Pin                   GPIO_Pin_15
#define MPU6050_DRDY_Port                  GPIOC
#define MPU6050_DRDY_RCC_Port              RCC_APB2Periph_GPIOC
#define MPU6050_DRDY_Port_Source           GPIO_PortSourceGPIOC
#define MPU6050_DRDY_Pin_Source            GPIO_PinSource15
#define MPU6050_DRDY_EXTI_Line             EXTI_Line15
#define MPU6050_DRDY_Edge                  EXTI_Trigger_Falling
#define MPU6050_DRDY_EXTI_IRQCHANNEL       EXTI15_10_IRQn
#define MPU6050_DRDY_Preemption_Priority   3	  //´ý¶¨
#define MPU6050_DRDY_Sub_Priority          3	  //´ý¶¨
#endif
