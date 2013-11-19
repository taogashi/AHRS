#ifndef _HAL_HMC5883_H_
#define _HAL_HMC5883_H_

#define HMC5883_I2C                  I2C2
#define HMC5883_I2C_RCC_Periph       RCC_APB1Periph_I2C2
#define HMC5883_I2C_Port             GPIOB
#define HMC5883_I2C_SCL_Pin          GPIO_Pin_10
#define HMC5883_I2C_SDA_Pin          GPIO_Pin_11
#define HMC5883_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define HMC5883_I2C_Speed            400000

#define HMC5883_DRDY_Pin                   GPIO_Pin_13
#define HMC5883_DRDY_Port                  GPIOC
#define HMC5883_DRDY_RCC_Port              RCC_APB2Periph_GPIOC
#define HMC5883_DRDY_Port_Source           GPIO_PortSourceGPIOC
#define HMC5883_DRDY_Pin_Source            GPIO_PinSource13
#define HMC5883_DRDY_EXTI_Line             EXTI_Line13
#define HMC5883_DRDY_Edge                  EXTI_Trigger_Falling
#define HMC5883_DRDY_EXTI_IRQCHANNEL       EXTI15_10_IRQHandler
#define HMC5883_DRDY_Preemption_Priority   3	  //´ý¶¨
#define HMC5883_DRDY_Sub_Priority          3	  //´ý¶¨

#endif
