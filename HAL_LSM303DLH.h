/**
  * @file    HAL_LSM303DLH.h
  * @author  ART Team IMS-Systems Lab
  * @version V2.2
  * @date    01/11/2011
  * @brief   Hardware Abstraction Layer for LSM303DLH.
  * @details
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
  * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
  * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */


/* Define to prevent recursive inclusion*/
#ifndef __HAL_LSM303DLH_H
#define __HAL_LSM303DLH_H

#ifdef __cplusplus
 extern "C" {
#endif 
  
/* Includes */
#include "stm32f10x.h"

/**
* @addtogroup LSM303DLH
* @{
*/

/**
* @addtogroup  LSM303DLH_I2C_Define
* @{
*/

#define LSM_I2C                  I2C2
#define LSM_I2C_RCC_Periph       RCC_APB1Periph_I2C2
#define LSM_I2C_Port             GPIOB
#define LSM_I2C_SCL_Pin          GPIO_Pin_10
#define LSM_I2C_SDA_Pin          GPIO_Pin_11
#define LSM_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define LSM_I2C_Speed            300000

/**
*@}
*/ /* end of group LSM303DLH_I2C_Define */ 

/**
 * @addtogroup Magnetometer
 * @{
 */

/**
* @addtogroup    Magnetometer_I2C_Define
* @{
*/

#define LSM_M_I2C                 I2C2
#define LSM_M_I2C_RCC_Periph      RCC_APB1Periph_I2C2
#define LSM_M_I2C_Port            GPIOB
#define LSM_M_I2C_SCL_Pin         GPIO_Pin_10
#define LSM_M_I2C_SDA_Pin         GPIO_Pin_11
#define LSM_M_I2C_RCC_Port        RCC_APB2Periph_GPIOB

#define LSM_M_I2C_Speed           300000
/**
*@}
*/ /* end of group Magnetometer_I2C_Define */ 

/**
* @addtogroup Magnetometer_Interrupt_Pin_Define
* @{
*/

//#ifdef  LSM_M_DRDY_ENABLE
#define LSM_M_DRDY_Pin                   GPIO_Pin_4
#define LSM_M_DRDY_Port                  GPIOB
#define LSM_M_DRDY_RCC_Port              RCC_APB2Periph_GPIOB
#define LSM_M_DRDY_Port_Source           GPIO_PortSourceGPIOB
#define LSM_M_DRDY_Pin_Source            GPIO_PinSource4
#define LSM_M_DRDY_EXTI_Line             EXTI_Line4
#define LSM_M_DRDY_Edge                  EXTI_Trigger_Falling
#define LSM_M_DRDY_EXTI_IRQCHANNEL       EXTI4_IRQn
#define LSM_M_DRDY_Preemption_Priority   3
#define LSM_M_DRDY_Sub_Priority          3
//#endif

/**
* @}
*/ /* end of group Magnetometer_Interrupt_Pin_Define */

/**
 * @}
 */ /* end of group Magnetometer */

/**
 * @addtogroup Accelerometer
 * @{
 */

/**
* @addtogroup Accelerometer_Interrupt_Pin_Define
* @{
*/
#ifdef  LSM_A_INT1_ENABLE
#define LSM_A_INT1_Pin           GPIO_Pin_3
#define LSM_A_INT1_Port          GPIOB
#define LSM_A_INT1_RCC_Port      RCC_APB2Periph_GPIOB
#endif

#ifdef LSM_A_INT2_ENABLE
#define LSM_A_INT2_Pin           GPIO_Pin_15
#define LSM_A_INT2_Port          GPIOA
#define LSM_A_INT2_RCC_Port      RCC_APB2Periph_GPIOA
#endif

/**
*@}
*/ /* end of group Accelerometer_Interrupt_Pin_Define */



/**
* @addtogroup  Accelerometer_I2C_Defines
* @{
*/

#define LSM_A_I2C                 I2C2
#define LSM_A_I2C_RCC_Periph      RCC_APB1Periph_I2C2
#define LSM_A_I2C_Port            GPIOB
#define LSM_A_I2C_SCL_Pin         GPIO_Pin_10
#define LSM_A_I2C_SDA_Pin         GPIO_Pin_11
#define LSM_A_I2C_RCC_Port        RCC_APB2Periph_GPIOB
#define LSM_A_I2C_Speed           300000

/**
*@}
*/ /* end of group Accelerometer_I2C_Defines */

/**
 * @} 
 */ /* end of group Accelerometer */

/**
*@}
*/ /* end of group LSM303DLH */



#endif /* __HAL_LSM303DLH_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
