/**
  * @file    LSM303DLH.h
  * @author  ART Team IMS-Systems Lab
  * @version V2.2
  * @date    01/11/2011
  * @brief   Header for LSM303DLH.c file
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



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LSM303DLH_SPI_H
#define __LSM303DLH_SPI_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "HAL_LSM303DLH.h"

/**
* @addtogroup LSM303DLH
* @{
*/

/**
 * @addtogroup Accelerometer
 * @{
 */


/**
  * @defgroup Accelerometer_Configuration_Defines
  *@{
  */

/** 
* \brief accelerometer sensitivity with 2 g full scale [mg/LSB] 
*/
#define LSM_Acc_Sensitivity_2g     1  /*!< accelerometer sensitivity with 2 g full scale [mg/LSB] */
#define LSM_Acc_Sensitivity_4g     0.5  /*!< accelerometer sensitivity with 4 g full scale [mg/LSB] */
#define LSM_Acc_Sensitivity_8g     0.25 /*!< accelerometer sensitivity with 8 g full scale [mg/LSB] */

#define LSM_Acc_Lowpower_NormalMode             ((u8)0x20)
#define LSM_Acc_Lowpower_05                     ((u8)0x40)
#define LSM_Acc_Lowpower_1                      ((u8)0x60)
#define LSM_Acc_Lowpower_2                      ((u8)0x80)
#define LSM_Acc_Lowpower_5                      ((u8)0xA0)
#define LSM_Acc_Lowpower_10                     ((u8)0xC0)

#define LSM_Acc_ODR_50                          ((u8)0x00)
#define LSM_Acc_ODR_100                         ((u8)0x08)
#define LSM_Acc_ODR_400                         ((u8)0x10)
#define LSM_Acc_ODR_1000                        ((u8)0x18)

#define LSM_Acc_XEN                             ((u8)0x01)
#define LSM_Acc_YEN                             ((u8)0x02)
#define LSM_Acc_ZEN                             ((u8)0x04)
#define LSM_Acc_XYZEN                           ((u8)0x07)

#define LSM_Acc_FS_2                            ((u8)0x00)
#define LSM_Acc_FS_4                            ((u8)0x10)
#define LSM_Acc_FS_8                            ((u8)0x30)

#define LSM_Acc_Little_Endian                   ((u8)0x00)
#define LSM_Acc_Big_Endian                      ((u8)0x40)

#define LSM_Acc_BDU_Continuos                   ((u8)0x00)
#define LSM_Acc_BDU_Single                      ((u8)0x80)

#define LSM_Acc_FilterMode_Normal               ((u8)0x00)
#define LSM_AccH_FilterMode_Reference             ((u8)0x20)

#define LSM_Acc_Filter_Enable                   ((u8)0x10)
#define LSM_Acc_Filter_Disable                  ((u8)0x00)

#define LSM_Acc_Filter_HPc8                     ((u8)0x00)
#define LSM_Acc_Filter_HPc16                    ((u8)0x01)
#define LSM_Acc_Filter_HPc32                    ((u8)0x02)
#define LSM_Acc_Filter_HPc64                    ((u8)0x03)

/**
  * @}
  */ /* end of group Accelerometer_Configuration_Defines */

/** @defgroup Accelerometer_Register_Mapping
  * @{
  */


/**
*  \brief Accelerometer I2C Slave Address
*/
#define LSM_A_I2C_ADDRESS         0x32

/**
*  \brief Accelerometer Control Register 1
*  \code
*   Read Write
*   Default value: 0x07
*   7:5 PM2-PM0: Power mode selection.
*              PM2 |  PM1 |  PM0 |   Power Mode Sel | Out Data Rate[Hz]
*             ------------------------------------------------------
*               0  |  0   |  0   |  Power Down      |      --
*               0  |  0   |  1   |  Normal Mode     |      ODR
*               0  |  1   |  0   |  Low Power       |      0.5
*               0  |  1   |  1   |  Low Power       |      1
*               1  |  0   |  0   |  Low Power       |      2
*               1  |  0   |  1   |  Low Power       |      5
*               1  |  1   |  0   |  Low Power       |      10
*   4:3 DR1-DR0: Output Data Rate selection:
*             ODR1 | ODR0 |   ACC.Out Data Rate | LPF cut-off freq[Hz]
*             ------------------------------------------------------
*               0  |  0   |        50 Hz        |        37
*               0  |  1   |       100 Hz        |        74
*               1  |  0   |       400 Hz        |       292
*               1  |  1   |      1000 Hz        |       780
*   2 Zen: Z axis enable. 0 - Z axis disabled  1- Z axis enabled
*   1 Yen: Y axis enable. 0 - Y axis disabled  1- Y axis enabled
*   0 Xen: X axis enable. 0 - X axis disabled  1- X axis enabled
*   \endcode
*/
#define LSM_A_CTRL_REG1_ADDR     0x20

/**
*   \brief Accelerometer Control Register 2
*   \code
*   Read Write
*   Default value: 0x00
*   7 BOOT: Reboot memory content 0 - normal mode  1 - reboot memory content
*   6:5 HPM1-HPM0: High pass filter mode selection:
*             HPM1 | HPM0 |   High pass filter mode
*             -----------------------------------------
*               0  |  0   |   Normal mode (reset reading HP_RESET_FILTER)
*               0  |  1   |   Reference signal for filtering
*               1  |  0   |   Normal mode (reset reading HP_RESET_FILTER)
*   4 Filtered data selection. 0 - internal filter bypassed; 1 - data from internal filter sent to output register
*   3 HPen2: High pass filter enabled for interrupt 2 source. 0 - filter bypassed; 1 - filter enabled
*   2 HPen1: High pass filter enabled for interrupt 1 source. 0 - filter bypassed; 1 - filter enabled
*   1:0 HPCF1-HPCF0 High pass filter cut-off frequency (ft) configuration.
*                 ft= ODR[hz]/6*HPc
*             HPCF1 | HPCF0 |   HPc
*             -----------------------------------------
*               0   |   0   |   8
*               0   |   1   |   16
*               1   |   0   |   32
*               1   |   1   |   64
*
*        HPcoeff |  ft[hz]  |  ft[hz]   |  ft[hz]    | ft[hz]     |
*                | ODR 50Hz | ODR 100Hz | ODR 400Hz  | ODR 1000Hz |
*        ----------------------------------------------------------
*         00     |    1     |    2      |     8      |   20       |
*         01     |    0.5   |    1      |     4      |   10       |
*         10     |    0.25  |    0.5    |     2      |   5        |
*         11     |    0.125 |    0.25   |     1      |   2.5      |
*   \endcode
*/

#define LSM_A_CTRL_REG2_ADDR     0x21

/**
*  \brief Accelerometer Control Register 3 Interrupt Control Register
*  \code
*   Read Write
*  Default value: 0x00
*  7 IHL active: Interrupt active high/low  0 - active high  1 - active low
*  6 PP_OD: push-pull/open-drain 0 - push-pull   1 - open-drain
*  5 LIR2 Latch interrupt request on INT2_SRC register, with INT2_SRC register cleared by reading INT2_SRC itself. 0-interrupt request not latched; 1-interrupt request latched
*  4:3 Int2_cfg1 - Int1_cfg0 Data signal on INT2  pad
*  2 LIR1 Latch interrupt request on INT1_SRC register, with INT1_SRC register cleared by reading INT1_SRC itself. 0-interrupt request not latched; 1-interrupt request latched
*  1:0 Int1_cfg1 - Int1_cfg0 Data signal on INT1  pad
*
*        Int1(2)_cfg1 |   Int1(2)_cfg0 | INT1(2) Pad
*        --------------------------------------------------------
*              0      |       0      | Interrupt 1 (2) source
*              0      |       1      | Interrupt 1 source OR interrupt 2 source
*              1      |       0      |    Data ready
*              1      |       1      |   Boot running
*
*   \endcode
*/
#define LSM_A_CTRL_REG3_ADDR     0x22

/**
*  \brief Accelerometer Control Register 4
*  \code
*   Read Write
*   Default value: 0x00
*   7 BDU Block data update. 0 -continuos update; 1- output registers not updated between MSB and LSB reading
*   6 BLE  Big/little endian data selection. 0 - data LSB @ lower address; 1 - data MSB @ lower address
*   5:4 FS1, FS0 Full-scale selection.(00: ±2 g; 01: ±4 g; 11: ±8 g)
*   3 STsign - Self-test sign. 0: self-test plus; 1: self-test minus
*   2 0
*   1 ST Self-test enable. 0: self-test disabled; 1: self-test enabled)
*   0 SIM SPI serial interface mode selection. 0: 4-wire interface; 1: 3-wire interface
*   \endcode
*/
#define LSM_A_CTRL_REG4_ADDR     0x23

/**
*  \brief Accelometer Control Register
*  \code
*  Read Write
*  Default value: 0x00
*  7:2 - 0
*  1:0 TurnOn1 TurnOn0 : Turn-on mode selection for sleep to wake function :
*                        00 Sleep to wake function is disabled
*                        11 Turned on: The device is in low power mode (ODR is defined in CTRL_REG1)
* \endcode
*/
#define LSM_A_CTRL_REG5_ADDR     0x24

/**
*  \brief Accelerometer HP Filter Reset Register
*  \code
*  Dummy register. Reading at this address zeroes instantaneously the content of the internal
*  high pass-filter. If the high pass filter is enabled all three axes are instantaneously set to 0g.
*  This allows to overcome the settling time of the high pass filter.
*  Read
*  Default value: 0x00
* \endcode
*/
#define LSM_A_HP_FILTER_RESET_REG_ADDR     0x25

/**
*  \brief Accelrometer HP Reference register
*  \code
*  Read Write
*  Default value: 0x00
*  7:0 Ref7 - Ref0  Reference value for high-pass filter.
*  This register sets the acceleration value taken as a reference for the high-pass filter output
*  When filter is turned on (at least one of FDS, HPen2, or HPen1 bit is equal to ‘1’) and HPM
*  bits are set to “01”, filter out is generated taking this value as a reference.
*  \endcode
*/
#define LSM_A_REFERENCE_REG_ADDR     0x26


/**
*  \brief Accelerometer Status Register

*/

#define LSM_A_STATUS_REG_ADDR     0x27


/**
*  \brief Accelerometer X axis Output Data LSB Register
*  \code
*  Read
*  Default value: ( The value is expressed as 16bit two’s complement)
*  7:0 XOUT7-XOUT0: ACC Data LSB (if in Little Endian Mode --> BLE bit in CTRL_REG4 is 0)
*                   ACC Data MSB (if in Big Endian Mode --> BLE bit in CTRL_REG4 is 1)
* \endcode
*/
#define LSM_A_OUT_X_L_ADDR     0x28

/**
*  \brief  Acceleration X-axis Output Data MSB Register
*  \code
*  Default value: ( The value is expressed as 16bit two’s complement)
*  7:0 XOUT15-XOUT8: ACC Data MSB (if in Little Endian Mode --> BLE bit in CTRL_REG1 is 0)
*                    ACC Data LSB (if in Big Endian Mode --> BLE bit in CTRL_REG1 is 1)
* \endcode
*/
#define LSM_A_OUT_X_H_ADDR     0x29

/**
*  \brief Acceleration Y-axis Output Data LSB Register
*  \code
*  Read
*  Default value: ( The value is expressed as 16bit two’s complement)
*  7:0 YOUT7-YOUT0: ACC Data LSB (if in Little Endian Mode --> BLE bit in CTRL_REG4 is 0)
*                   ACC Data MSB (if in Big Endian Mode --> BLE bit in CTRL_REG4 is 1)
* \endcode
*/
#define LSM_A_OUT_Y_L_ADDR     0x2A

/**
*  \brief  Acceleration Y-axis Output Data MSB Register
*  \code
*   Read register
*   Default value:  ( The value is expressed as 16bit two’s complement)
*   7:0 YOUT15-YOUT8: ACC Data MSB (if in Little Endian Mode --> BLE bit in CTRL_REG1 is 0)
*                   ACC Data LSB (if in Big Endian Mode --> BLE bit in CTRL_REG1 is 1)
* \endcode
*/
#define LSM_A_OUT_Y_H_ADDR     0x2B


/**
*  \brief  Acceleration Z-axis Output Data LSB Register
*  \code
*  Read
*  Default value: ( The value is expressed as 16bit two’s complement)
*  7:0 YOUT7-YOUT0: ACC Data LSB (if in Little Endian Mode --> BLE bit in CTRL_REG4 is 0)
*                   ACC Data MSB (if in Big Endian Mode --> BLE bit in CTRL_REG4 is 1)
* \endcode
*/
#define LSM_A_OUT_Z_L_ADDR     0x2C

/**
*  \brief  Acceleration Z-axis Output Data MSB Register
*  \code
*  Default value: ( The value is expressed as 16bit two’s complement)
*  7:0 YOUT15-YOUT8: ACC Data MSB (if in Little Endian Mode --> BLE bit in CTRL_REG1 is 0)
*                   ACC Data LSB (if in Big Endian Mode --> BLE bit in CTRL_REG1 is 1)
*   \endcode
*/
#define LSM_A_OUT_Z_H_ADDR     0x2D

/**
*  \brief Accelerometer Configuration Register for Interrupt 1 source.
*  \code
*  Read
*  Default value: 0x00
*  7 AOI: AND/OR combination of Interrupt events. See table below
*  6 6D:  6 direction detection function enable. See table below
*  5 ZHIE:  Enable interrupt generation on Z high event. 0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold
*  4 ZLIE:  Enable interrupt generation on Z low event. 0: disable interrupt request;  1: enable interrupt request on measured accel. value lower than preset threshold
*  3 YHIE:  Enable interrupt generation on Y high event. 0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold
*  2 YLIE:  Enable interrupt generation on Y low event. 0: disable interrupt request;  1: enable interrupt request on measured accel. value lower than preset threshold
*  1 XHIE:  Enable interrupt generation on X high event. 0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold
*  0 XLIE:  Enable interrupt generation on X low event. 0: disable interrupt request; 1: enable interrupt request on measured accel. value lower than preset threshold
*
*             AOI     |   6D         | Interrupt mode
*        --------------------------------------------------------
*              0      |       0      | OR combination of interrupt events
*              0      |       1      | 6 direction movement recognition
*              1      |       0      | AND combination of interrupt events
*              1      |       1      |  6 direction position recognition
*
*  \endcode
*/

#define LSM_A_INT1_CFG_REG_ADDR 0x30

/**
*  \brief Accelerometer Interrupt 1 source register.
*  \code
*  Read only register.
*  Reading at this address clears INT1_SRC IA bit (and the interrupt signal on INT 1 pin) and
*  allows the refreshment of data in the INT1_SRC register if the latched option was chosen.
*  Read
*  Default value: 0x00
*  7 0
*  6 IA : Interrupt active. 0: no interrupt has been generated; 1: one or more interrupts have been generated
*  5 ZH:  Z high. 0: no interrupt, 1: Z High event has occurred
*  4 ZL:  Z low. 0: no interrupt; 1: Z Low event has occurred
*  3 YH:  Y high. 0: no interrupt, 1: Y High event has occurred
*  2 YL:  Y low. 0: no interrupt; 1: Y Low event has occurred
*  1 YH:  X high. 0: no interrupt, 1: X High event has occurred
*  0 YL:  X low. 0: no interrupt; 1: X Low event has occurred
* \endcode
*/
#define LSM_A_INT1_SCR_REG_ADDR 0x31

/**
*  \brief Accelerometer Interrupt 1 Threshold Register
*  \code
*  Default value: 0x00
*  7 0
*  6 THS6-THS0 Interrupt 1 threshold.
* \endcode
*/
#define LSM_A_INT1_THS_REG_ADDR 0x32

/**
*  \brief Acceleroemter INT1_DURATION Register
*  \code
*  Default value: 0x00
*  7 0
*  6 D6-D0 Duration value. (Duration  steps and maximum values depend on the ODR chosen)
*   \endcode
*/
#define LSM_A_INT1_DURATION_REG_ADDR 0x33


/**
*  \brief INT2_CFG Register Configuration register for Interrupt 2 source.
*  \code
*  Read/write
*  Default value: 0x00
*  7 AOI: AND/OR combination of Interrupt events. See table below
*  6 6D:  6 direction detection function enable. See table below
*  5 ZHIE:  Enable interrupt generation on Z high event. 0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold
*  4 ZLIE:  Enable interrupt generation on Z low event. 0: disable interrupt request;  1: enable interrupt request on measured accel. value lower than preset threshold
*  3 YHIE:  Enable interrupt generation on Y high event. 0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold
*  2 YLIE:  Enable interrupt generation on Y low event. 0: disable interrupt request;  1: enable interrupt request on measured accel. value lower than preset threshold
*  1 XHIE:  Enable interrupt generation on X high event. 0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold
*  0 XLIE:  Enable interrupt generation on X low event. 0: disable interrupt request; 1: enable interrupt request on measured accel. value lower than preset threshold
*
*             AOI     |   6D         | Interrupt mode
*        --------------------------------------------------------
*              0      |       0      | OR combination of interrupt events
*              0      |       1      | 6 direction movement recognition
*              1      |       0      | AND combination of interrupt events
*              1      |       1      |  6 direction position recognition
* \endcode
*/
#define LSM_A_INT2_CFG_REG_ADDR 0x34

/**
* \brief INT2_SCR Register Interrupt 2 source register.
*  \code
*  Read only register.
*  Reading at this address clears INT2_SRC IA bit (and the interrupt signal on INT 2 pin) and
*  allows the refreshment of data in the INT2_SRC register if the latched option was chosen.
*  Read
*  Default value: 0x00
*  7 0
*  6 IA : Interrupt active. 0: no interrupt has been generated; 1: one or more interrupts have been generated
*  5 ZH:  Z high. 0: no interrupt, 1: Z High event has occurred
*  4 ZL:  Z low. 0: no interrupt; 1: Z Low event has occurred
*  3 YH:  Y high. 0: no interrupt, 1: Y High event has occurred
*  2 YL:  Y low. 0: no interrupt; 1: Y Low event has occurred
*  1 YH:  X high. 0: no interrupt, 1: X High event has occurred
*  0 YL:  X low. 0: no interrupt; 1: X Low event has occurred
* \endcode
*/
#define LSM_A_INT2_SCR_REG_ADDR 0x35

/**
*  \brief Accelerometer Interrupt 2 Threshold Register
*  \code
*  Default value: 0x00
*  7 0
*  6 THS6-THS0 Interrupt 2 threshold.
*  \endcode
*/
#define LSM_A_INT2_THS_REG_ADDR 0x36

/**
*  \brief Acceromter INT2_DURATION Register
*  \code
*  Default value: 0x00
*  7 0
*  6 D6-D0 Duration value. (Duration  steps and maximum values depend on the ODR chosen)
*  \endcode
*/
#define LSM_A_INT2_DURATION_REG_ADDR 0x37

/**
  * @}
  */ /* end of group Accelerometer_Register_Mapping */


/**
* @brief Accelerometer Init structure definition
*/

typedef struct
{

  u8 Power_Mode;  /*!<  Low power mode selection (see table 19 datasheet) */
  u8 ODR; /*!< Output Data Rate */
  u8 Axes_Enable;    /*!< Axes Enable */
  u8 FS;     /*!< Full Scale */
  u8 Data_Update;  /*!< Data Update mode : Continuos update or data don`t change until MSB and LSB nex reading */
  u8 Endianess;   /*!< Endianess */
}LSM_Acc_ConfigTypeDef;



/**
  * @brief Accelerometer  Filter Init structure definition
  */


typedef struct
{
  u8 HPF_Enable; /*!< HPF enable*/
  u8 HPF_Mode;  /*!<HPF MODE: Normal mode or Reference signal for filtering*/
  u8 HPF_Reference;     /*!< Reference value for filtering*/
  u8 HPF_Frequency;     /*!< HPF_frequency ft=ODR/6*HPc  HPc=8,16,32,64*/
}LSM_Acc_Filter_ConfigTypeDef;


/**
 * @}
 */ /* end of group Accelerometer */

/**
 * @addtogroup Magnetometer
 * @{
 */

/**
  *@defgroup Magnetometer_Register_Mapping
  *@{
*/

/**
  *\brief Magnetometer I2C Slave Address
*/
#define LSM_M_I2C_ADDRESS         0x3C


/**
*  \brief Magnetometer  Control Register A
*  \code
*  Read Write
*  Default value: 0x10
*  7:5  0   These bits must be cleared for correct operation.
*  4:2 DO2-DO0: Data Output Rate Bits
*              DO2 |  DO1 |  DO0 |   Minimum Data Output Rate (Hz)
*             ------------------------------------------------------
*               0  |  0   |  0   |            0.75
*               0  |  0   |  1   |            1.5
*               0  |  1   |  0   |            3.0
*               0  |  1   |  1   |            7.5
*               1  |  0   |  0   |            15(default)
*               1  |  0   |  1   |            30
*               1  |  1   |  0   |            75
*               1  |  1   |  1   |           Not Used
*  1:0 MS1-MS0: Measurement Configuration Bits
*              MS1 | MS0 |   MODE
*             ------------------------------
*               0  |  0   |  Normal
*               0  |  1   |  Positive Bias
*               1  |  0   |  Negative Bias
*               1  |  1   |  Not Used
* \endcode
*/

#define LSM_M_CRA_REG_ADDR     (u8) 0x00

/**
*  \breif Magnetometer Control Register B
 * \code
*  Read/Write
*  Default value: 0x20
*  7:5 GN2-GN0: Gain Configuration Bits.
*              GN2 |  GN1 |  GN0 |   Mag Input   | Gain       | Output Range
                   |      |      |  Range[Ga]    | [LSB/mGa]  |
*             ------------------------------------------------------
*               0  |  0   |  0   |      NA       |    NA      | 0xF800–0x07FF (-2048:2047)
*               0  |  0   |  1   |      NA       |    NA      |          ""
*               0  |  1   |  0   |  ±0.56Ga      |   970      |          ""
*               0  |  1   |  1   |  ±1.1Ga       |   780      |          ""
*               1  |  0   |  0   |  ±2.4Ga       |   530      |          ""
*               1  |  0   |  1   |  ±3.1Ga       |   460      |          ""
*               1  |  1   |  0   |  ±3.9Ga       |   390      |          ""
*               1  |  1   |  1   |  ±4.0Ga       |   280      |          ""
*  4:0 CRB4-CRB: 0 This bit must be cleared for correct operation.
*  \endcode
*/
#define LSM_M_CRB_REG_ADDR     (u8) 0x01


/**
*  \brief Magnetometer  Mode Register
*  \code
*  Read/Write
*  Default value: 0x02
*  7:2  0   These bits must be cleared for correct operation.
*  1:0 MD1-MD0: Mode Select Bits
*              MS1 | MS0 |   MODE
*             ------------------------------
*               0  |  0   |  Continuous-Conversion Mode.
*               0  |  1   |  Single-Conversion Mode
*               1  |  0   |  Idle Mode. Device is placed in idle mode.
*               1  |  1   |  N.A.
*  \endcode
*/
#define LSM_M_MR_REG_ADDR       (u8) 0x02

/**
*  \brief Magnetometer X-axis Magnetisc Field Data MSB register
* \code
*  The value (MSB+LSB) is expressed as 16bit two’s complement
*  Read
*  Default value:
* \endcode
*/
#define LSM_M_OUT_X_H_ADDR  0x03

/**
*  \brief Magnetometer X-axis Magnetisc Field Data LSB register
*  \code
*  The value (MSB+LSB) is expressed as 16bit two’s complement
*  Read
*  Default value:
*  \endcode
*/
#define LSM_M_OUT_X_L_ADDR  0x04

/**
*  \brief Magnetometer Y-axis Magnetisc Field Data MSB register
* \code
*  The value (MSB+LSB) is expressed as 16bit two’s complement
*  Read
*  Default value:
* \endcode
*/
#define LSM_M_OUT_Y_H_ADDR  0x05

/**
*  \brief Magnetometer Y-axis Magnetisc Field Data LSB register
* \code
*  The value (MSB+LSB) is expressed as 16bit two’s complement
*  Read
*  Default value:
* \endcode
*/
#define LSM_M_OUT_Y_L_ADDR  0x06

/**
*  \brief Magnetometer Z-axis Magnetisc Field Data MSB register
* \code
*  The value (MSB+LSB) is expressed as 16bit two’s complement
*  Read
*  Default value:
* \endcode
*/
#define LSM_M_OUT_Z_H_ADDR  0x07

/**
*  \brief Magnetometer Z-axis Magnetisc Field Data LSB register
* \code
*  The value (MSB+LSB) is expressed as 16bit two’s complement
*  Read
*  Default value:
* \endcode
*/
#define LSM_M_OUT_Z_L_ADDR  0x08

/**
*  \brief Magnetometer Status Register
*  \code
*  Read Only
*  Default value: 0x00
*  7:3 0  These bits must be cleared for correct operation
*  2 REN:  Regulator Enabled Bit -  1 internal voltage regulator enable; 0 - regulator disable
*  1 LOCK: Data output register lock - 1 some but not all for of the six data output registers have been read 0 - the six data output registers are locked(see datasheet)
*  0 RDY:  Ready Bit. 1- data is written to all six data registers  0 -device initiates a write to the data output registers
* \endcode
*/
#define LSM_M_SR_REG_ADDR  0x09

/**
* \brief Magnetometer Identification Register A
* \code
* Read Only
*  Default value: 0x48
*  \endcode
*/
#define LSM_M_IRA_REG_ADDR  0x0A

/**
*  \brief Magnetometer Identification Register B
*  \code
*  Read only
*  Default value: 0x34
*  \endcode
*/
#define LSM_M_IRB_REG_ADDR  0x0B

/**
*  \brief Magnetometer Identification Register C
*  \ code
*  Read Only
*  Default value: 0x33
*  \endcode
*/
#define LSM_M_IRC_REG_ADDR  0x0C

/**
  *@}
  */ /* end of group Magnetometer_Register_Mapping */




/**
  * @brief Magnetometer Init structure definition
  */
typedef struct
{

  u8 M_ODR;     /*!< Magnetometer Output data Rate*/
  u8 Meas_Conf;    /*<Measurement Configuration: Normal, positive bias, or negative bias*/
  u8 Gain;        /*!<Gain Configuration*/
  u8 Mode;  /*!<Mode Conf: Continuos - Single*/
}LSM_Magn_ConfigTypeDef;



/**
* @defgroup Magnetometer_Config_Define
* @{
*/

#define LSM_Magn_Sensitivity_XY_1_3Ga     1055  /*!< magnetometer X Y axes sensitivity for 1.3 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_XY_1_9Ga     795  /*!< magnetometer X Y axes sensitivity for 1.9 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_XY_2_5Ga     635  /*!< magnetometer X Y axes sensitivity for 2.5 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_XY_4Ga       430  /*!< magnetometer X Y axes sensitivity for 4 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_XY_4_7Ga     375  /*!< magnetometer X Y axes sensitivity for 4.7 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_XY_5_6Ga     320  /*!< magnetometer X Y axes sensitivity for 5.6 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_XY_8_1Ga     230  /*!< magnetometer X Y axes sensitivity for 8.1 Ga full scale [LSB/Ga] */

#define LSM_Magn_Sensitivity_Z_1_3Ga     950  /*!< magnetometer Z axes sensitivity for 1.3 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_Z_1_9Ga     710  /*!< magnetometer Z axes sensitivity for 1.9 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_Z_2_5Ga     570  /*!< magnetometer Z axes sensitivity for 2.5 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_Z_4Ga       385  /*!< magnetometer Z axes sensitivity for 4 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_Z_4_7Ga     335  /*!< magnetometer Z axes sensitivity for 4.7 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_Z_5_6Ga     285  /*!< magnetometer Z axes sensitivity for 5.6 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_Z_8_1Ga     205  /*!< magnetometer Z axes sensitivity for 8.1 Ga full scale [LSB/Ga] */


#define LSM_Magn_ODR_0_75              0x00
#define LSM_Magn_ODR_1_5               0x04
#define LSM_Magn_ODR_3                 0x08
#define LSM_Magn_ODR_7_5               0x0C
#define LSM_Magn_ODR_15                0x10
#define LSM_Magn_ODR_30                0x14
#define LSM_Magn_ODR_75                0x18

#define LSM_Magn_MEASCONF_NORMAL       0x00
#define LSM_Magn_MEASCONF_BIAS_POS     0x01
#define LSM_Magn_MEASCONF_BIAS_NEG     0x02

#define LSM_Magn_GAIN_1_3              0x40
#define LSM_Magn_GAIN_1_9              0x60
#define LSM_Magn_GAIN_2_5              0x80
#define LSM_Magn_GAIN_4_0              0xA0
#define LSM_Magn_GAIN_4_7              0xB0
#define LSM_Magn_GAIN_5_6              0xC0
#define LSM_Magn_GAIN_8_1              0xE0

#define LSM_Magn_MODE_CONTINUOS        0x00
#define LSM_Magn_MODE_SINGLE           0x01
#define LSM_Magn_MODE_IDLE             0x02

/**
  * @} 
  */ /* end of group Magnetometer_Register_Mapping */


/**
 * @} 
 */  /* end of group Magnetometer */



void LSM303DLH_I2C_Init(void);
u8 LSM303DLH_I2C_ByteWrite(u8 slAddr, u8* pBuffer, u8 WriteAddr);
u8 LSM303DLH_I2C_BufferRead(u8 slAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
u8 LSM303DLH_Acc_Config(LSM_Acc_ConfigTypeDef *LSM_Acc_Config_Struct);
u8 LSM303DLH_Acc_Filter_Config(LSM_Acc_Filter_ConfigTypeDef *LSM_Acc_Filter_Config_Struct);
u8 LSM303DLH_Acc_Lowpower_Cmd(u8 LowPowerMode);
u8 LSM303DLH_Acc_FullScale_Cmd(u8 FS_value);
u8 LSM303DLH_Acc_DataRate_Cmd(u8 DataRateValue);
u8 LSM303DLH_Acc_Reboot_Cmd(void);
u8 LSM303DLH_Acc_Read_OutReg(u8* out);
u8 LSM303DLH_Acc_Read_RawData(s16* out);

u8 LSM303DLH_Magn_I2C_Init(void);
u8 LSM303DLH_Magn_Config(LSM_Magn_ConfigTypeDef *LSM_Magn_Config_Struct);
u8 LSM303DLH_Magn_DRDY_Config(void);
u8 LSM303DLH_Magn_Read_OutReg(u8* out);
u8 LSM303DLH_Magn_Read_Magn(float* out);
u8 LSM303DLH_Magn_Read_RawData(s16* out);

u8 LSM303DLH_Read_Acc(float* out);
u8 LSM303DLH_Read_Mag(s16* out);

void LSM303DLH_Raw2Acc(u8 *raw, float *acc);
void LSM303DLH_Raw2Mag(u8 *raw, s16 *mag);

void LSM_Config(void);
/**
 * @} 
 */  /* end of group LSM303DLH */

#endif /* __LSM303DLH_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

