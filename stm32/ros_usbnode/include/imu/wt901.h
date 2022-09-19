/****************************************************************************
* Title                 :   WT901 driver
* Filename              :   WT901.h
* Author                :   Nekraus
* Origin Date           :   15/09/2022
* Version               :   1.0.0

*****************************************************************************/
/** \file WT901.h
*  \brief 
*
*/
#ifndef __WT901_H
#define __WT901_H

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
* Includes
*******************************************************************************/
#include "stm32f1xx_hal.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Constants
*******************************************************************************/

/******************************************************************************
* Macros
*******************************************************************************/

/******************************************************************************
* Typedefs
*******************************************************************************/

/******************************************************************************
* Variables
*******************************************************************************/

/******************************************************************************
* PUBLIC Function Prototypes
*******************************************************************************/
/**
  * @brief  Test Device 
  * Perform any tests possible before actually enabling and using the device,
  * for example check the i2c address and whoami registers if existing  
  *
  * @retval          0 -> test failed 1-> test ok, good to init and use
  *
  */
uint8_t WT901_TestDevice(void);

/**
  * @brief  Initialize IMU
  *  
  */
void WT901_Init(void);

/**
  * @brief  Reads the 3 accelerometer channels and stores them in *x,*y,*z
  * units are m/s^2
  */
void WT901_ReadAccelerometerRaw(float *x, float *y, float *z);

/**
  * @brief  Reads the 3 gyro channels and stores them in *x,*y,*z
  * units are rad/sec
  */
void WT901_ReadGyroRaw(float *x, float *y, float *z);

/**
  * @brief  Reads the 3 magnetometer channels and stores them in *x,*y,*z  
  * units are tesla uncalibrated
  */
void WT901_ReadMagnetometerRaw(double *x, double *y, double *z);

/**
  * @brief  Reads the raw temp value
  * (internal function only)
  * @retval float temp in °C
  */
float WT901_TempRaw(void);

#ifdef __cplusplus
}
#endif

#endif /*WT901_H*/ 

/*** End of File **************************************************************/