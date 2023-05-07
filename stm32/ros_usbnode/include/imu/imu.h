
#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

typedef struct 
{
    double x, y, z;
} VECTOR;

/*
 * conversions to ROS units
 */
#define RAD_PER_G               0.01745             // convert °/sec to rad/sec
#define MS2_PER_G               9.80665             // convert G to m/s^2 
#define T_PER_GAUSS             1/10000             // convert Gauss to T

// magnetometer data is only used relative, so the magniute should not matter,
// however ROS defines Tesla as unit - so if another ros node requires it we can convert it here
// default is to report data in Tesla
#define MAG_IN_TESLA            1

/*
 * IMU functions that a compatible IMU needs to be able to provide
 */




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
uint8_t IMU_TestDevice(void);

/**
  * @brief  Initialize IMU
  *  
  */
void IMU_Init(void);

/**
  * @brief  Reads the 3 accelerometer channels and stores them in *x,*y,*z
  * units are m/s^2
  */
void IMU_ReadAccelerometerRaw(float *x, float *y, float *z);

/**
  * @brief  Reads the 3 gyro channels and stores them in *x,*y,*z
  * units are rad/sec
  */
void IMU_ReadGyroRaw(float *x, float *y, float *z);

/**
  * @brief  Reads the 3 magnetometer channels and stores them in *x,*y,*z  
  * units are tesla uncalibrated
  */
void IMU_ReadMagnetometerRaw(double *x, double *y, double *z);

/**
  * @brief  Reads the raw temp value
  * (internal function only)
  * @retval float temp in °C
  */
float IMU_TempRaw(void);

void IMU_ReadAccelerometer(float *x, float *y, float *z);
void IMU_Onboard_ReadAccelerometer(float *x, float *y, float *z);
float IMU_Onboard_ReadTemp(void);
void IMU_ReadGyro(float *x, float *y, float *z);
void IMU_ReadMagnetometer(double *x, double *y, double *z);
void IMU_ReadMagnetometerNormalized(double *x, double *y, double *z);
float IMU_ReadBarometerTemperatureC(void);
float IMU_ReadBarometerAltitudeMeters(void);
void IMU_Onboard_AccelerometerSetCovariance(float *cm);
void IMU_AccelerometerSetCovariance(float *cm);
void IMU_GyroSetCovariance(float *cm);
void IMU_ApplyMagTransformation(double x, double y, double z, double *x_cal, double *y_cal, double *z_cal);
void IMU_Normalize( VECTOR* p );
float IMU_MagHeading(void);


/* end of functions to implement for IMU */


/* IMU calibration (accel/gyro only) */
#define IMU_CAL_SAMPLES     100
void IMU_Calibrate(void);

/* External Magnetometer Calibration Values */
extern double external_imu_mag_bias[3];          // hard iron compensation
extern double external_imu_mag_cal_matrix[3][3]; // soft iron compensation
#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */