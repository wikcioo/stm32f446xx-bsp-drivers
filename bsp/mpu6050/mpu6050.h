#ifndef __MPU6050_H__
#define __MPU6050_H__

#define MPU6050_SLAVE_ADDR 0x68

/* MPU6050 registers */
#define MPU6050_REG_PWM_MGMT_1      0x6B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_GYRO_CONFIG     0x1B

#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_GYRO_XOUT_H     0x43

/* MPU6050 bit positions */
#define MPU6050_PWR_MGMT_1_SLEEP       6
#define MPU6050_ACCEL_CONFIG_AFS_SEL   3
#define MPU6050_GYRO_CONFIG_FS_SEL     3

/* Full-scale ranges for accelerometer and gyroscope */
#define MPU6050_ACCEL_AFS_SEL_0     0
#define MPU6050_ACCEL_AFS_SEL_1     1
#define MPU6050_ACCEL_AFS_SEL_2     2
#define MPU6050_ACCEL_AFS_SEL_3     3

#define MPU6050_GYRO_FS_SEL_0       0
#define MPU6050_GYRO_FS_SEL_1       1
#define MPU6050_GYRO_FS_SEL_2       2
#define MPU6050_GYRO_FS_SEL_3       3

/* Sensitivity scale factor when full-scale is 0 */
#define MPU6050_ACCEL_AFS_SEL_0_SENSITIVITY     16384
#define MPU6050_GYRO_FS_SEL_0_SENSITIVITY       131

/* Other MPU6050 defines */
#define MPU6050_SUCCESS     1
#define MPU6050_FAIL        0

#define MPU6050_SET         1
#define MPU6050_CLEAR       0

#endif
