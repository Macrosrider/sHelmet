#ifndef _MPU9265_REGISTER_MAP_H_
#define _MPU9265_REGISTER_MAP_H_

enum mpu9265_registers{
	/*
	This header include all registers which access to get raw data from the MPU9265
	*/
#define AK8963_WHO_AM_I             0x00 // should return 0x48
#define MPU9150_RA_MAG_ADDRESS      0x0C
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02  // data ready status bit 0
#define MPU9150_RA_MAG_XOUT_L       0x03
#define MPU9150_RA_MAG_XOUT_H       0x04
#define MPU9150_RA_MAG_YOUT_L       0x05
#define MPU9150_RA_MAG_YOUT_H       0x06
#define MPU9150_RA_MAG_ZOUT_L       0x07
#define MPU9150_RA_MAG_ZOUT_H       0x08
#define AK8963_ST2                  0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL                 0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC                 0x0C  // Self test control
#define AK8963_I2CDIS               0x0F  // I2C disable
#define AK8963_ASAX                 0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY                 0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ                 0x12  // Fuse ROM z-axis sensitivity adjustment value
#define MPU9150_ADDRESS_AD0_LOW     0x68
#define MPU9150_DEFAULT_ADDRESS     MPU9150_ADDRESS_AD0_LOW
#define MPU9150_RA_INT_PIN_CFG      0x37
#define ACCEL_XOUT_H                0x3B
#define ACCEL_XOUT_L                0x3C
#define ACCEL_YOUT_H                0x3D
#define ACCEL_YOUT_L                0x3E
#define ACCEL_ZOUT_H                0x3F
#define ACCEL_ZOUT_L                0x40
#define TEMP_OUT_H                  0x41
#define TEMP_OUT_L                  0x42
#define GYRO_XOUT_H                 0x43
#define GYRO_XOUT_L                 0x44
#define GYRO_YOUT_H                 0x45
#define GYRO_YOUT_L                 0x46
#define GYRO_ZOUT_H                 0x47
#define GYRO_ZOUT_L                 0x48
};
#endif // _MPU9265_REGISTER_MAP_H_