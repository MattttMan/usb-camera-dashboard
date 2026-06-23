/* *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ *
* File: icm_decode.h
* Description : Header file for ICM42688 IMU data decoding
* Author : TSTC
* Date : 2025 - 09 - 16
* Version : 10.00.00
​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ */

#ifndef _ICM42688_DECODE_H_
#define _ICM42688_DECODE_H_

#include <string>
#include <stdint.h>
#include <stdio.h>



/* *​
* @brief ICM42688 IMU data structure with floating point values
* Contains timestamp and accelerometer / gyroscope data in floating point format
*/
typedef struct {
	uint64_t	uTime;			/* *​ < Timestamp in microseconds(watch for 32 - bit overflow)*/
	float		fAccData_X;		/* *​ < Accelerometer X - axis data */
	float		fAccData_Y;		/* *​ < Accelerometer Y - axis data */
	float		fAccData_Z;		/* *​ < Accelerometer Z - axis data */
	float		fGyroData_X;	/* *​ < Gyroscope X - axis data */
	float		fGyroData_Y;	/* *​ < Gyroscope Y - axis data */
	float		fGyroData_Z;	/* *​ < Gyroscope Z - axis data */
}sICM42688_XYZ_float;



typedef struct {
	uint64_t	uTime;			/* *​ < Timestamp in microseconds(watch for 32 - bit overflow)*/
	int32_t		iAccData_X;		/* *​ < Raw accelerometer X - axis data */
	int32_t		iAccData_Y;		/* *​ < Raw accelerometer Y - axis data */
	int32_t		iAccData_Z;		/* *​ < Raw accelerometer Z - axis data */
	int32_t		iGyroData_X;	/* *​ < Raw gyroscope X - axis data */
	int32_t		iGyroData_Y;	/* *​ < Raw gyroscope Y - axis data */
	int32_t		iGyroData_Z;	/* *​ < Raw gyroscope Z - axis data */
}sICM42688_XYZ_int;






/* Accelerometer full scale range definitions */
#define AFS_2G  0x03			/* *​ < ±2g accelerometer range */
#define AFS_4G  0x02 			/* *​ < ±4g accelerometer range (default) */
#define AFS_8G  0x01			/* *​ < ±8g accelerometer range */
#define AFS_16G 0x00  			/* *​ < ±16g accelerometer range */

/* Gyroscope full scale range definitions */
#define GFS_2000DPS   0x00		/* *​ < ±2000 dps gyroscope range */
#define GFS_1000DPS   0x01 		/* *​ ​< ±1000 dps gyroscope range (default) */
#define GFS_500DPS    0x02		/* *​ < ±500 dps gyroscope range */
#define GFS_250DPS    0x03		/* *​ ​< ±250 dps gyroscope range */
#define GFS_125DPS    0x04		/* *​ < ±125 dps gyroscope range */
#define GFS_62_5DPS   0x05		/* *​ ​< ±62.5 dps gyroscope range */
#define GFS_31_25DPS  0x06		/* *​ ​< ±31.25 dps gyroscope range */
#define GFS_15_125DPS 0x07		/* *​ ​< ±15.125 dps gyroscope range */


/* *​
* @brief Convert 4 bytes to 32 - bit unsigned integer
* @param _val_ Destination variable for converted value
* @param p_dt_ Pointer to source data bytes
*/
#define D_U8_TO_U32(_val_, p_dt_) \
    { \
        (_val_) = ((p_dt_)[0]); \
        (_val_) = ((_val_) << 8) | ((p_dt_)[1]); \
        (_val_) = ((_val_) << 8) | ((p_dt_)[2]); \
        (_val_) = ((_val_) << 8) | ((p_dt_)[3]); \
    }
/* *​
* @brief Convert 2 bytes to 16 - bit unsigned integer
* @param _val_ Destination variable for converted value
* @param p_dt_ Pointer to source data bytes
*/
#define D_U8_TO_U16(_val_, p_dt_) \
    { \
        (_val_) = ((p_dt_)[0]); \
        (_val_) = ((_val_) << 8) | ((p_dt_)[1]); \
    }




/* *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ *
*@brief Get accelerometer resolution based on scale setting
* @param[in] Ascale Accelerometer scale setting(AFS_2G / 4G / 8G / 16G)
* @return Accelerometer resolution in g / LSB
*
*@details Possible accelerometer scales(and their register bit settings) :
*±2g(11), ±4g(10), ±8g(01), and ±16g(00)
​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​*/

static inline float_t	F_Dsp_Icm42688GetAres(uint8_t Ascale)
{
	float_t accSensitivity = 0.244f;    //Resolution of acceleration	mg/LSB

	switch (Ascale)
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00).
	case AFS_2G:
		accSensitivity = 2000 / 32768.0f;
		break;
	case AFS_4G:
		accSensitivity = 4000 / 32768.0f;
		break;
	case AFS_8G:
		accSensitivity = 8000 / 32768.0f;
		break;
	case AFS_16G:
		accSensitivity = 16000 / 32768.0f;
		break;
	}

	return accSensitivity;
}

/* *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ *
*@brief Get gyroscope resolution based on scale setting
* @param[in] Gscale Gyroscope scale setting
* @return Gyroscope resolution in dps / LSB
​* *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ */

static inline float		F_Dsp_Icm42688GetGres(uint8_t Gscale)
{
	float_t gyroSensitivity = 32.8f;     //resolution of gyroscope

	switch (Gscale)
	{
	case GFS_15_125DPS:
		gyroSensitivity = 15.125f / 32768.0f;
		break;
	case GFS_31_25DPS:
		gyroSensitivity = 31.25f / 32768.0f;
		break;
	case GFS_62_5DPS:
		gyroSensitivity = 62.5f / 32768.0f;
		break;
	case GFS_125DPS:
		gyroSensitivity = 125.0f / 32768.0f;
		break;
	case GFS_250DPS:
		gyroSensitivity = 250.0f / 32768.0f;
		break;
	case GFS_500DPS:
		gyroSensitivity = 500.0f / 32768.0f;
		break;
	case GFS_1000DPS:
		gyroSensitivity = 1000.0f / 32768.0f;
		break;
	case GFS_2000DPS:
		gyroSensitivity = 2000.0f / 32768.0f;
		break;
	}
	return gyroSensitivity;
}

/* *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ *
*@brief Extract  ICM42688 IMU data(integer format)
* @param[out] s_ICM Structure containing IMU data and timestamp
* @param[in] pDt Pointer to decoded data buffer
*
*@details If error is detected(all axes - 1 or gyro X = -32768), the structure is cleared except for timestamp
​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​*/
static inline void F_Analysis_Icm42688_Int(sICM42688_XYZ_int& s_ICM, const uint8_t* pDt)
{
	uint32_t uval32 = 0;

	int16_t  ival16 = 0;

	D_U8_TO_U32(uval32, pDt + 0);

	s_ICM.uTime = uval32;

	D_U8_TO_U16(ival16, pDt + 4);	s_ICM.iAccData_X = ival16;	ival16 = 0;
	D_U8_TO_U16(ival16, pDt + 6);	s_ICM.iAccData_Y = ival16;	ival16 = 0;
	D_U8_TO_U16(ival16, pDt + 8);	s_ICM.iAccData_Z = ival16;	ival16 = 0;
	D_U8_TO_U16(ival16, pDt + 10);	s_ICM.iGyroData_X = ival16;	ival16 = 0;
	D_U8_TO_U16(ival16, pDt + 12);	s_ICM.iGyroData_Y = ival16;	ival16 = 0;
	D_U8_TO_U16(ival16, pDt + 14);	s_ICM.iGyroData_Z = ival16;	ival16 = 0;

	// Check for IMU error conditions
	if ((s_ICM.iAccData_X == -1 && s_ICM.iAccData_Y == -1 && s_ICM.iAccData_Z == -1) || s_ICM.iGyroData_X == -32768)
	{
		memset(&s_ICM, 0, sizeof(s_ICM));

		s_ICM.uTime = uval32;
	}

	/*
	fprintf(stdout, "IMC RAW AccData X,Y,Z (%d,\t%d,\t%d)	GyroData X,Y,Z(%d,\t%d,\t%d) sampling time[%ldus]\r\n",
		s_ICM.iAccData_X,
		s_ICM.iAccData_Y,
		s_ICM.iAccData_Z,
		s_ICM.iGyroData_X,
		s_ICM.iGyroData_Y,
		s_ICM.iGyroData_Z,
		s_ICM.uTime);
	*/

}

/* *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ * ​ * *​ *
*@brief Extract and convert ICM42688 IMU data to physical units
* @param[out] s_ICM42688 Structure containing converted sensor data
* @param[in] pDt Pointer to decoded data buffer
* @param[in] Ascale Accelerometer scale setting(default: AFS_4G)
* @param[in] Gscale Gyroscope scale setting(default: GFS_1000DPS)
​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​* ​** ​*/
static inline void F_Analysis_Icm42688(sICM42688_XYZ_float& s_ICM42688, const uint8_t* pDt, uint8_t Ascale, uint8_t Gscale)
{
	float_t	accSensitivity = F_Dsp_Icm42688GetAres(Ascale);
	float_t gyroSensitivity = F_Dsp_Icm42688GetGres(Gscale);

	sICM42688_XYZ_int	s_ICM42688_Raw;

	F_Analysis_Icm42688_Int(s_ICM42688_Raw, pDt);


	s_ICM42688.uTime = s_ICM42688_Raw.uTime;
	s_ICM42688.fAccData_X = s_ICM42688_Raw.iAccData_X * accSensitivity;
	s_ICM42688.fAccData_Y = s_ICM42688_Raw.iAccData_Y * accSensitivity;
	s_ICM42688.fAccData_Z = s_ICM42688_Raw.iAccData_Z * accSensitivity;
	s_ICM42688.fGyroData_X = s_ICM42688_Raw.iGyroData_X * gyroSensitivity;
	s_ICM42688.fGyroData_Y = s_ICM42688_Raw.iGyroData_Y * gyroSensitivity;
	s_ICM42688.fGyroData_Z = s_ICM42688_Raw.iGyroData_Z * gyroSensitivity;
}


#endif //ICM42688_DECODE_H_
