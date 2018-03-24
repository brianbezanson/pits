#define BMP280_ADDRESS	(0x77)
#define BMP280_REGISTER_DIG_T1              0x88
#define BMP280_REGISTER_DIG_T2              0x8A
#define BMP280_REGISTER_DIG_T3              0x8C

#define BMP280_REGISTER_DIG_P1              0x8E
#define BMP280_REGISTER_DIG_P2              0x90
#define BMP280_REGISTER_DIG_P3              0x92
#define BMP280_REGISTER_DIG_P4              0x94
#define BMP280_REGISTER_DIG_P5              0x96
#define BMP280_REGISTER_DIG_P6              0x98
#define BMP280_REGISTER_DIG_P7              0x9A
#define BMP280_REGISTER_DIG_P8              0x9C
#define BMP280_REGISTER_DIG_P9              0x9E

#define BMP280_REGISTER_CHIPID             0xD0
#define BMP280_REGISTER_VERSION            0xD1
#define BMP280_REGISTER_SOFTRESET          0xE0

#define BMP280_REGISTER_CAL26              0xE1  // R calibration stored in 0xE1-0xF0

#define BMP280_REGISTER_CONTROL            0xF4
#define BMP280_REGISTER_CONFIG             0xF5
#define BMP280_REGISTER_PRESSUREDATA       0xF7
#define BMP280_REGISTER_TEMPDATA           0xFA

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#include "gps.h"
#include "misc.h"

struct TBMP280
{
	// i2c file descriptor
	int fd;

	// Calibration constants
	uint16_t dig_T1;
	int16_t  dig_T2, dig_T3;

	uint16_t dig_P1;
	int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

	uint8_t  dig_H1, dig_H3;
	int16_t  dig_H2, dig_H4, dig_H5;
	int8_t   dig_H6;
	
	int32_t  t_fine;
};

int bmp280Calibration (struct TBMP280 *bmp);
double bmp280GetTemperature(struct TBMP280 *bmp);
double bmp280GetPressure(struct TBMP280 *bmp);
int32_t bmp280ReadInt16(int fd, unsigned char address);
int32_t bmp280ReadInt24(int fd, unsigned char address);

void *BMP280Loop(void *some_void_ptr)
{
	struct TBMP280 bmp;
	struct TGPS *GPS;
	
	GPS = ( struct TGPS *)some_void_ptr;
	if(0 <= (bmp.fd = open_i2c(BMP280_ADDRESS)))
	{
		int NoBMP;

		NoBMP = bmp280Calibration(&bmp);
		close(bmp.fd);
		
		if(NoBMP)
		{
			return 0;
		}
	} else 
	{
		return 0;
	}

	while(1)
	{
		if((bmp.fd = open_i2c(BMP280_ADDRESS)) >= 0)
		{
			GPS->BMP180Temperature = bmp280GetTemperature(&bmp);
			GPS->Pressure = bmp280GetPressure(&bmp);
			//printf("Temperature: %fdegC\n", temp/10.0);
			//printf("Pressure: %fPa\n", press);
			close(bmp.fd);
		}

		sleep(10);
	}
	return 0;
}


int32_t bmp280ReadInt16(int fd, unsigned char address)
{
	unsigned char buf[2];

	buf[0] = address;

	if(1 != write(fd, buf, 1)) {
		return -1;
	}

	if(2 != read(fd, buf, 2)) {
		return -1;
	}

	return (int16_t)buf[1]<<8 | buf[0];
}

int32_t bmp280ReadInt24(int fd, unsigned char address)
{
	unsigned char buf[3];

	buf[0]=address;

	if(1 != write(fd, buf, 1)) {
		return -1;
	}
	
	if(3 != read(fd, buf, 3)) {
		return -1;
	}

	return (int32_t) buf[0]<<16 | buf[1]<<8 | buf[2];
}


int bmp280Calibration(struct TBMP280 *bmp)
{
	int res;
	res =  bmp280ReadInt16(bmp->fd, BMP280_REGISTER_DIG_T1);
	if(res < 0){
		return 1;
	}

	bmp->dig_T1 = (uint16_t) bmp280ReadInt16(bmp->fd, BMP280_REGISTER_DIG_T1);
	bmp->dig_T2 = bmp280ReadInt16(bmp->fd, BMP280_REGISTER_DIG_T2);
	bmp->dig_T3 = bmp280ReadInt16(bmp->fd, BMP280_REGISTER_DIG_T3);

	bmp->dig_P1 = (uint16_t) bmp280ReadInt16(bmp->fd, BMP280_REGISTER_DIG_P1);
	bmp->dig_P2 = bmp280ReadInt16(bmp->fd, BMP280_REGISTER_DIG_P2);
	bmp->dig_P3 = bmp280ReadInt16(bmp->fd, BMP280_REGISTER_DIG_P3);
	bmp->dig_P4 = bmp280ReadInt16(bmp->fd, BMP280_REGISTER_DIG_P4);
	bmp->dig_P5 = bmp280ReadInt16(bmp->fd, BMP280_REGISTER_DIG_P5);
	bmp->dig_P6 = bmp280ReadInt16(bmp->fd, BMP280_REGISTER_DIG_P6);
	bmp->dig_P7 = bmp280ReadInt16(bmp->fd, BMP280_REGISTER_DIG_P7);
	bmp->dig_P8 = bmp280ReadInt16(bmp->fd, BMP280_REGISTER_DIG_P8);
	bmp->dig_P9 = bmp280ReadInt16(bmp->fd, BMP280_REGISTER_DIG_P9);

	return 0;
}

double bmp280GetTemperature(struct TBMP280 *bmp)
{
	int32_t var1, var2;
	int32_t adc_T;
	adc_T = bmp280ReadInt24(bmp->fd, BMP280_REGISTER_TEMPDATA);
	adc_T >>= 4;

	var1  = ((((adc_T>>3) - ((int32_t)bmp->dig_T1 <<1))) *
		((int32_t)bmp->dig_T2)) >> 11;

	var2  = (((((adc_T>>4) - ((int32_t)bmp->dig_T1)) *
		((adc_T>>4) - ((int32_t)bmp->dig_T1))) >> 12) *
		((int32_t)bmp->dig_T3)) >> 14;

	bmp->t_fine = var1 + var2;

	float T  = (bmp->t_fine * 5 + 128) >> 8;
	return T/100.0;
}

//Be sure that bmp280GetTemperature has been called recently, since it sets
//the t_fine value that gets used here.  This is taken care of in BMP280Loop
double bmp280GetPressure(struct TBMP280 *bmp)
{
	int64_t var1, var2, p;

	int32_t adc_P = bmp280ReadInt24(bmp->fd, BMP280_REGISTER_PRESSUREDATA);
	adc_P >>= 4;

	var1 = ((int64_t)bmp->t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)bmp->dig_P6;
	var2 = var2 + ((var1*(int64_t)bmp->dig_P5)<<17);
	var2 = var2 + (((int64_t)bmp->dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)bmp->dig_P3)>>8) +
		((var1 * (int64_t)bmp->dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bmp->dig_P1)>>33;

	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p<<31) - var2)*3125) / var1;
	var1 = (((int64_t)bmp->dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)bmp->dig_P8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t)bmp->dig_P7)<<4);
	return (float)p/256;
}

