#define MS5637_CMD_READ_PROM1 	0xA1
#define MS5637_CMD_READ_PROM2 	0xA2
#define MS5637_CMD_READ_PROM3 	0xA3
#define MS5637_CMD_READ_PROM4 	0xA4
#define MS5637_CMD_READ_PROM5 	0xA5
#define MS5637_CMD_READ_PROM6 	0xA6
#define MS5637_CMD_RESET	0x1E
#define MS5637_CMD_READ_RESULT	0x00
//These are conversion commands for maximum oversampling, taking 16.44ms.
//See datasheet for faster, lower resolution conversion commands.
#define MS5637_CMD_CONV_D1	0x4A
#define MS5637_CMD_CONV_D2	0x5A
#define MS5637_ADDRESS		0x76

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

struct TMS5637
{
	// i2c file descriptor
	int fd;

	// Calibration constants
	uint16_t C1, C2, C3, C4, C5, C6;	

	//temp in 0.01 degree C and pressure in Pa
	int32_t temp, press;
};

int ms5637Calibration(struct TMS5637 *baro);
void ms5637Calculate(struct TMS5637 *baro);
double ms5637GetTemperature(struct TMS5637 *baro);
double ms5637GetPressure(struct TMS5637 *baro);
int32_t ms5637ReadInt16(int fd, unsigned char address);
int32_t ms5637ReadInt24(int fd, unsigned char address);
int ms5637SendCommand(int fd, unsigned char cmd);

void *MS5637Loop(void *some_void_ptr)
{
	struct TMS5637 baro;
	struct TGPS *GPS;
	
	GPS = ( struct TGPS *)some_void_ptr;
	if(0 <= (baro.fd = open_i2c(MS5637_ADDRESS)))
	{
		int NoBMP;

		NoBMP = ms5637Calibration(&baro);
		close(baro.fd);
		
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
		if((baro.fd = open_i2c(MS5637_ADDRESS)) >= 0)
		{
			ms5637Calculate(&baro);
			GPS->BMP180Temperature = ms5637GetTemperature(&baro);
			GPS->Pressure = ms5637GetPressure(&baro);
			//printf("Temperature: %fdegC\n", temp/10.0);
			//printf("Pressure: %fPa\n", press);
			close(baro.fd);
		}

		sleep(10);
	}
	return 0;
}


int32_t ms5637ReadUInt16(int fd, unsigned char address)
{
	unsigned char buf[2];

	buf[0] = address;

	if(1 != write(fd, buf, 1)) {
		return -1;
	}

	if(2 != read(fd, buf, 2)) {
		return -1;
	}

	return (uint16_t)buf[0]<<8 | buf[1];
}

int32_t ms5637ReadInt24(int fd, unsigned char address)
{
	unsigned char buf[3];

	buf[0]=address;

	if(1 != write(fd, buf, 1)) {
		return -1;
	}
	
	if(3 != read(fd, buf, 3)) {
		return -1;
	}

	return (int32_t) buf[2]<<16 | buf[1]<<8 | buf[0];
}

int ms5637SendCommand(int fd, unsigned char cmd){
	return write(fd, &cmd, 1);
}

int ms5637Calibration(struct TMS5637 *baro)
{
	int res;
	unsigned char cmd;

	cmd = MS5637_CMD_RESET;
	res = write(baro->fd, &cmd, 1);
	//see if it's happy without a delay after that reset?
	//
	if(res < 1){
		return 1;
	}
	baro->C1 = ms5637ReadUInt16(baro->fd, MS5637_CMD_READ_PROM1);
	baro->C2 = ms5637ReadUInt16(baro->fd, MS5637_CMD_READ_PROM2);
	baro->C3 = ms5637ReadUInt16(baro->fd, MS5637_CMD_READ_PROM3);
	baro->C4 = ms5637ReadUInt16(baro->fd, MS5637_CMD_READ_PROM4);
	baro->C5 = ms5637ReadUInt16(baro->fd, MS5637_CMD_READ_PROM5);
	baro->C6 = ms5637ReadUInt16(baro->fd, MS5637_CMD_READ_PROM6);

	return 0;
}

void ms5637Calculate(struct TMS5637 *baro)
{
	uint32_t raw_temp; //direct reading from ADC
	int32_t temp; //compensated temperature in hundredths of a degree C
	int32_t dT; //intermediate step in temp calculation
	uint32_t raw_press; //direct reading from ADC
	int32_t press; //first-order compensated pressure in Pa
	int64_t offset, sensitivity; //pressure calculation intermediates
	int64_t t2, off2, sens2; //second-order correction intermediates

	//read temperature data
	ms5637SendCommand(baro->fd, MS5637_CMD_CONV_D2);
	usleep(17000); //datasheet says this will take 16.44ms
	raw_temp = ms5637ReadInt24(baro->fd, MS5637_CMD_READ_RESULT);

	//temperature calculation
	dT = raw_temp - ((uint32_t)baro->C5 << 8);
	temp = 2000 + (int64_t)dT * (int64_t)baro->C6/(1<<23);

	//read pressure data
	ms5637SendCommand(baro->fd, MS5637_CMD_CONV_D1);
	usleep(17000);
	raw_press = ms5637ReadInt24(baro->fd, MS5637_CMD_READ_RESULT);

	//pressure calculation
	offset = ((int64_t)baro->C2<<17) + (baro->C4*dT)/(1<<6);
	sensitivity = ((int64_t)baro->C1<<16) + (int64_t)baro->C3*(int64_t)dT/(1<<7);

	//second-order corrections
	if(temp<2000)
	{
		t2 = 3*(int64_t)dT*(int64_t)dT/((int64_t)1<<33);
		off2 = 61*(temp-2000)*(temp-2000)/16;
		sens2 = 29*(temp-2000)*(temp-2000)/16;
		
		if(temp < -1500)
		{
			off2 += 17*(temp+1500)*(temp+1500);
			sens2 += 9*(temp+1500)*(temp+1500);
		}
	}
	else
	{
		t2 = 5*(int64_t)dT*(int64_t)dT/((int64_t)1<<38);
		off2 = sens2 = 0;
	}

	temp -= t2;
	offset -= off2;
	sensitivity -= sens2;

	press = ((raw_press*sensitivity)/(1<<21) - offset)/(1<<15);

	baro->temp = temp;
	baro->press = press;
}

double ms5637GetPressure(struct TMS5637 *baro)
{
	return (double)baro->press;
}

double ms5637GetTemperature(struct TMS5637 *baro)
{
	return (double)baro->temp/100.0;
}
