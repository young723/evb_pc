#ifndef I2C_SPI_H
#define I2C_SPI_H
#include "USBIOX.H"
#include "USB2UARTSPIIICDLL.h"

enum
{
	USB_NONE,
	USB_I2C,
	USB_SPI,
	USB_SERIAL,
	USB_MAX
};

enum
{
	DEVICE_NONE,
	DEVICE_CH341A,
	DEVICE_STM32F103,
	DEVICE_CP2102,
	DEVICE_MAX,
};


typedef struct
{
	int				protocol;
	int				device;
	//unsigned char slave;
	BOOL			open;
	UINT			mCh341Index;
	HANDLE			mCh341Handle;	
	UINT			mStm32Index;
	unsigned char	buf[64];
}i2c_spi_t;


int i2c_spi_open_device(void);
BOOL i2c_spi_close_device(void);
BOOL i2c_init(int device, int rate);
BOOL spi_init(int device, int rate, int mode);
BOOL i2c_write_reg(unsigned char slave, unsigned char addr, unsigned char value);
BOOL i2c_read_reg(unsigned char slave, unsigned char addr, unsigned char *buf, unsigned short len);
BOOL spi_write_reg(unsigned char addr, unsigned char value);
BOOL spi_read_reg(unsigned char addr, unsigned char *buf, unsigned short len);
int get_i2c_spi_protocol(void);
BOOL device_write_io(unsigned char value);

#endif
