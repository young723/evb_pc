// comm_demo.cpp : 定义应用程序的类行为。
//

#include "stdafx.h"
#include "i2c_spi.h"

#define I2C_SPI_LOG		printf

static i2c_spi_t device_info = 
{
	USB_I2C,
	DEVICE_CH341A,
	FALSE,
	0,
	INVALID_HANDLE_VALUE,
	0,
	{0,}
};

int i2c_spi_open_device(void)
{	
	if(device_info.open)
	{	
		if(device_info.mCh341Handle != INVALID_HANDLE_VALUE)
			return DEVICE_CH341A;
		else
			return DEVICE_STM32F103;
	}
	else
	{
		if(device_info.mCh341Handle == INVALID_HANDLE_VALUE)
		{
			device_info.mCh341Handle = USBIO_OpenDevice(device_info.mCh341Index);
		}
		if(device_info.mCh341Handle != INVALID_HANDLE_VALUE)
		{
			device_info.open = TRUE;
			return DEVICE_CH341A;			
		}
		else
		{
			device_info.open = OpenUsb(device_info.mStm32Index);
			if(device_info.open)
				return DEVICE_STM32F103;
		}
	}

	return DEVICE_NONE;
}

BOOL i2c_spi_close_device(void)
{
	if(device_info.open)
	{
		if(device_info.mCh341Handle!=INVALID_HANDLE_VALUE)
		{
			USBIO_CloseDevice(device_info.mCh341Index);
			device_info.mCh341Handle=INVALID_HANDLE_VALUE;
			device_info.open = FALSE;
		}
		else
		{
			CloseUSB(device_info.mStm32Index);
			device_info.open = FALSE;
		}
	}

	return TRUE;
}


BOOL i2c_init(int device, int rate)
{
	BOOL status = FALSE;

	if(device == DEVICE_CH341A)
	{
		if(device_info.mCh341Handle==INVALID_HANDLE_VALUE)
		{
			device_info.mCh341Handle = USBIO_OpenDevice(device_info.mCh341Index);
		}
		I2C_SPI_LOG("mCh341Handle = %d \n", (int)device_info.mCh341Handle);
		if(device_info.mCh341Handle != INVALID_HANDLE_VALUE)
		{
			device_info.open = TRUE;
			//0x81-100k 0x82-400k 0x83-750k
			if(rate == 20*1000)
				USBIO_SetStream(device_info.mCh341Index, 0x80);
			else if(rate == 100*1000)
				USBIO_SetStream(device_info.mCh341Index, 0x81);
			else if(rate == 400*1000)
				USBIO_SetStream(device_info.mCh341Index, 0x82);
			else if(rate == 750*1000)
				USBIO_SetStream(device_info.mCh341Index, 0x83);
			else
				USBIO_SetStream(device_info.mCh341Index, 0x81);
			// 设置中断，INT脚上升沿，执行回调函数。
			//USBIO_SetIntRoutine(device_info.mCh341Index,ch341_int_callback);
		}
		else
		{
			device_info.open = FALSE;
			I2C_SPI_LOG("CH341A open fail!\n");
		}
	}
	else if(device == DEVICE_STM32F103)
	{
		if(device_info.open == FALSE)
		{
			status = OpenUsb(device_info.mStm32Index);
			device_info.open = status;
		}
		I2C_SPI_LOG("stm32f103 open status = %d \n", status);
		if(device_info.open == TRUE)
		{
			device_info.open = TRUE;
			if(rate == 1*1000)
				ConfigIICParam(IIC_Rate_1K, device_info.mStm32Index);
			else if(rate == 10*1000)
				ConfigIICParam(IIC_Rate_10K, device_info.mStm32Index);
			else if(rate == 100*1000)
				ConfigIICParam(IIC_Rate_100K, device_info.mStm32Index);
			else if(rate == 400*1000)
				ConfigIICParam(IIC_Rate_400K, device_info.mStm32Index);
		}
		else
		{
			device_info.open = FALSE;
			I2C_SPI_LOG("STM32F103 open fail!\n");
		}
	}
	device_info.protocol = USB_I2C;
	device_info.device = device;
	memset(device_info.buf, 0, sizeof(device_info.buf));
	
	return device_info.open;
}

BOOL spi_init(int device, int rate, int mode)
{
	BOOL status = FALSE;

	if(device == DEVICE_CH341A)
	{
		if(device_info.mCh341Handle==INVALID_HANDLE_VALUE)
		{
			device_info.mCh341Handle = USBIO_OpenDevice(device_info.mCh341Index);
		}
		I2C_SPI_LOG("mCh341Handle = %d \n", (int)device_info.mCh341Handle);
		if(device_info.mCh341Handle != INVALID_HANDLE_VALUE)
		{
			device_info.open = TRUE;			
			USBIO_Set_D5_D0(device_info.mCh341Index,0xff,0x00); 	// add by yangzhiqiang
		}
		else
		{
			device_info.open = FALSE;
			I2C_SPI_LOG("CH341A open fail!\n");
		}
	}
	else if(device == DEVICE_STM32F103)
	{
		if(device_info.open == FALSE)
		{
			status = OpenUsb(device_info.mStm32Index);
			device_info.open = status;
		}
		I2C_SPI_LOG("stm32f103 open status = %d \n", status);
		if(device_info.open == TRUE)
		{
			device_info.open = TRUE;
			if(rate <= 2*1000*1000)
				ConfigSPIParam(SPI_Rate_1_125M, SPI_MSB, mode, device_info.mStm32Index);
			else if(rate <= 4*1000*1000)
				ConfigSPIParam(SPI_Rate_2_25M, SPI_MSB, mode, device_info.mStm32Index);
			else if(rate <= 6*1000*1000)
				ConfigSPIParam(SPI_Rate_4_5M, SPI_MSB, mode, device_info.mStm32Index);
			else if(rate <= 10*1000*1000)
				ConfigSPIParam(SPI_Rate_9M, SPI_MSB, mode, device_info.mStm32Index);
			else
				ConfigSPIParam(SPI_Rate_18M, SPI_MSB, mode, device_info.mStm32Index);
		}
		else
		{
			device_info.open = FALSE;
			I2C_SPI_LOG("STM32F103 open fail!\n");
		}
	}
	device_info.protocol = USB_SPI;
	device_info.device = device;
	memset(device_info.buf, 0, sizeof(device_info.buf));
	
	return device_info.open;
}



BOOL i2c_write_reg(unsigned char slave, unsigned char addr, unsigned char value)
{
	BOOL status = FALSE;

	if(device_info.device == DEVICE_CH341A)
	{
		status = USBIO_WriteI2C(device_info.mCh341Index, slave, addr, value);
		//Sleep(2);	// add by yangzhiqiang 0819
	}
	else if(device_info.device == DEVICE_STM32F103)
	{
		device_info.buf[0] = slave<<1;
		device_info.buf[1] = addr;
		device_info.buf[2] = value;
		status = IICSendData(TRUE, device_info.buf, 3, 1, device_info.mStm32Index);
	}

	return status;
}

BOOL i2c_read_reg(unsigned char slave, unsigned char addr, unsigned char *buf, unsigned short len)
{
	BOOL status = FALSE;

	if(device_info.device == DEVICE_CH341A)
	{
		device_info.buf[0] = slave<<1;
		device_info.buf[1] = addr;
		status = USBIO_StreamI2C(device_info.mCh341Index, 2, device_info.buf, len, buf);
	}
	else if(device_info.device == DEVICE_STM32F103)
	{
		device_info.buf[0] = slave<<1;
		device_info.buf[1] = addr;
		status = IICSendData(TRUE, device_info.buf, 2, 0, device_info.mStm32Index);
		device_info.buf[0] = (slave<<1)|0x01;
		status = IICSendData(TRUE, device_info.buf, 1, 0, device_info.mStm32Index);
		status = IICRcvData(buf, len, 1, device_info.mStm32Index);
	}

	return status;
}

BOOL spi_write_reg(unsigned char addr, unsigned char value)
{
	BOOL status = FALSE;

	if(device_info.device == DEVICE_CH341A)
	{
		device_info.buf[0] = addr;
		device_info.buf[1] = value;
		status = USBIO_StreamSPI4(device_info.mCh341Index, 0x80, 2, device_info.buf);		// use CS0
	}
	else if(device_info.device == DEVICE_STM32F103)
	{
		device_info.buf[0] = addr;
		device_info.buf[1] = value;
		status = SPISendData(0, 1, device_info.buf, 2, device_info.mStm32Index);
	}
	return status;
}

BOOL spi_read_reg(unsigned char addr, unsigned char *buf, unsigned short len)
{
	BOOL status = FALSE;

	if(device_info.device == DEVICE_CH341A)
	{
		device_info.buf[0] = addr;
		status = USBIO_StreamSPI4(device_info.mCh341Index, 0x80, len+1, device_info.buf);		// use CS0
		memcpy(buf, &device_info.buf[1], len);
	}
	else if(device_info.device == DEVICE_STM32F103)
	{
		device_info.buf[0] = addr;
		status = SPISendData(0, 0, device_info.buf, 1, device_info.mStm32Index);
		status = SPIRcvData(0, 1, buf, len, device_info.mStm32Index);
	}
	return status;
}

int get_i2c_spi_protocol(void)
{
	return device_info.protocol;
}

BOOL device_write_io(unsigned char value)
{
	int index;
	BOOL status;

	if(device_info.device == DEVICE_CH341A)
	{
		status = USBIO_Set_D5_D0(device_info.mCh341Index, 0x3f, 0x3f&value);
	}
	else if(device_info.device == DEVICE_STM32F103)
	{
		for(index=0; index<8; index++)
		{
			if((value)&(1<<index))
				status = IOSetAndRead(index, 1, 1, device_info.mStm32Index);
			else
				status = IOSetAndRead(index, 1, 0, device_info.mStm32Index);
		}
	}

	return status;
}

