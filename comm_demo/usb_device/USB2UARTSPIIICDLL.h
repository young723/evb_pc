/*********************************************************************************
ʹ�ñ�ת�Ӱ���Ĳ����ú�����
��һ������usb���ú��� OpenUsb
�ڶ���������UART����SPI����IIC����
        ���ú�����ConfigUARTParam �� ConfigSPIParam �� ConfigIICParam
�����������ͻ��������
        �������ݵ��ã�UARTSendData ���� SPISendData ���� IICSendData
        �������ݵ��ã�UARTRcvData ���� SPIRcvData ���� IICRcvData
        ����IO�ڵ��ã�IOSetAndRead
���Ĳ����ر�USB���ú��� CloseUSB
*********************************************************************************/
/*********************************************************************************
UsbIndex:
����ÿ��������UsbIndex������˵����
1. ������ĵ�����ֻ������һ��ת�Ӱ壬�������ʹ��Ĭ��ֵ0
2. ������ĵ����������������������������豸���������ָ���ӵ����Եĵڼ���ת�Ӱ壬��0��ʼ�����9
*********************************************************************************/
#ifndef __USB2UARTSPIIICDLL_H__
#define	__USB2UARTSPIIICDLL_H__

#ifdef USB2UARTSPIIICDLL_EXPORTS
#define USB2UARTSPIIICDLL_API extern "C" __declspec(dllexport)
#else
#define USB2UARTSPIIICDLL_API extern "C" __declspec(dllimport)
#endif


//UART������
#define UART_BaudRate_1200                  1200
#define UART_BaudRate_2400                  2400
#define UART_BaudRate_4800                  4800
#define UART_BaudRate_9600                  9600
#define UART_BaudRate_14400                 14400
#define UART_BaudRate_19200                 19200
#define UART_BaudRate_38400                 38400
#define UART_BaudRate_57600                 57600
#define UART_BaudRate_115200                115200
//UART��żУ��λ
#define UART_Parity_No                      0
#define UART_Parity_Even                    1
#define UART_Parity_Odd                     2
//UARTֹͣλ
#define UART_StopBits_1                     0
#define UART_StopBits_1_5                   1
#define UART_StopBits_2                     2
//SPI����
#define SPI_Rate_281K           0
#define SPI_Rate_562K           1
#define SPI_Rate_1_125M           2
#define SPI_Rate_2_25M            3
#define SPI_Rate_4_5M           4
#define SPI_Rate_9M             5
#define SPI_Rate_18M            6
#define SPI_Rate_1K            7
//SPI֡��ʽ
#define SPI_MSB               0
#define SPI_LSB               1
//SPI��ģʽ
#define SPI_SubMode_0           0
#define SPI_SubMode_1           1
#define SPI_SubMode_2           2
#define SPI_SubMode_3           3
//IIC����
#define IIC_Rate_100K           0
#define IIC_Rate_200K           1
#define IIC_Rate_400K           2
#define IIC_Rate_10K           3
#define IIC_Rate_1K           4
//SPI��ģʽ���Ƿ���CS�ܽ�
#define SPI_SLAVE_CSDS               0
#define SPI_SLAVE_CSEN               1

//IO��
/****************************************************************************
���ƣ�OpenUsb                           
������UsbIndex USB����(0-9)                               
����ֵ��TRUE    ��ת�Ӱ�USB�ɹ� 
        FALSH   ��ת�Ӱ�USBʧ��
���ܣ���ת�Ӱ�USB
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL OpenUsb(int UsbIndex);


/****************************************************************************
���ƣ�CloseUSB                            
������UsbIndex USB����(0-9)                                
����ֵ��TRUE    �ر�ת�Ӱ�USB�ɹ� 
        FALSH   �ر�ת�Ӱ�USBʧ��
���ܣ��ر�ת�Ӱ�USB
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL CloseUSB(int UsbIndex);


/****************************************************************************
���ƣ�ConfigUARTParam                           
������baudRate  UART������     ֻ�������²���:
				UART_BaudRate_1200  
                UART_BaudRate_2400  
                UART_BaudRate_4800  
                UART_BaudRate_9600  
                UART_BaudRate_14400 
                UART_BaudRate_19200 
                UART_BaudRate_38400 
                UART_BaudRate_57600 
                UART_BaudRate_115200
      parity    UART��żУ��λ  ֻ�������²���:
                UART_Parity_No                    ��У��
                UART_Parity_Even                  żУ��
                UART_Parity_Odd                   ��У��
      stopBits  UARTֹͣλλ    ֻ�������²���:
                UART_StopBits_1                   1λֹͣλ
                UART_StopBits_1_5                 1.5λֹͣλ
                UART_StopBits_2                   2λֹͣλ

         
����ֵ��TRUE    �������óɹ�  
        FALSH   ��������ʧ��
���ܣ�����UART����
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL ConfigUARTParam (unsigned int baudRate,unsigned int parity,unsigned int stopBits,int UsbIndex);

/****************************************************************************
���ƣ�ConfigSPIParam                           
������rate      SPI����     ֻ�������²���:
                SPI_Rate_281K                   281K      
                SPI_Rate_562K                   562K  
                SPI_Rate_1_125M                 1.125M  
                SPI_Rate_2_25M                  2.25M
                SPI_Rate_4_5M                   4.5M
                SPI_Rate_9M                     9M
                SPI_Rate_18M                    18M
				SPI_Rate_1K						1K

      fistBit   SPI֡��ʽ   ֻ�������²���:
                SPI_MSB                         �ȷ���λ
                SPI_LSB                         �ȷ���λ
      subMode   SPI��ģʽ   ֻ�������²���:
                SPI_SubMode_0                   ��ģʽ0��CPOL=0, CPHA=0��
                SPI_SubMode_1                   ��ģʽ1��CPOL=0, CPHA=1��
                SPI_SubMode_2                   ��ģʽ2��CPOL=1, CPHA=0��
                SPI_SubMode_3                   ��ģʽ3��CPOL=1, CPHA=1��
         
����ֵ��TRUE    �������óɹ�  
        FALSH   ��������ʧ��
���ܣ�����SPI��ģʽ����  
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL ConfigSPIParam(unsigned int rate,unsigned int fistBit,unsigned int subMode,int UsbIndex);

/****************************************************************************
���ƣ�ConfigSPIParamSlave                               
������CSEn      SPI CS�ܽ� �Ƿ���Ч     ֻ�������²���:
                SPI_SLAVE_CSDS               0             �����CS�ܽ�   ʵʱ���պͷ�������
				SPI_SLAVE_CSEN               1             ֻ����CS�ܽ�Ϊ�͵�ƽ��ʱ��  ���ܷ��ͺͽ�������

      fistBit   SPI֡��ʽ   ֻ�������²���:
                SPI_MSB                         �ȷ���λ
                SPI_LSB                         �ȷ���λ
      subMode   SPI��ģʽ   ֻ�������²���:
                SPI_SubMode_0                   ��ģʽ0��CPOL=0, CPHA=0��
                SPI_SubMode_1                   ��ģʽ1��CPOL=0, CPHA=1��
                SPI_SubMode_2                   ��ģʽ2��CPOL=1, CPHA=0��
                SPI_SubMode_3                   ��ģʽ3��CPOL=1, CPHA=1��
         
����ֵ��TRUE    �������óɹ�  
        FALSH   ��������ʧ��
���ܣ�����SPI��ģʽ���� 
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL ConfigSPIParamSlave(unsigned int CSEn,unsigned int fistBit,unsigned int subMode,int UsbIndex);

/****************************************************************************
���ƣ�ConfigIICParam                           
������rate      IIC����     ֻ�������²���:
                IIC_Rate_100K
                IIC_Rate_200K
                IIC_Rate_400K
				IIC_Rate_10K
				IIC_Rate_1K
����ֵ��TRUE    �������óɹ�  
        FALSH   ��������ʧ��
���ܣ�����IIC��ģʽ����
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL ConfigIICParam(unsigned int rate,int UsbIndex);

/****************************************************************************
���ƣ�ConfigIICParamSlave                           
������addr      IIC��ģʽ�µ��豸��ַ  ֻ����0x00��0x7F֮��
����ֵ��TRUE    �������óɹ�  
        FALSH   ��������ʧ��
���ܣ�����IIC��ģʽ����
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL ConfigIICParamSlave(unsigned char addr,int UsbIndex);

/****************************************************************************
���ƣ�UARTSendData                           
������sendBuf   Ҫ�������ݵĻ���
      len       Ҫ�������ݵĳ��ȣ����4096��
����ֵ��TRUE    ���ͳɹ�  
        FALSH   ����ʧ��
���ܣ�UART��������
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL UARTSendData(unsigned char *sendBuf, unsigned int len,int UsbIndex);

/****************************************************************************
���ƣ�UARTRcvData                           
������rcvBuf    Ҫ�������ݵĻ���
      len       Ҫ�������ݵ���󳤶�
����ֵ������    ��ʾʵ�ʽ��յ��ֽ��������4096��  
        ����    ����ʧ��
���ܣ�UART��������
****************************************************************************/
USB2UARTSPIIICDLL_API int UARTRcvData(unsigned char *rcvBuf, unsigned int len,int UsbIndex);

/****************************************************************************
���ƣ�SPISendData                          
������startCS   ��������ǰCS�ܽŵĵ�ƽ
                0���͵�ƽ
                1���ߵ�ƽ
      endCS     �������ݺ�CS�ܽŵĵ�ƽ
                0���͵�ƽ
                1���ߵ�ƽ
      sndBuf    Ҫ�������ݵĻ���
      len       Ҫ�������ݵĳ��ȣ����4096��
����ֵ��TRUE    ���ͳɹ�  
        FALSH   ����ʧ��
���ܣ�SPI ��ģʽ�·������� 
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL SPISendData(unsigned int startCS, unsigned int endCS,unsigned char *sendBuf, unsigned int len,int UsbIndex);

/****************************************************************************
���ƣ�SPISendAndRecData                     
������startCS   ��������ǰCS�ܽŵĵ�ƽ
                0���͵�ƽ
                1���ߵ�ƽ
      endCS     �������ݺ�CS�ܽŵĵ�ƽ
                0���͵�ƽ
                1���ߵ�ƽ
      sndBuf    Ҫ�������ݵĻ���
	  rcvBuf    Ҫ�������ݵĻ���
      len       Ҫ�������ݵĳ��ȣ����4096��
����ֵ��TRUE    ���ͳɹ�  
        FALSH   ����ʧ��
���ܣ�SPI ��ģʽ�·��ͺͽ������� 
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL SPISendAndRecData(unsigned int startCS, unsigned int endCS,unsigned char *sendBuf,unsigned char *rcvBuf,unsigned int len,int UsbIndex);

/****************************************************************************
���ƣ�SPIRcvData                         
������startCS   ��������ǰCS�ܽŵĵ�ƽ
                0���͵�ƽ
                1���ߵ�ƽ
      endCS     �������ݺ�CS�ܽŵĵ�ƽ
                0���͵�ƽ
                1���ߵ�ƽ
      rcvBuf    Ҫ�������ݵĻ���
      len       Ҫ�������ݵĳ��ȣ����4096��
����ֵ��TRUE    ���ճɹ�  
        FALSH   ����ʧ��
���ܣ�SPI ��ģʽ�½������� 
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL SPIRcvData(unsigned int startCS, unsigned int endCS,unsigned char *rcvBuf, unsigned int len,int UsbIndex);


/****************************************************************************
���ƣ�SPISendDataSlave                          
������sndBuf    Ҫ�������ݵĻ���
      len       Ҫ�������ݵĳ��ȣ����4096��
����ֵ��TRUE    ���ͳɹ�  
        FALSH   ����ʧ��
���ܣ�spi��ģʽ�� ��Ҫ���͵�����Ԥװ����Ƭ��   
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL SPISendDataSlave(unsigned char *sendBuf, unsigned int len,int UsbIndex);

/****************************************************************************
���ƣ�SPIRcvDataSlave                           
������rcvBuf    Ҫ�������ݵĻ���
      len       Ҫ�������ݵ���󳤶�
����ֵ������    ��ʾʵ�ʽ��յ��ֽ��������4096��  
        ����    ����ʧ��
���ܣ�SPI��ģʽ�������ݽ�������
****************************************************************************/
USB2UARTSPIIICDLL_API int SPIRcvDataSlave(unsigned char *rcvBuf, unsigned int len,int UsbIndex);


/****************************************************************************
���ƣ�IICSendData                         
������strartBit ��������ǰ�Ƿ���IIC Start�ź�
                TRUE������
                FALSH��������
      stopBit   �������ݺ��Ƿ���IIC Stop�ź�
                TRUE������
                FALSH��������
      sendBuf   Ҫ�������ݵĻ���
      len       Ҫ�������ݵĳ��ȣ����4096��
����ֵ��TRUE    ���ͳɹ�  
        FALSH   ����ʧ��
���ܣ�IIC��������
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL IICSendData(BOOL strartBit,unsigned char *sendBuf, unsigned int len, BOOL stopBit,int UsbIndex);

/****************************************************************************
���ƣ�IICRcvData                         
������stopBit   �������ݺ��Ƿ���IIC Stop�ź�
                TRUE������
                FALSH��������
      rcvBuf    Ҫ�������ݵĻ���
      len       Ҫ�������ݵĳ��ȣ����4096��
����ֵ��TRUE    ���ճɹ�  
        FALSH   ����ʧ��
���ܣ�IIC��������
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL IICRcvData(unsigned char *rcvBuf, unsigned int len, BOOL stopBit,int UsbIndex);


/****************************************************************************
���ƣ�IICSendDataSlave                          
������sndBuf    Ҫ�������ݵĻ���
      len       Ҫ�������ݵĳ��ȣ����4096��
����ֵ��TRUE    ���ͳɹ�  
        FALSH   ����ʧ��
���ܣ�IIC��ģʽ�� ��Ҫ���͵�����Ԥװ����Ƭ��   
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL IICSendDataSlave(unsigned char *sendBuf, unsigned int len,int UsbIndex);

/****************************************************************************
���ƣ�IICRcvDataSlave                           
������rcvBuf    Ҫ�������ݵĻ���
      len       Ҫ�������ݵ���󳤶�
����ֵ������    ��ʾʵ�ʽ��յ��ֽ��������4096��  
        ����    ����ʧ��
���ܣ�IIC��ģʽ�������ݽ�������
****************************************************************************/
USB2UARTSPIIICDLL_API int IICRcvDataSlave(unsigned char *rcvBuf, unsigned int len,int UsbIndex);

/****************************************************************************
���ƣ�IOSetAndRead                        
������IONum     Ҫ���û��ȡ��IO�ںţ�0-7��
      IODir     ��IO�ķ���  
                1�����
                0������
      IOBit     ��IO�ڵĵ�ƽ��ֻ�������Ч��
                1���ߵ�ƽ
                0���͵�ƽ
����ֵ�����IO�ڱ����ó�����
        ����TRUE����ʾ��IO�ڶ�ȡ���ߵ�ƽ 
        ����FALSH����ʾ��IO�ڶ�ȡ���͵�ƽ
���ܣ����úͶ�ȡIO�ڵĵ�ƽ
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL IOSetAndRead(unsigned int IONum, unsigned int IODir, unsigned int IOBit,int UsbIndex);

/****************************************************************************
���ƣ�SPISendDataFast                         
������startCS   ��������ǰCS�ܽŵĵ�ƽ
                0���͵�ƽ
                1���ߵ�ƽ
      endCS     �������ݺ�CS�ܽŵĵ�ƽ
                0���͵�ƽ
                1���ߵ�ƽ
      sndBuf    Ҫ�������ݵĻ���
      len       Ҫ�������ݵĳ��ȣ����4096��
����ֵ��TRUE    ���ͳɹ�  
        FALSH   ����ʧ��
���ܣ�SPI ��ģʽ�·������� ����ģʽ ��ģʽ�²�֧��1K����
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL SPISendDataFast(unsigned int startCS, unsigned int endCS,unsigned char *sendBuf, unsigned int len,int UsbIndex);
/****************************************************************************
���ƣ�SPIRcvDataFast                        
������startCS   ��������ǰCS�ܽŵĵ�ƽ
                0���͵�ƽ
                1���ߵ�ƽ
      endCS     �������ݺ�CS�ܽŵĵ�ƽ
                0���͵�ƽ
                1���ߵ�ƽ
      rcvBuf    Ҫ�������ݵĻ���
      len       Ҫ�������ݵĳ��ȣ����4096��
����ֵ��TRUE    ���ճɹ�  
        FALSH   ����ʧ��
���ܣ�SPI ��ģʽ�½�������  ����ģʽ  ��ģʽ�²�֧��1K����
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL SPIRcvDataFast(unsigned int startCS, unsigned int endCS,unsigned char *rcvBuf, unsigned int len,int UsbIndex);
/****************************************************************************
���ƣ�SPISendDataFast                         
������sndBuf    Ҫ�������ݵĻ���
      len       Ҫ�������ݵĳ��ȣ����4096��
����ֵ��TRUE    ���ͳɹ�  
        FALSH   ����ʧ��
���ܣ�SPI ��ģʽ�·������� ��ģʽ��ÿ����һ���ֽ�CS�ܽŵ�ƽ����һ�� ��ģʽ�²�֧��1K����
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL SPISendDataSpecial(unsigned char *sendBuf, unsigned int len,int UsbIndex);
/****************************************************************************
���ƣ�SPIRcvDataFast                        
������rcvBuf    Ҫ�������ݵĻ���
      len       Ҫ�������ݵĳ��ȣ����4096��
����ֵ��TRUE    ���ճɹ�  
        FALSH   ����ʧ��
���ܣ�SPI ��ģʽ�½�������  ��ģʽ��ÿ����һ���ֽ�CS�ܽŵ�ƽ����һ��  ��ģʽ�²�֧��1K����
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL SPIRcvDataSpecial(unsigned char *rcvBuf, unsigned int len,int UsbIndex);

#endif






