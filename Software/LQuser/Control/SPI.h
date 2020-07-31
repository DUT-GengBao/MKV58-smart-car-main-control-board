/*
 * SPI.h
 *
 *  Created on: Feb 20, 2018
 *      Author: ZBT
 */

#ifndef USER_DRIVER_SPI_SPI_H_
#define USER_DRIVER_SPI_SPI_H_

//定义主从机模式
typedef enum
{
    MASTER,    //主机模式
    SLAVE      //主机模式
} SPI_CFG;

//定义SPI模块号
typedef enum
{
	kSPI0,
	kSPI1,
	kSPI2
} SPIn_e;

//定义SPI模块片选号
typedef enum
{
    SPIn_PCS0 = 1 << 0,
    SPIn_PCS1 = 1 << 1,
    SPIn_PCS2 = 1 << 2,
    SPIn_PCS3 = 1 << 3,
    SPIn_PCS4 = 1 << 4,
    SPIn_PCS5 = 1 << 5,
} SPIn_PCSn_e;

#define SPI0_SCK    PTE17       //PTE17、PTA15、PTC5 、PTD1
#define SPI0_SOUT   PTE18       //PTE18、PTA16、PTC6 、PTD2
#define SPI0_SIN    PTE19       //PTE19、PTA17、PTC7 、PTD3

#define SPI0_PCS0   PTE16       //PTE16、PTA14、PTC0 、PTC4 、PTD0
#define SPI0_PCS1   NULL        //PTC3 、PTD4
#define SPI0_PCS2   NULL        //PTC2 、PTD5
#define SPI0_PCS3   NULL        //PTC1 、PTD6
#define SPI0_PCS4   NULL        //PTC0
#define SPI0_PCS5   NULL        //PTB23


#define SPI1_SCK    PTE2        //PTE2 、PTB11、PTD5
#define SPI1_SOUT   PTE1        //PTE1 、PTB16、PTD6
#define SPI1_SIN    PTE3        //PTE3 、PTB17、PTD7

#define SPI1_PCS0   PTE4        //PTE4 、PTB10、PTD4
#define SPI1_PCS1   NULL        //PTE0 、PTB9
#define SPI1_PCS2   NULL        //PTE5
#define SPI1_PCS3   NULL        //PTE6


#define SPI2_SCK    PTD12       //PTB21、PTD12
#define SPI2_SOUT   PTD13       //PTB22、PTD13
#define SPI2_SIN    PTD14       //PTB23、PTD14
#define SPI2_PCS0   PTD11       //PTB20、PTD11
#define SPI2_PCS1   NULL        //PTD15、PTC12

uint32 spi_init       (SPIn_e, SPIn_PCSn_e , SPI_CFG,uint32 baud);                                        //SPI初始化，设置模式

//主机接收发送函数
void spi_mosi       (SPIn_e spin, SPIn_PCSn_e pcs,                              uint8 *modata, uint8 *midata,               uint32 len);    //SPI发送接收函数,发送databuff数据，并把接收到的数据存放在databuff里(注意，会覆盖原来的databuff)
void spi_mosi_cmd   (SPIn_e spin, SPIn_PCSn_e pcs, uint8 *mocmd , uint8 *micmd , uint8 *modata, uint8 *midata, uint32 cmdlen , uint32 len); //SPI发送接收函数,与spi_mosi相比，多了先发送cmd 缓冲区的步骤，即分开两部分发送
void  port_init_NRF(PTXn_e ptxn, uint32 cfg );
void ASSERT(unsigned char x,unsigned char* Log);
#endif /* USER_DRIVER_SPI_SPI_H_ */
