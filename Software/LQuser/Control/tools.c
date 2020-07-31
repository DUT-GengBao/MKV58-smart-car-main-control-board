/*
* tools.c
*
*  Created on: Feb 17, 2018
*      Author: ZBT
*/

#include "include.h"

uart_edma_handle_t uart_eDMA_Handler;
uart_transfer_t uart_tx_data;
edma_handle_t g_uartTxEdmaHandle;

struct
{
  unsigned char a11;    //×Ö½Ú¶ÔÆë
  unsigned char b11;
  unsigned char Header1[2];
  float data[8];
  unsigned char Header2[2];
  unsigned char a21;
  unsigned char b21;
}Uart_data;

void UART_eDMA_init(void)
{
  uart_config_t uartConfig;
  edma_config_t config;
  UART_GetDefaultConfig(&uartConfig);
  ////////////////
  SIM_SCGC1 |= SIM_SCGC1_UART5_MASK;
  PORT_PCR_REG(PORTD,9)=PORT_PCR_MUX(3);
  PORT_PCR_REG(PORTD,8)=PORT_PCR_MUX(3);
  UART_C2_REG(DEBUG_UART)|=UART_C2_RIE_MASK;
  NVIC_EnableIRQ(DEBUG_UART_IRQ);
  ///////////////
  uartConfig.baudRate_Bps = DEBUG_UART_BAUDRATE;
  uartConfig.enableTx = true;
  uartConfig.enableRx = false;
  UART_Init(DEBUG_UART, &uartConfig, CLOCK_GetFreq(kCLOCK_FastPeriphClk));
  DMAMUX_Init(DMAMUX);
  DMAMUX_SetSource(DMAMUX, UART_DMA_CHN, UART_DMA_SOURCE);
  DMAMUX_EnableChannel(DMAMUX, UART_DMA_CHN);
  EDMA_GetDefaultConfig(&config);
  EDMA_Init(DMA0, &config);
  EDMA_CreateHandle(&g_uartTxEdmaHandle, DMA0, UART_DMA_CHN);
  UART_TransferCreateHandleEDMA(DEBUG_UART, &uart_eDMA_Handler, NULL, NULL,
                                &g_uartTxEdmaHandle, NULL);
}

void Uart_make_data()
{
  Uart_data.Header1[0] = 0x03;
  Uart_data.Header1[1] = 0xFC;
  Uart_data.Header2[0] = 0xFC;
  Uart_data.Header2[1] = 0x03;
  Uart_data.data[0] = 0;
  Uart_data.data[1] = 0;
  Uart_data.data[2] = 0;
  Uart_data.data[3] = 0;
}

void UART_eDMA_datasend(void)
{
  if(EDMA_GetChannelStatusFlags(DMA0,UART_DMA_CHN)==0)
  {
    Uart_make_data();
    uart_tx_data.data = (void *)&Uart_data;
    uart_tx_data.dataSize = sizeof(Uart_data);
    UART_SendEDMA(DEBUG_UART, &uart_eDMA_Handler, &uart_tx_data);
  }
}

void Buz_ring_ms(int ms)
{
  BUZ = 1;
  delayms(ms);
  BUZ = 0;
}

void Buz_handler(void)
{
  if(Buz_flag)
  {
    BUZ = 1;
    Buz_flag--;
  }
  else
    BUZ = 0;
}

void Wait_click(void)
{
  while(1)
  {
    if(!BUT1)
    {
      delayms(10);
      if(!BUT1)
      {
        while(!BUT1);
        return;
      }
    }
  }
}
