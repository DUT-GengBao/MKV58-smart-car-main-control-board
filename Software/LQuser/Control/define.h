/*
 * define.h
 *
 *  Created on: Feb 14, 2018
 *      Author: ZBT
 */

#ifndef DEFINE_H_
#define DEFINE_H_

//Motor Driver
#define MOTOR_EN_PORT           PORTE,19
#define MOTOR_LF_DIR_PORT       PORTA,9       //Left Front Motor Dir Pin
#define MOTOR_LF_FTM            FTM0,CH4
#define MOTOR_RF_DIR_PORT       PORTA,8
#define MOTOR_RF_FTM            FTM0,CH3
#define MOTOR_LB_DIR_PORT       PORTE,27
#define MOTOR_LB_FTM            FTM0,CH0
#define MOTOR_RB_DIR_PORT       PORTE,28      //Right Back Motor Dir Pin
#define MOTOR_RB_FTM            FTM0,CH1

#define MOTOR_EN        PTE19_OUT
#define MOTOR_LF_DIR    PTA9_OUT
#define MOTOR_RF_DIR    PTA8_OUT
#define MOTOR_LB_DIR    PTE27_OUT
#define MOTOR_RB_DIR    PTE28_OUT

//Decoder
#define FRONT_FTM       FTM1
#define BACK_FTM        FTM2
#define FRONT_L_SEL_PORT        PORTA,17
#define FRONT_R_SEL_PORT        PORTA,16
#define BACK_L_SEL_PORT         PORTE,20
#define BACK_R_SEL_PORT         PORTE,29

#define FRONT_L_SEL             PTA17_OUT
#define FRONT_R_SEL             PTA16_OUT
#define BACK_L_SEL              PTE20_OUT
#define BACK_R_SEL              PTE29_OUT

//Buzzer
#define BUZ_PORT        PORTE,21
#define BUZ             PTE21_OUT

//Button
#define BUT1_PORT       PORTE,26
#define BUT1            PTE26_IN

//Camera
#define Camera_CNST     85
#define PCLK_PORT       PORTC,5
#define VSYNC_PORT      PORTC,6
#define HSYNC_PORT      PORTC,7
#define VSYNC_IRQ       PORTC_IRQn
#define VSYNC_ISFR      PORTC_ISFR

//Camera-SCCB
#define SCCB_SDA_PORT   PORTC,0
#define SCCB_SCL_PORT   PORTC,1
#define SCL_H()         PTC1_OUT = 1
#define SCL_L()         PTC1_OUT = 0
#define	SCL_DDR_OUT() 	DDRC1 = 1
#define	SCL_DDR_IN() 	DDRC1 = 0
#define SDA_H()         PTC0_OUT = 1
#define SDA_L()         PTC0_OUT = 0
//#define SDA_IN()      	PTC0_IN
#define SDA_DDR_OUT()	DDRC0 = 1
#define SDA_DDR_IN()	DDRC0 = 0

/*
#define LED1_PORT       PORTC,0
#define LED2_PORT       PORTD,15
#define LED3_PORT       PORTE,29
#define LED4_PORT       PORTA,17

#define LED1_GPIO       PC
#define LED2_GPIO       PD
#define LED3_GPIO       PE
#define LED4_GPIO       PA
  
#define LED1            PTC0_OUT
#define LED2            PTD15_OUT
#define LED3            PTE29_OUT
#define LED4            PTA17_OUT
   */
//UART
#define DEBUG_UART      UART5
#define DEBUG_UART_IRQ  UART5_RX_TX_IRQn
#define DEBUG_UART_BAUDRATE     115200
#define UART_DMA_CHN    16
#define UART_DMA_SOURCE 57

//LCD
#define LCD_RST_PORT    GPIOE,12
#define LCD_RS_PORT     GPIOE,16
#define LCD_SDA_PORT    GPIOE,18
#define LCD_SCL_PORT    GPIOE,17

#define LCD_SPIn        kSPI0
#define LCD_OTH_PCSn    SPIn_PCS1
#define LCD_DC_PCSn     SPIn_PCS0

#define rs      PTE16_OUT
#define sda     PTE18_OUT
#define scl     PTE17_OUT
#define reset   PTE12_OUT

//Switch
#define SW1_PORT        PORTA,15
#define SW2_PORT        PORTA,14
#define SW3_PORT        PORTA,11
#define SW4_PORT        PORTA,10

#define SW1             PTA15_IN
#define SW2             PTA14_IN
#define SW3             PTA11_IN
#define SW4             PTA10_IN

//nRF (Not Used But need definition)
#define NRF_LINKADDRESS {0x74, 0x18, 0x52, 0x96, 0x30}
#define NRF_SPIn        kSPI0
#define NRF_PCSn        SPIn_PCS0
#define NRF_CE_PORT     PORTA,25
#define NRF_IRQ_PORT    PORTA,24
#define NRF_CE          PTA25_OUT
#define NRF_IRQ         PTA24_OUT

#endif /* DEFINE_H_ */
