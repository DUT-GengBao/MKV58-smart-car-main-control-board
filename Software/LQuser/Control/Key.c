#include "include.h"

void Key_Init()
{
    GPIO_Init(GPIOD,14,GPO,1);//BUT_MCU1_1
    GPIO_Init(GPIOD,13,GPO,1);//BUT_MCU1_2
    GPIO_Init(GPIOD,12,GPO,1);//BUT_MCU1_3
    EXTI_Init(GPIOD,14,falling_up);//BUT_MCU2_1
    EXTI_Init(GPIOD,13,falling_up);//BUT_MCU2_2
    EXTI_Init(GPIOD,12,falling_up);//BUT_MCU2_3
    //   EXTI_Init(GPIOD,11,falling_up);//BUT_MCU2_4
    
    GPIO_Init(GPIOE,29,GPO,0);//BUT_MCU1_1
    GPIO_Init(GPIOE,30,GPO,0);//BUT_MCU1_2
    GPIO_Init(GPIOE,13,GPO,0);//BUT_MCU1_3
    
}

void PORTD_IRQHandler(void)
{
    if((PORTD_ISFR & (1<<14)))
    {
        PORTD_ISFR |= (1<<14);
        /* 用户自行添加中断内程序 */
        //angle-=10;
        //if(angle<=0)angle=0;
        //Servo_Ctrl(angle);
        //time_delay_ms(8);
       BUT_MCU1_1 = 1;
         LPTMR_delay_us(1000);
        BUT_MCU1_1 = 0;
    }
    if((PORTD_ISFR & (1<<13)))
    {
        PORTD_ISFR |= (1<<13);
        /* 用户自行添加中断内程序 */
        BUT_MCU1_2 = 1;
         LPTMR_delay_us(1000);
        BUT_MCU1_2 = 0;
    } 
    if((PORTD_ISFR & (1<<12)))
    {
        PORTD_ISFR |= (1<<12);
        /* 用户自行添加中断内程序 */
        BUT_MCU1_3 = 1;
         LPTMR_delay_us(1000);
        BUT_MCU1_3 = 0;
    } 
    if((PORTD_ISFR & (1<<11)))
    {
        PORTD_ISFR |= (1<<11);
        /* 用户自行添加中断内程序 */
    }
}