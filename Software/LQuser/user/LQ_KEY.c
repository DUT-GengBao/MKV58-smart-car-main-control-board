/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】龙邱KV58F24智能车VD母板
【编    写】CHIUSIR
【E-mail  】chiusir@163.com
【软件版本】V1.0
【最后更新】2017年12月15日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
------------------------------------------------
【dev.env.】IAR7.80.4
【Target  】MKV58F1M0VLQ24
【Crystal 】 50.000Mhz
【busclock】137.500MHz
【pllclock】275.000MHz
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "include.h"

void KEY_Init(void)
{
  //  GPIO_Init(GPIOE,26,GPI,1);
  //  EXTI_Init(GPIOE,26,rising_down);
  GPIO_Init(GPIOD,14,GPI,1);
  EXTI_Init(GPIOD,14,rising_down);
  GPIO_Init(GPIOD,13,GPI,1);
  EXTI_Init(GPIOD,13,rising_down);
  GPIO_Init(GPIOD,12,GPI,1);
  EXTI_Init(GPIOD,12,rising_down);
  GPIO_Init(GPIOD,11,GPI,1);
  EXTI_Init(GPIOD,11,rising_down);
  //GPIO_Init(GPIOB,21,GPI,1);
  //GPIO_Init(GPIOB,22,GPI,1);   
}

u8 KEY_Read(KEYn_e keyno)
{
  switch(keyno) 
  {
  case KEY0:
    return GPIO_Get(PTE26);
    break;
    
  case KEY1:
    return GPIO_Get(PTB21);
    break;
    
  case KEY2:
    return GPIO_Get(PTB22);
    break;
  default:
    return 0XFF;    
  }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【作  者】CHIUSIR
【功能说明】测试按键及OLED显示
【软件版本】V1.0
【最后更新】2017年11月24日 
【函数名】
【返回值】无
【参数值】无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
//void Test_GPIO_KEY(void)
//{ 
//  while (1)
//  {  
//    //测试按键      
//    if(KEY_Read(KEY0)==0)
//      LCD_P6x8Str(13,3,(uint8_t*)"KEY0 Pressed!   ");
//    else LCD_P6x8Str(13,3,(uint8_t*)"KEY0 NO Pressed!");
//    if(KEY_Read(KEY1)==0)
//      LCD_P6x8Str(13,5,(uint8_t*)"KEY1 Pressed!   ");
//    else LCD_P6x8Str(13,5,(uint8_t*)"KEY1 NO Pressed!");
//    if(KEY_Read(KEY2)==0)
//      LCD_P6x8Str(13,7,(uint8_t*)"KEY2 Pressed!   ");
//    else LCD_P6x8Str(13,7,(uint8_t*)"KEY2 NO Pressed!");
//    
//    //LED闪烁
//    LED_Ctrl(LED1, RVS);     
//    
//    //延时
//    time_delay_ms(50);
//  }
//}

void PORTE_IRQHandler(void)
{
  if((PORTE_ISFR & (1<<18)))//0
  {
    PORTE_ISFR |= (1<<18);
    /* 中断内程序 */
    hongwaidat[0]++;
  }
  else  if((PORTE_ISFR & (1<<4)))//1
  {
    PORTE_ISFR |= (1<<4);
    /* 中断内程序 */
    hongwaidat[1]++;
  }
  else  if((PORTE_ISFR & (1<<7)))//2
  {
    PORTE_ISFR |= (1<<7);
    /* 中断内程序 */
    hongwaidat[2]++;
  }
  else  if((PORTE_ISFR & (1<<6)))//3
  {
    PORTE_ISFR |= (1<<6);
    /* 中断内程序 */
    hongwaidat[3]++;
  }
  else  if((PORTE_ISFR & (1<<5)))//4
  {
    PORTE_ISFR |= (1<<5);
    /* 中断内程序 */
    hongwaidat[4]++;
  }
  else  if((PORTE_ISFR & (1<<3)))//5
  {
    PORTE_ISFR |= (1<<3);
    /* 中断内程序 */
    hongwaidat[5]++;
  }
  else  if((PORTE_ISFR & (1<<0)))//6
  {
    PORTE_ISFR |= (1<<0);
    /* 中断内程序 */
    hongwaidat[6]++;
  }
  else  if((PORTE_ISFR & (1<<1)))//7
  {
    PORTE_ISFR |= (1<<1);
    /* 中断内程序 */
    hongwaidat[7]++;
  }
  else  if((PORTE_ISFR & (1<<2)))//8
  {
    PORTE_ISFR |= (1<<2);
    /* 中断内程序 */
    hongwaidat[8]++;
  }
  else  if((PORTE_ISFR & (1<<11)))//9
  {
    PORTE_ISFR |= (1<<11);
    /* 中断内程序 */
    hongwaidat[9]++;
  }
  else  if((PORTE_ISFR & (1<<9)))//10
  {
    PORTE_ISFR |= (1<<9);
    /* 中断内程序 */
    hongwaidat[10]++;
  }
  else  if((PORTE_ISFR & (1<<8)))//11
  {
    PORTE_ISFR |= (1<<8);
    /* 中断内程序 */
    hongwaidat[11]++;
  }
  else  if((PORTE_ISFR & (1<<10)))//12
  {
    PORTE_ISFR |= (1<<10);
    /* 中断内程序 */
    hongwaidat[12]++;
  }
  else  if((PORTE_ISFR & (1<<17)))//13
  {
    PORTE_ISFR |= (1<<17);
    /* 中断内程序 */
    hongwaidat[13]++;
  }
  else  if((PORTE_ISFR & (1<<16)))//14
  {
    PORTE_ISFR |= (1<<16);
    /* 中断内程序 */
    hongwaidat[14]++;
  }
  else  if((PORTE_ISFR & (1<<12)))//15
  {
    PORTE_ISFR |= (1<<12);
    /* 中断内程序 */
    hongwaidat[15]++;
  }
  else  
  {
    PORTE_ISFR ;
    /* 中断内程序 */
  }
  
}
void PORTD_IRQHandler(void)
{
  if((PORTD_ISFR & (1<<14)))
  {
    PORTD_ISFR |= (1<<14);
    /* 用户自行添加中断内程序 */
    LPTMR_delay_ms(1);
    if(PTD14_IN)
    {
      BUT_MCU1_1 = 1;
      LPTMR_delay_ms(1);
      BUT_MCU1_1 = 0;
    }
  }
  if((PORTD_ISFR & (1<<13)))
  {
    PORTD_ISFR |= (1<<13);
    /* 用户自行添加中断内程序 */
    LPTMR_delay_ms(1);
    if(PTD13_IN)
    {
      BUT_MCU1_2 = 1;
      LPTMR_delay_ms(1);
      BUT_MCU1_2 = 0;
    }
  }
  if((PORTD_ISFR & (1<<12)))
  {
    PORTD_ISFR |= (1<<12);
    /* 用户自行添加中断内程序 */
    
  }
  if((PORTD_ISFR & (1<<11)))
  {
    PORTD_ISFR |= (1<<11);
    /* 用户自行添加中断内程序 */
    LPTMR_delay_ms(1);
    if(PTD11_IN)
    {
      BUT_MCU1_3= 1;
      LPTMR_delay_ms(1);
      BUT_MCU1_3 = 0;
    }
  }
  PORTD_ISFR ;
}
//
//void PORTE_IRQHandler(void)
//{
//  int n;
//  n=26;
//  if((PORTE_ISFR & (1<<n)))
//  {
//  PORTE_ISFR |= (1<<n);
//  /* 用户自行添加中断内程序 */
//  }
//  
//}
void Test_GPIO_EXINT(void)
{  
  EXTI_Init(GPIOB,22,rising_down);   //K2
  EXTI_Init(GPIOB,21,falling_up);    //K1中断  
  EXTI_Init(GPIOB,20,falling_up);    //K0中断 
  for(;;)
  {  
    //systick中断延时
    time_delay_ms(5);
  }
}