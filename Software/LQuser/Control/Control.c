#include "include.h"
#include "math.h"
int LF=0,LB=0,Dir=17;
unsigned int angle=2300;
u8 SpeedMeasureFlag=0;
//测速
char speed[6];
int hongwaidat[16];

void Speed_meauser(void)
{
  LF=-FTM_AB_Get(FRONT_FTM);
  LB=-FTM_AB_Get(BACK_FTM);  
}

//测速初始化
void Measure_Init(void)
{ 
  GPIO_Init (GPIOA, 12,GPI,0);//测速ftm脚初始化
  GPIO_Init (GPIOA, 13,GPI,0);
  GPIO_Init (GPIOE, 22,GPI,1);
  GPIO_Init (GPIOE, 23,GPI,1); 
  FTM_AB_Init(FTM2);//测速ftm+测速引脚初始化
  FTM_AB_Init(FTM1);
}

void Servo_Init(void)
{
  GPIO_Init (GPIOC, 1,GPO,0); 
  ServoFTM_PWM_Init(FTM0,FTM_CH0,149410,1680);//PTC1    舵机方向左边大右边小
}

void Servo_Ctrl(u32 duty)
{      
  /*归一化为460--2290*/
    if((duty<460))
    { 
      FTM_CnV_REG(FTM0, FTM_CH0) =460;  //舵机，用PTC1
    }
    else if(duty>2290)
    {
      FTM_CnV_REG(FTM0, FTM_CH0) = 2290;
    }
    else 
      FTM_CnV_REG(FTM0, FTM_CH0) = duty;
}

void hongwai_Init(void)
{
  /*----------0--------------*/
  GPIO_Init(GPIOE,18,GPI,1);
  EXTI_Init(GPIOE,18,rising_down);
  /*----------1--------------*/
  GPIO_Init(GPIOE,4,GPI,1);
  EXTI_Init(GPIOE,4,rising_down);
  /*----------2--------------*/
  GPIO_Init(GPIOE,7,GPI,1);
  EXTI_Init(GPIOE,7,rising_down);
  /*----------3--------------*/
  GPIO_Init(GPIOE,6,GPI,1);
  EXTI_Init(GPIOE,6,rising_down);
  /*----------4--------------*/
  GPIO_Init(GPIOE,5,GPI,1);
  EXTI_Init(GPIOE,5,rising_down);
  /*----------5--------------*/
  GPIO_Init(GPIOE,3,GPI,1);
  EXTI_Init(GPIOE,3,rising_down);
  /*----------6--------------*/
  GPIO_Init(GPIOE,0,GPI,1);
  EXTI_Init(GPIOE,0,rising_down);
  /*----------7--------------*/
  GPIO_Init(GPIOE,1,GPI,1);
  EXTI_Init(GPIOE,1,rising_down);
  /*----------8--------------*/
  GPIO_Init(GPIOE,2,GPI,1);
  EXTI_Init(GPIOD,2,rising_down);
  /*----------9--------------*/
  GPIO_Init(GPIOE,11,GPI,1);
  EXTI_Init(GPIOE,11,rising_down);
  /*----------10--------------*/
  GPIO_Init(GPIOE,9,GPI,1);
  EXTI_Init(GPIOE,9,rising_down);
  /*----------11--------------*/
  GPIO_Init(GPIOE,8,GPI,1);
  EXTI_Init(GPIOE,8,rising_down);
  /*----------12--------------*/
  GPIO_Init(GPIOE,10,GPI,1);
  EXTI_Init(GPIOE,10,rising_down);
  /*----------13--------------*/
  GPIO_Init(GPIOE,17,GPI,1);
  EXTI_Init(GPIOE,17,rising_down);
  /*----------14--------------*/
  GPIO_Init(GPIOE,16,GPI,1);
  EXTI_Init(GPIOE,16,rising_down);
  /*----------15--------------*/
  GPIO_Init(GPIOE,12,GPI,1);
  EXTI_Init(GPIOE,12,rising_down); 
}
void hongwaiDat(void)
{
  int i=0;
  for(i=0;i<16;i++)
  {
  if(hongwaidat[i]==3)
  {
  Dir=i;
  break;
  }
  for(i=0;i<16;i++)hongwaidat[i]=0;
  
  }



}