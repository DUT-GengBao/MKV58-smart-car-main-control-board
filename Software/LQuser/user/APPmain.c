#include "include.h" 
int i=0;
u32 se=1600;
void main(void)
{
  PLL_Init(PLL235);         	//设置内核及总线频率等
  DisableInterrupts
    Servo_Init();
    PIT_Init(PIT1,20);		//计时1初始化
  UART_Init(UART_0,460800); 	//串口初始化
  UART_Irq_En(UART_0);          //串口中断开启
  //UART_Init(UART_5,115200); 	//串口初始化
  //UART_Irq_En(UART_5);          //串口中断开启
  Measure_Init();
  KEY_Init();
  EnableInterrupts
    GPIO_Init(GPIOE,29,GPO,0);//BUT_MCU1_1
    GPIO_Init(GPIOE,30,GPO,0);//BUT_MCU1_2
    GPIO_Init(GPIOE,13,GPO,0);//BUT_MCU1_3
    char speed[6];
  while(1)
  { 
   if(SpeedMeasureFlag)
    {
       int2str(LF, speed);
      cJSON * pJsonRoot = NULL;
  pJsonRoot = cJSON_CreateObject();
  cJSON_AddStringToObject(pJsonRoot, "LF", speed);
  for(i=0;i<6;i++){speed[i]=0;}
  int2str(LB, speed);
 cJSON_AddStringToObject(pJsonRoot, "LB", speed);
 for(i=0;i<6;i++){speed[i]=0;}
 int2str(Dir, speed);
 cJSON_AddStringToObject(pJsonRoot, "Dir", speed);
   char * p= cJSON_Print(pJsonRoot);
    UART_Put_Str(UART_0,p);
   cJSON_Delete(pJsonRoot);
   free(p);
      SpeedMeasureFlag=0;  
    } 
  }
}
