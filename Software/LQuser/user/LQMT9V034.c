///*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
//��ƽ    ̨������KV58F24���ܳ�VDĸ��
//����    д��CHIUSIR
//��E-mail  ��chiusir@163.com
//������汾��V1.0
//�������¡�2018��3��28��
//�������Ϣ�ο����е�ַ��
//����    վ��http://www.lqist.cn
//���Ա����̡�http://shop36265907.taobao.com
//------------------------------------------------
//��dev.env.��IAR7.80.4������
//��Target  ��MKV58F1M0VLQ24
//��Crystal �� 50.000Mhz
//��busclock��137.500MHz
//��pllclock��275.000MHz
//QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
//#include "include.h"
//extern int8 frame;
// extern int cam;
// extern int lcd;
//int lineflag=0;
//u8 Image_Data[IMAGEH][IMAGEW];      //ͼ��ԭʼ���ݴ��
//u8 Image_Use[LCDH][LCDW]={0,}; //ѹ����֮�����ڴ����Ļ��ʾ����
////u16 Pixle[LCDH][LCDW];              //��ֵ��������OLED��ʾ������
////uint8_t Threshold;                  //OSTU��򷨼����ͼ����ֵ
////u8 GaveValue;//ƽ��ֵ��ֵ
//u8  Line_Cont=0;          //�м���
//u8  Field_Over_Flag=0;    //����ʶ
////uint8_t ThresholdV=100;
////
////
////int OFFSET0=0;      //��Զ������������ֵ�ۺ�ƫ����
////int OFFSET1=0;      //�ڶ���
////int OFFSET2=0;      //�����������
////int TXV=0;          //���ε���߶ȣ��Ҹ߶�
///*************************************************************** 
//* 
//* �������ƣ�void Get_Use_Image(void)
//* ����˵������DMA�洢��ȡ��ͼ�񲢽��о�ֵ�˲�
//* ����˵���� 
//* �������أ�void 
//* �޸�ʱ�䣺2019��4��3�� 
//* �� ע�� 
//***************************************************************/  
//void Get_Use_Image(void)
//{
//  int i = 0,j = 0; 
//  for(i = 0; i  < LCDH; i++)  //120�� 
//  {
//    for(j = 0;j < LCDW; j++)  //188��
//    {        
//      if(i<118&&j<186)Image_Use[i][j]= (Image_Data[i][j]+Image_Data[i+1][j]+Image_Data[i][j+1]+Image_Data[i+1][j+1])/4;
//      else Image_Use[i][j]= (Image_Data[i][j]+Image_Data[i-1][j]+Image_Data[i][j-1]+Image_Data[i-1][j-1])/4;
//    }       
//  }  
//}
///*************************************************************** 
//* 
//* �������ƣ�void BinaryImage(uint8_t tmImage[IMAGEH][IMAGEW]) 
//* ����˵����ͼ�����ݶ�ֵ�� 
//* ����˵���� 
//* �������أ�void 
//* �޸�ʱ�䣺2018��3��27�� 
//* �� ע�� 
//***************************************************************/ 
//void BinaryImage(u8 Image_Use[LCDH][LCDW],uint8_t ThresholdV) 
//{ 
//  int i = 0, j = 0; 
//  for(i = 0;i <LCDH;i++) 
//  { 
//    for(j = 0; j<LCDW;j++) 
//    { 
//      if(Image_Use[i][j] >=ThresholdV) 
//      { 
//       Image_Use[i][j] = 1; 
//      } 
//      else 
//      { 
//       Image_Use[i][j] = 0; 
//      } 
//    } 
//  } 
//} 
///*************************************************************** 
//* 
//* ��������void Draw_erzhi_Picture(unsigned int x,unsigned int y)
//* ����˵������ʾ��ֵ��ͼ��LCD
//* ����˵����X ˮƽ���� Y ��ֱ����
//* �������أ�void 
//* �޸�ʱ�䣺2019��4��1�� 
//* �� ע�� 
//***************************************************************/ 
//void Draw_erzhi_Picture(unsigned int x,unsigned int y)
//{
//  u8 i = 0, j = 0;
//  for(i=0;i<120;i++)
//  {
//    for(j=0;j<188;j++)  //����
//    { 
//      if(Image_Use[i][j]==1){
//        Gui_DrawPoint(i+x,240-j-y,COLOR_WHITE);
//      }
//      else {
//        Gui_DrawPoint(i+x,240-j-y,COLOR_BLACK);
//      }
//    }
//  }  
//}
/////*************************************************************** 
////* 
////* ��������void Draw_erzhi_Picture(unsigned int x,unsigned int y)
////* ����˵������ʾ�Ҷ�ͼ��LCD
////* ����˵����X ˮƽ���� Y ��ֱ����
////* �������أ�void 
////* �޸�ʱ�䣺2019��4��1�� 
////* �� ע�� 
////***************************************************************/ 
////void Draw_erzhi_Picture(unsigned int x,unsigned int y)
////{
////  u8 i = 0, j = 0;
////  for(i=0;i<120;i++)
////  {
////    for(j=0;j<188;j++)  //����
////    { 
////      if(Image_Use[i][j]==1){
////        Gui_DrawPoint(i+x,240-j-y,COLOR_WHITE);
////      }
////      else {
////        Gui_DrawPoint(i+x,240-j-y,COLOR_BLACK);
////      }
////    }
////  }  
////}
///*************************************************************** 
//* 
//* ��������void PictureProcessing(void)
//* ����˵����ͼ������
//* ����˵����
//* �������أ�void 
//* �޸�ʱ�䣺2019��4��1�� 
//* �� ע�� 
//***************************************************************/ 
//
/////*************************************************************** ��򷨷�
////* 
////* �������ƣ�uint8_t GetOSTU(uint8_t tmImage[IMAGEH][IMAGEW]) 
////* ����˵��������ֵ��С 
////* ����˵���� 
////* �������أ���ֵ��С 
////* �޸�ʱ�䣺2018��3��27�� 
////* �� ע�� 
////�ο���https://blog.csdn.net/zyzhangyue/article/details/45841255
////https://www.cnblogs.com/moon1992/p/5092726.html
////https://www.cnblogs.com/zhonghuasong/p/7250540.html     
////Ostu������������������ͨ��ͳ������ͼ���ֱ��ͼ������ʵ��ȫ����ֵT���Զ�ѡȡ�����㷨����Ϊ��
////1) �ȼ���ͼ���ֱ��ͼ������ͼ�����е����ص㰴��0~255��256��bin��ͳ������ÿ��bin�����ص�����
////2) ��һ��ֱ��ͼ��Ҳ����ÿ��bin�����ص����������ܵ����ص�
////3) i��ʾ�������ֵ��Ҳ��һ���Ҷȼ�����0��ʼ����
////4) ͨ����һ����ֱ��ͼ��ͳ��0~i �Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ���ǰ������) ��ռ����ͼ��ı���w0����ͳ��ǰ�����ص�ƽ���Ҷ�u0��ͳ��i~255�Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ�����������) ��ռ����ͼ��ı���w1����ͳ�Ʊ������ص�ƽ���Ҷ�u1��
////5) ����ǰ�����غͱ������صķ��� g = w0*w1*(u0-u1) (u0-u1)
////6) i++��ת��4)��ֱ��iΪ256ʱ��������
////7�������g��Ӧ��iֵ��Ϊͼ���ȫ����ֵ
////ȱ��:OSTU�㷨�ڴ�����ղ����ȵ�ͼ���ʱ��Ч�������Բ��ã���Ϊ���õ���ȫ��������Ϣ��
////***************************************************************/ 
////uint8_t GetOSTU(u8 tmImage[LCDH][LCDW]) 
////{ 
////  int16_t i,j; 
////  uint32_t Amount = 0; 
////  uint32_t PixelBack = 0; 
////  uint32_t PixelIntegralBack = 0; 
////  uint32_t PixelIntegral = 0; 
////  int32_t PixelIntegralFore = 0; 
////  int32_t PixelFore = 0; 
////  double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��; 
////  int16_t MinValue, MaxValue; 
////  uint8_t Threshold = 0;
////  uint8_t HistoGram[256];              //  
////  
////  for (j = 0; j < 256; j++)  HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ 
////  
////  for (j = 0; j < IMAGEH; j++) 
////  { 
////    for (i = 0; i < IMAGEW; i++) 
////    { 
////      HistoGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
////    } 
////  } 
////  
////  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //��ȡ��С�Ҷȵ�ֵ
////  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //��ȡ���Ҷȵ�ֵ
////  
////  if (MaxValue == MinValue)     return MaxValue;         // ͼ����ֻ��һ����ɫ    
////  if (MinValue + 1 == MaxValue)  return MinValue;        // ͼ����ֻ�ж�����ɫ
////  
////  for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  ��������
////  
////  PixelIntegral = 0;
////  for (j = MinValue; j <= MaxValue; j++)
////  {
////    PixelIntegral += HistoGram[j] * j;//�Ҷ�ֵ����
////  }
////  SigmaB = -1;
////  for (j = MinValue; j < MaxValue; j++)
////  {
////    PixelBack = PixelBack + HistoGram[j];    //ǰ�����ص���
////    PixelFore = Amount - PixelBack;         //�������ص���
////    OmegaBack = (double)PixelBack / Amount;//ǰ�����ذٷֱ�
////    OmegaFore = (double)PixelFore / Amount;//�������ذٷֱ�
////    PixelIntegralBack += HistoGram[j] * j;  //ǰ���Ҷ�ֵ
////    PixelIntegralFore = PixelIntegral - PixelIntegralBack;//�����Ҷ�ֵ
////    MicroBack = (double)PixelIntegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
////    MicroFore = (double)PixelIntegralFore / PixelFore;   //�����ҶȰٷֱ�
////    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//������䷽��
////    if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
////    {
////      SigmaB = Sigma;
////      Threshold = j;
////    }
////  }
////  return Threshold;                        //���������ֵ;
////} 
//
/////***************************************************************
////* 
////* �������ƣ�SendPicture 
////* ����˵��������ͼ����λ�� ����ͬ����λ��ע���޸Ķ�Ӧ�����ݽ���Э��
////* ����˵���� 
////* �������أ�void 
////* �޸�ʱ�䣺2018��3��27�� 
////* �� ע�� 
////***************************************************************/ 
////void UARTSendPicture2(uint8_t tmImage[IMAGEH][IMAGEW]) 
////{ 
////  int i = 0, j = 0; 
////  UART_Put_Char(UART_5,0xFE); //����֡ͷ��־ WindowsFormsApplication1.exe
////  UART_Put_Char(UART_5,0xEF); //����֡ͷ��־ WindowsFormsApplication1.exe
////  for(i=0;i < IMAGEH; i++) 
////  { 
////    for(j=0;j <IMAGEW;j++) 
////    { 
////      if(tmImage[i][j]==0xff) 
////      { 
////        tmImage[i][j]=0xfe; //��ֹ���ͱ�־λ 
////      } 
////      UART_Put_Char(UART_5,tmImage[i][j]); 
////    } 
////  }
////  UART_Put_Char(UART_5,0xEF); //����֡β��־ 
////  UART_Put_Char(UART_5,0xFE); //����֡β��־ 
////} 
////void UARTSendPicture3(uint8_t tmImage[LCDH][LCDW]) 
////{ 
////  int i = 0, j = 0; 
////  UART_Put_Char(UART_5,0xFE); //����֡ͷ��־ WindowsFormsApplication1.exe
////  UART_Put_Char(UART_5,0xEF); //����֡ͷ��־ WindowsFormsApplication1.exe
////  for(i=0;i < LCDH; i++) 
////  { 
////    for(j=0;j <LCDW;j++) 
////    { 
////      if(tmImage[i][j]==0xff) 
////      { 
////        tmImage[i][j]=0xfe; //��ֹ���ͱ�־λ 
////      } 
////      UART_Put_Char(UART_5,tmImage[i][j]); 
////    } 
////  }
////  UART_Put_Char(UART_5,0xEF); //����֡β��־ 
////  UART_Put_Char(UART_5,0xFE); //����֡β��־ 
////} 
////void UARTSendPicture(uint8_t tmImage[IMAGEH][IMAGEW]) 
////{ 
////  int i = 0, j = 0; 
////  UART_Put_Char(UART_5,0xFF); //����֡ͷ��־ DEMOK��λ��  
////  for(i=0;i < IMAGEH; i++) 
////  { 
////    for(j=0;j <IMAGEW;j++) 
////    { 
////      if(tmImage[i][j]==0xff) 
////      { 
////        tmImage[i][j]=0xfe; //��ֹ���ͱ�־λ 
////      } 
////      UART_Put_Char(UART_5,tmImage[i][j]); 
////    } 
////  }
////} 
//
////��ʾͼ��OLEDģ��
////void Draw_huidu_Picture(unsigned int x,unsigned int y)
////{
////  u8 i = 0, j = 0,temp=0;
////  for(i=0;i<94;i++)
////  {
////    for(j=0;j<60;j++)  //����
////    { 
////      if(Pixle[j][i]==65535){
////        Gui_DrawPoint(2*i+x,2*j+y,COLOR_WHITE);
////        Gui_DrawPoint(2*i+1+x,2*j+y,COLOR_WHITE);
////        Gui_DrawPoint(2*i+x,2*j+y+1,COLOR_WHITE);
////        Gui_DrawPoint(2*i+x+1,2*j+y+1,COLOR_WHITE);
////      }
////      else if(Pixle[j][i]==0){
////        Gui_DrawPoint(2*i+x,2*j+y,COLOR_BLACK);
////        Gui_DrawPoint(2*i+1+x,2*j+y,COLOR_BLACK);
////        Gui_DrawPoint(2*i+x,2*j+y+1,COLOR_BLACK);
////        Gui_DrawPoint(2*i+x+1,2*j+y+1,COLOR_BLACK);
////      }
////    }
////  }  
////
////}
////void Draw_Road(unsigned int x,unsigned int y)
////{ 	 
////  u8 i = 0, j = 0,temp=0;
////  
////  //����֡ͷ��־
////  for(i=0;i<94;i++)
////    //for(i=8;i<128;i+=8)//6*8=48�� 
////  {
////    //TFTSPI_Set_Pos(18,i+10,18+188-1,i+10+8-1);//��ʼλ��
////    for(j=0;j<60;j++)  //����
////    { 
////      /*
////      temp=0;
////      if(Image_Data[0+i][j]) temp|=1;
////      if(Image_Data[1+i][j]) temp|=2;
////      if(Image_Data[2+i][j]) temp|=4;
////      if(Image_Data[3+i][j]) temp|=8;
////      if(Image_Data[4+i][j]) temp|=0x10;
////      if(Image_Data[5+i][j]) temp|=0x20;
////      if(Image_Data[6+i][j]) temp|=0x40;
////      if(Image_Data[7+i][j]) temp|=0x80;
////      TFTSPI_Write_Byte(temp); 	  	
////      */
////      //      for(m=0;m<1;m++)
////      //        for(n=0;n<1;n++)
//////      time_delay_ms(100);
////      if(Pixle[j][i]==65535){
////        Gui_DrawPoint(2*i+x,2*j+y,COLOR_WHITE);
////        Gui_DrawPoint(2*i+1+x,2*j+y,COLOR_WHITE);
////        Gui_DrawPoint(2*i+x,2*j+y+1,COLOR_WHITE);
////        Gui_DrawPoint(2*i+x+1,2*j+y+1,COLOR_WHITE);
////      }
////      else if(Pixle[j][i]==0){
////        Gui_DrawPoint(2*i+x,2*j+y,COLOR_BLACK);
////        Gui_DrawPoint(2*i+1+x,2*j+y,COLOR_BLACK);
////        Gui_DrawPoint(2*i+x,2*j+y+1,COLOR_BLACK);
////        Gui_DrawPoint(2*i+x+1,2*j+y+1,COLOR_BLACK);
////      }
////    }
////  }  
////}
//const MT9V034_REG_DATA MT_RegData[] = 
//{
//  {MT9V034_READ_MODE,           0x003A},
//  {MT9V034_WINDOW_WIDTH,        752},
//  {MT9V034_WINDOW_HEIGHT,       480},
//  {MT9V034_COLUMN_START,        MT9V034_COLUMN_START_MIN},
//  {MT9V034_ROW_START,           MT9V034_ROW_START_MIN},
//  {MT9V034_AEC_AGC_ENABLE,      MT9V034_AEC_ENABLE|MT9V034_AGC_ENABLE},
//  {0xAC,                        0x0001},
//  {0xAD,                        0x01E0},
//  {0x2C,                        0x0004},
//  {MT9V034_ANALOG_CTRL,         MT9V034_ANTI_ECLIPSE_ENABLE},
//  {0x0F,                        0x0000}, 
//  {MT9V034_HDR_ENABLE_REG,      0x0101},
//  {MT9V034_CHIP_CONTROL,        0x0188}, 
//  {MT9V034_ROW_NOISE_CORR_CTRL_REG, 0},
//  {MT9V034_AEC_AGC_ENABLE_REG,  0x0000},
//  {MT9V034_MIN_EXPOSURE_REG,    0x0001},
//  {MT9V034_MAX_EXPOSURE_REG,    0x01E0},
//  {MT9V034_MAX_GAIN_REG,        50},            //40
//  {MT9V034_AGC_AEC_PIXEL_COUNT_REG, 188*120},
//  {MT9V034_ADC_RES_CTRL_REG,    0x0303},     
//  {0x13,                        0x2D2E},        
//  {0x20,                        0x03C7},
//  {0x24,                        0x0010},
//  {0x2B,                        0x0003},
//  {0x2F,                        0x0003},  
//  {MT9V034_SHUTTER_WIDTH1,      0x01BB},
//  {MT9V034_SHUTTER_WIDTH2,      0x01D9},
//  {MT9V034_SHUTTER_WIDTH_CONTROL,0x000f},//*********************
//  {MT9V034_V2_CTRL_REG_A,       0x001A},
//  {MT9V034_TOTAL_SHUTTER_WIDTH, 0x0188},//***************************
//  {MT9V034_HDR_ENABLE_REG,      0x0103},
//  {MT9V034_AGC_AEC_DESIRED_BIN_REG, MT9V034_BRIGHT},
//  {MT9V034_ANALOG_GAIN,0x8010},
//  {MT9V034_RESET, 0x03}
//};
//
//const int Reg_cnt = sizeof(MT_RegData)/sizeof(MT9V034_REG_DATA);
//
//__ramfunc void DMATransDataInit(DMA_CHn CHn,void *SADDR, void *DADDR,PTXn_e ptxn,DMA_BYTEn byten,u32 count,DMA_Count_cfg cfg) ;
//__ramfunc void DMATransDataStart(uint8_t CHn,uint32_t address,uint32_t Val) ;
//
////����ͷͼ��ɼ��жϴ����� 
//void PORTC_IRQHandler(void) 
//{  
//  //���ж�PTD13 
//  if((PORTC_ISFR & 0x80)) //���ж� PTD13 (1<<13) 
//  { 
//    PORTC_ISFR |= 0x80;   //����жϱ�ʶ 
//    if(Line_Cont > IMAGEH)  //�ɼ�����
//    { 
//      Line_Cont=0; 
//      return ;
//    }
//    DMATransDataStart(DMA_CH4,(uint32_t)(&Image_Data[Line_Cont][0]),IMAGEW); //DMA��ʼ��������PTD12�ܽŴ���
//    ++Line_Cont;            //�м���
//    return ;
//  }
//  //���ж�PTD14
//  if((PORTC_ISFR & 0x40))//���ж� PTD14 (1<<14)
//  {
//    PORTC_ISFR |= 0x40;  //����жϱ�ʶ    
//    // �û����� 
//    Line_Cont = 0;         //�м�������
//    Field_Over_Flag=1;     //��������ʶ
//    return ;
//  }  
//} 
////
////void PictureProcessing(u8 Image_Data[IMAGEH][IMAGEW],uint8_t ThresholdV)
////{  
////  int i = 0, j = 0; 
////  if(Field_Over_Flag)   //���һ��ͼ��ɼ�����ʾ���������ݵ���λ��
////  {
////
////  for(i = 0;i <IMAGEH;i++) 
////  { 
////    for(j = 0; j< IMAGEW;j++) 
////    { 
////      if(tmImage[i][j] >= ThresholdV) 
////      { 
////        Pixle[i][j] = 65535; 
////        
////      } 
////      else 
////      { 
////        Pixle[i][j] = 0; 
////      } 
////    } 
////  } 
////    
////    Field_Over_Flag= 0;
////  } 
////}
//
//
////void Test_LQV034(void)//����������
////{  
////  if(Field_Over_Flag)   //���һ��ͼ��ɼ�����ʾ���������ݵ���λ��
////  {
////    Get_Use_Image();    //�ɼ�ͼ�����ݴ������  		��Dataѹ����Use�ķ�֮һƽ��
////    //Threshold = GetOSTU(Image_Use); //OSTU���		��ȡuse��ֵ
////    //Draw_Road(0,0);
////    BinaryImage(Image_Use,sign); //��ֵ��ͼ������	��use��ֵ��ΪPixle
////    Field_Over_Flag= 0;
////  //  UARTSendPicture2(Image_Data); 
////    lcd++;
//////    numtostr2(x_cm);
//////      LCD_PutString(0,20,nts2,COLOR_RED,COLOR_WHITE);
////  } 
////}
//
//
//
//void MT9V034_Init(void)// MT9V034 Port Init
//{       
//  uint16_t data = 0; 
//  
//  //GPIO�ڳ�ʼ��
//  EXTI_Init(GPIOC,7,rising_down);   //���ж�
//  EXTI_Init(GPIOC,6,falling_up);    //���ж�  
//  GPIO_Init(GPIOB,16,GPI,0);          //��λ���������      
//  GPIO_Init(GPIOB,17,GPI,0);
//  GPIO_Init(GPIOB,18,GPI,0);
//  GPIO_Init(GPIOB,19,GPI,0);
//  GPIO_Init(GPIOB,20,GPI,0);
//  GPIO_Init(GPIOB,21,GPI,0);
//  GPIO_Init(GPIOB,22,GPI,0);
//  GPIO_Init(GPIOB,23,GPI,0);
//  GPIO_Init(GPIOC,5,GPI,0);     //����  
//  
//  //����ͷ�Ĵ�������
//  SCCB_Init();                     //������ַ�ڶ����ߣ�ΪMT9V034_I2C_ADDR 
//  
//  if(SCCB_RegRead(MT9V034_I2C_ADDR>>1,MT9V034_CHIP_VERSION,&data) == 0)//��ȡ����ͷ�汾�Ĵ��� 
//  {     
//    if(data != MT9V034_CHIP_ID)                                  //оƬID����ȷ��˵��û����ȷ��ȡ�����ݣ��ȴ�      
//    { 
//      //LCD_P6x8Str(2,1,(u8*)"V034 NG");                      //����ͷʶ��ʧ�ܣ�ֹͣ����
//      GPIO_Ctrl (GPIOE, 19, 0);
//      while(1); 
//    } 
//    else                                                   //оƬID��ȷ
//    {
//      //LCD_P6x8Str(2,1,(u8*)"V034 OK");
//    }  
//  } 
//  else 
//  { 
//    GPIO_Ctrl (GPIOE, 19, 0);
//    while(1); //����ͷʶ��ʧ�ܣ�ֹͣ����
//  }  
//  
//
//  MT9V034_SetFrameResolution(IMAGEH, IMAGEW);//��������ͷͼ��4*4��Ƶ���PCLK, 27/4 = 6.75M ,BIT4,5��������:�������Ҿ����� 
//
//  MT9V034_SetAutoExposure(true);
//  
//  SCCB_RegWrite(MT9V034_I2C_ADDR, 0xAC, 0x0001);
//  SCCB_RegWrite(MT9V034_I2C_ADDR, 0xAD, 0x01E0);  
//  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x2C, 0x0004);
//  
//  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x7F, 0x3000);           // test pattern
//  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x05, 0x0100);// ˮƽ����ʱ����ֵ̫СЧ����
//  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x06, 0x00AC );// VERT_BLANK_CONTEXTA_REG��ֱ����ʱ���Զ��ع�AC�ֶ�A����Сֵ,����ֵЧ������ ��
//
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_ANALOG_CTRL, MT9V034_ANTI_ECLIPSE_ENABLE);
//      SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_SHUTTER_WIDTH_CONTROL,0x0064); //0x0A Coarse Shutter IMAGEW Control 
//   SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_HDR_ENABLE_REG,0x0001);        /* 0x0F High Dynamic Range enable,bit is set (R0x0F[1]=1), 
//                                                                          the sensor uses black level correction values from one green plane, 
//                                                                          which are applied to all colors. */
//  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_SHUTTER_WIDTH1,0x01BB);        //0x08 Coarse Shutter IMAGEW   1 imagew�ֿ��ſ��1
//  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_SHUTTER_WIDTH2,0x01D9);        //0x09 Coarse Shutter IMAGEW 2�ֿ��ſ��2
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AEC_AGC_ENABLE_REG, 0x0303 );   //0xAF  AEC/AGC A~bit0:1AE;bit1:1AG/B~bit2:1AE;bit3:1AG
//  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_TOTAL_SHUTTER_WIDTH,0x0008);   //0x0B Coarse Shutter IMAGEW Total�ع�ʱ��*�������Զ��ع⣬��Ч*
//SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_ADC_RES_CTRL_REG, 0x0002);     //0x1C  here is the way to regulate darkness :) ADCѹ��2��3   
//  
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x2C, 0x0004 );// VREF_ADC_CONTROL
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x31, 0x003F );// V1_CONTROL_CONTEXTA
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x32, 0x0021 );// V2_CONTROL_CONTEXTA
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x33, 0x0006 );// V3_CONTROL_CONTEXTA
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x34, 0x0003 );// V4_CONTROL_CONTEXTA
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x35, 0x0010 );// GLOBAL_GAIN_CONTEXTA_REG
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x36, 0x0010 );// GLOBAL_GAIN_CONTEXTB_REG
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x39, 0x003F );// V1_CONTROL_CONTEXTB
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x3A, 0x0021 );// V2_CONTROL_CONTEXTB
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x3B, 0x0006 );// V3_CONTROL_CONTEXTB
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x3C, 0x0003 );// V4_CONTROL_CONTEXTB
//
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x46, 0x350D );// DARK_AVG_THRESHOLDSƽ���ڰ���ֵ
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x47, 0x0081 );// CALIB_CONTROL_REG�ڶ�ˮƽ�궨����
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x48, 0x00AF );// Black Level Calibration Value Context A�ڶ�ˮƽ�궨ֵ����������ڰ׶Աȣ�
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x4C, 0x0001 );// STEP_SIZE_AVG_MODE�ڶ�ˮƽУ׼ֵ0-31����ͷ�Ķ�̬��ֵ����
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x70, 0x0003 );// ROW_NOISE_CONTROL��������������
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x71, 0x002A );// NOISE_CONSTANT����������
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x72, 0x0000 );// PIXCLK_CONTROL����ʱ�Ӻ���Ч�п���
////SCCB_RegWrite(MT9V034_I2C_ADDR, 0x7F, 0x0000 );// TEST_DATA
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x80, 0x04F4 );// TILE_X0_Y0
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x81, 0x04F4 );// TILE_X1_Y0
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x82, 0x04F4 );// TILE_X2_Y0
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x83, 0x04F4 );// TILE_X3_Y0
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x84, 0x04F4 );// TILE_X4_Y0
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x85, 0x04F4 );// TILE_X0_Y1
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x86, 0x04F4 );// TILE_X1_Y1
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x87, 0x04F4 );// TILE_X2_Y1
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x88, 0x04F4 );// TILE_X3_Y1
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x89, 0x04F4 );// TILE_X4_Y1
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x8A, 0x04F4 );// TILE_X0_Y2
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x8B, 0x04F4 );// TILE_X1_Y2
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x8C, 0x04F4 );// TILE_X2_Y2
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x8D, 0x04F4 );// TILE_X3_Y2
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x8E, 0x04F4 );// TILE_X4_Y2
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x8F, 0x04F4 );// TILE_X0_Y3
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x90, 0x04F4 );// TILE_X1_Y3
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x91, 0x04F4 );// TILE_X2_Y3
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x92, 0x04F4 );// TILE_X3_Y3
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x93, 0x04F4 );// TILE_X4_Y3
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x94, 0x04F4 );// TILE_X0_Y4
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x95, 0x04F4 );// TILE_X1_Y4
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x96, 0x04F4 );// TILE_X2_Y4
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x97, 0x04F4 );// TILE_X3_Y4
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x98, 0x04F4 );// TILE_X4_Y4
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x99, 0x0000 );// X0_SLASH5
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x9A, 0x0096 );// X1_SLASH5
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x9B, 0x012C );// X2_SLASH5
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x9C, 0x01C2 );// X3_SLASH5
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x9D, 0x0258 );// X4_SLASH5
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x9E, 0x02F0 );// X5_SLASH5
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0x9F, 0x0000 );// Y0_SLASH5
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0xA0, 0x0060 );// Y1_SLASH5
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0xA1, 0x00C0 );// Y2_SLASH5
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0xA2, 0x0120 );// Y3_SLASH5
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0xA3, 0x0180 );// Y4_SLASH5
//SCCB_RegWrite(MT9V034_I2C_ADDR, 0xA4, 0x01E0 );// Y5_SLASH5
//
//
//  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_V2_CTRL_REG_A, 0x001A);        //0x32   0x001A  
//
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_HDR_ENABLE_REG, 0x0000); // 0x0F bit8:1HDR,0linear; bit1:1��ɫ,0�Ҷ�;bit0:1HDR,0linear
//  //MT9V034_WriteReg16(MT9V034_HDR_ENABLE_REG, 0x0103);     // 0x0F bit8:1HDR,0linear; bit1:1��ɫ,0�Ҷ�;bit0:1HDR,0linear
//  //0x07 Chip Control bit2-0:0����ɨ��,1��Ч��2����3������bit5:1�����Ӿ�ʹ��,bit7:1����ʹ�ܣ�bit8:1ͬ��ģʽ;bit9:1����������bit15:0A/1B
//  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_CHIP_CONTROL, 0x0188);          //Context A  0x07
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_ROW_NOISE_CORR_CTRL_REG, 0);   //0x70  0x0000 
//
//  
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MIN_EXPOSURE_REG, 0x0001);     //0xAC  ��С�ֿ��ſ��   0x0001
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MAX_EXPOSURE_REG, 0x0480);     //0xAD  ���׿��ſ��   0x01E0-480
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MAX_GAIN_REG, 50);             //0xAB  ���ģ������     64
//  
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AGC_AEC_PIXEL_COUNT_REG, 188*120);//0xB0  ����AEC/AGCֱ��ͼ������Ŀ,���44000   4096=320*240  
//  
//  SCCB_RegWrite(MT9V034_I2C_ADDR,0x13,0x2D2E);//We also recommended using R0x13 = 0x2D2E with this setting for better column FPN.  
//  SCCB_RegWrite(MT9V034_I2C_ADDR,0x20,0x03C7);//0x01C7�ԱȶȲ���ף�0x03C7�Աȶ���� Recommended by design to improve performance in HDR mode and when frame rate is low.
//  SCCB_RegWrite(MT9V034_I2C_ADDR,0x24,0x0010);//Corrects pixel negative dark offset when global reset in R0x20[9] is enabled.
//  SCCB_RegWrite(MT9V034_I2C_ADDR,0x2B,0x0003);//Improves column FPN.
//  SCCB_RegWrite(MT9V034_I2C_ADDR,0x2F,0x0003);//Improves FPN at near-saturation.  
//  
//  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x08, 0x03D4);
//  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x09, 0x03E7);
//  //100DB //�����ع�ʱ�� 
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AGC_AEC_DESIRED_BIN_REG, 64); //0xA5  ͼ������  50  1-64
//  SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_ANALOG_GAIN,0x8010);           //0x35
//  //SCCB_RegWrite(MT9V034_I2C_ADDR,0xAF,0x0003);
//  //80dB HDR
//  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x08, 0x03CA);
//  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x09, 0x03DE);
//  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x0A, 0x0064);
//  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x0B, 0x03E8);
//  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x0F, 0x0103);
//  //SCCB_RegWrite(MT9V034_I2C_ADDR, 0x35, 0x8010);   
//  
//  
////  for(int i = 0;i<Reg_cnt;i++)
////    SCCB_RegWrite(MT9V034_I2C_ADDR,MT_RegData[i].REG_ADDRESS,MT_RegData[i].REG_VAL);
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_RESET, 0x0003);          //0x0c  ��λ
//  DMATransDataInit(DMA_CH4,(void*)&PTB_BYTE2_IN,(void*)Image_Data,PTC5,DMA_BYTE1,IMAGEW,DMA_rising_down);//��ʼ��DMA�ɼ�  ����λC5
//  
//}
//void MT9V034_SetFrameResolution(uint16_t height,uint16_t width)
//{
//  uint16_t data = 0;
//  
//  if((width*4)<=MAX_IMAGE_WIDTH && (height*4)<=MAX_IMAGE_HEIGHT)
//  {
//    width *= 4;
//    height *= 4;
//    data |= MT9V034_READ_MODE_ROW_BIN_4;
//    data |= MT9V034_READ_MODE_COL_BIN_4;
//  }
//  else if((width*2)<=MAX_IMAGE_WIDTH && (height*2)<=MAX_IMAGE_HEIGHT)
//  {
//    width *= 2;
//    height *= 2;
//    data |= MT9V034_READ_MODE_ROW_BIN_2;
//    data |= MT9V034_READ_MODE_COL_BIN_2;
//  }
//  
//  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x01,MT9V034_COLUMN_START_DEF);     // Column Start
//  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x02,MT9V034_ROW_START_DEF);        // Row Start  
//  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x03,MT9V034_WINDOW_HEIGHT_DEF);    // height 
//  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x04,MT9V034_WINDOW_WIDTH_DEF);     // width  
//  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x05,MT9V034_HORIZONTAL_BLANKING_MIN);   // Horizontal Blanking  809-640
//  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x06,MT9V034_VERTICAL_BLANKING_MIN);     // Vertical Blanking    499-480 
//  
//  //����ͼ�������СΪ120*188������ʧ��Ұ��ͬʱ�������ܳ���Ҫ
//  SCCB_RegWrite(MT9V034_I2C_ADDR,0x0D,0x030A);   //��������ͷͼ��4*4��Ƶ���PCLK, 27/4 = 6.75M ,BIT4,5�������� 
//  //SCCB_RegWrite(MT9V034_I2C_ADDR,0x0D,0x033A);   //��������ͷͼ��4*4��Ƶ���PCLK, 27/4 = 6.75M ,BIT4,5��������:�������Ҿ����� 
//  
//  data |= (MT9V034_READ_MODE_ROW_FLIP|MT9V034_READ_MODE_COLUMN_FLIP);       //LQ-MT9V034 needs vertical mirror to capture correct image
//  
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_READ_MODE, data);
//  
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_WINDOW_WIDTH,  width);
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_WINDOW_HEIGHT, height);
//  
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_COLUMN_START, (MAX_IMAGE_WIDTH-width)/2+MT9V034_COLUMN_START_MIN);
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_ROW_START, (MAX_IMAGE_HEIGHT-height)/2+MT9V034_ROW_START_MIN);
//}
//
//void MT9V034_SetAutoExposure(bool enable)
//{
//  uint16_t reg =0;
//  SCCB_RegRead(MT9V034_I2C_ADDR, MT9V034_AEC_AGC_ENABLE,&reg);
//  if(true == enable)
//  {
//    SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AEC_AGC_ENABLE, reg|MT9V034_AEC_ENABLE|MT9V034_AGC_ENABLE);
//  }
//  else
//  {
//    SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AEC_AGC_ENABLE, reg&~(MT9V034_AEC_ENABLE|MT9V034_AGC_ENABLE));
//  }
//}
//void MT9V034_SetFrameRate(uint8_t frameRate)
//{
//  
//}
//
//void MT9V034_Reset(void)
//{
//  //Reset MT9V034, but register config will not change.
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_RESET, 0x0001);
//  time_delay_ms(10);
//  
//  //Unlock MT9V034, allowing user to initiate register settings and readout
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_CHIP_CONTROL, 0x0188);
//  
//  //Reset Again.
//  SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_RESET, 0x0001);
//  time_delay_ms(10);
//}
//
//void MT9V034_SetReservedReg(void)
//{
//  //Here we write some reserved registers as recommendations from Rev.G datasheet, Table.8
//  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x13, 0x2D2E);
//  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x20, 0x03C7);
//  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x24, 0x001B);
//  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x2B, 0x0003);
//  SCCB_RegWrite(MT9V034_I2C_ADDR, 0x2F, 0x0003);
//}
///***************************************************************  
//* 
//* �������ƣ�void DMATransDataInit(DMA_CHn CHn,void *SADDR, void *DADDR,PTXn_e ptxn,DMA_BYTEn byten,u32 count,DMA_PORTx2BUFF_cfg cfg) 
//* ����˵������ʼ��DMA 
//* ����˵����ͨ����ţ�Դ��ַ��Ŀ�ĵ�ַ�������ܽţ��ֽ�����ѭ���������������� 
//* �������أ��� 
//* �޸�ʱ�䣺2018��3��27�� 
//* �� ע�� 
//***************************************************************/ 
//__ramfunc void DMATransDataInit(DMA_CHn CHn,void *SADDR, void *DADDR,PTXn_e ptxn,DMA_BYTEn byten,u32 count,DMA_Count_cfg cfg) 
//{ 
//  u8 BYTEs = (byten == DMA_BYTE1 ? 1 : (byten == DMA_BYTE2 ? 2 : (byten == DMA_BYTE4 ? 4 : 16 ) ) ); //���㴫���ֽ���
//  //����ʱ�� 
//  SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;                        //��DMAģ��ʱ��
//  SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;                     //��DMA��·������ʱ��
//  // ���� DMA ͨ�� �� ������ƿ� TCD ( Transfer Control Descriptor ) 
//  DMA_SADDR(CHn) =    (u32)SADDR;                         // ����  Դ��ַ
//  DMA_DADDR(CHn) =    (u32)DADDR;                         // ����Ŀ�ĵ�ַ
//  DMA_SOFF(CHn)  =    0;                                  // ����Դ��ַ����
//  DMA_DOFF(CHn)  =    BYTEs;                                  // ÿ�δ����Ŀ�ļ�BYUEs
//  DMA_ATTR(CHn)  =    (0
//                       | DMA_ATTR_SMOD(0x0)                // Դ��ַģ����ֹ  Source address modulo feature is disabled
//                         | DMA_ATTR_SSIZE(byten)             // Դ����λ�� ��DMA_BYTEn  ��    SSIZE = 0 -> 8-bit ��SSIZE = 1 -> 16-bit ��SSIZE = 2 -> 32-bit ��SSIZE = 4 -> 16-byte
//                           | DMA_ATTR_DMOD(0x0)                // Ŀ���ַģ����ֹ
//                             | DMA_ATTR_DSIZE(byten)             // Ŀ������λ�� ��DMA_BYTEn  ��  ���òο�  SSIZE
//                               );
//  DMA_CITER_ELINKNO(CHn)  = DMA_CITER_ELINKNO_CITER(count); //��ǰ��ѭ������
//  DMA_BITER_ELINKNO(CHn)  = DMA_BITER_ELINKYES_BITER(count);//��ʼ��ѭ������
//  
//  DMA_CR &= ~DMA_CR_EMLM_MASK;                            // CR[EMLM] = 0  disable Minor Loop Mapping
//  
//  DMA_NBYTES_MLNO(CHn) =   DMA_NBYTES_MLNO_NBYTES(BYTEs); // ͨ��ÿ�δ����ֽ�������������ΪBYTEs���ֽڡ�ע��ֵΪ0��ʾ����4GB 
//  
//  // ���� DMA ���������Ĳ��� 
//  DMA_SLAST(CHn)      =   0;                              //����Դ��ַ�ĸ���ֵ,��ѭ��������ָ�Դ��ַ
//  DMA_DLAST_SGA(CHn)  =   0;                              //����Ŀ�ĵ�ַ�ĸ���ֵ,��ѭ��������ָ�Ŀ�ĵ�ַ���߱��ֵ�ַ
//  DMA_CSR(CHn)        =   (0
//                           | DMA_CSR_DREQ_MASK            //��ѭ��������ֹͣӲ������
//                             | DMA_CSR_INTMAJOR_MASK        //��ѭ������������ж�
//                               );
//  // ���� DMA ����Դ 
//  DMAMUX_CHCFG_REG(DMAMUX_BASE_PTR, CHn) = (0
//                                            | DMAMUX_CHCFG_ENBL_MASK                        // Enable routing of DMA request 
//                                              | DMAMUX_CHCFG_SOURCE((ptxn >> 5) + DMA_Port_A) // ͨ����������Դ:     
//                                                );
//  //GPIO_Init(GPIOD,12,GPI,0); //����  
//  SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK << (ptxn>>5));                                //����PORTx�˿�
//  GPIO_PDDR_REG(GPIOX[(ptxn>>5)]) &= ~(1 << (ptxn & 0x1f));                        //���ö˿ڷ���Ϊ����
//  PORT_PCR_REG(PORTX[(ptxn>>5)], (ptxn & 0x1F)) = ( 0
//                                                   | PORT_PCR_MUX(1)               // ����GPIO
//                                                     | PORT_PCR_IRQC(cfg & 0x03 )    // ȷ������ģʽ
//                                                       | ((cfg & 0xc0 ) >> 6)          // �����������������裬����û��
//                                                         );
//  GPIO_PDDR_REG(GPIOX[(ptxn>>5)]) &= ~(1 << (ptxn && 0x1F));  
//  
//  //�����ж� 
//  DMA_EN(CHn);                                      //ʹ��ͨ��CHn Ӳ������
//  //DMA_IRQ_EN(CHn);                                //����DMAͨ������
//} 
//
///*************************************************************** 
//
//* 
//* �������ƣ�void DMATransDataStart(uint8_t CHn,uint32_t address,uint32_t Val) 
//* ����˵��������DMA�����Ŀ�ĵ�ַ��ѭ������ 
//* ����˵���� 
//* �������أ���
//* �޸�ʱ�䣺2018��3��27�� 
//* �� ע�� 
//***************************************************************/ 
//__ramfunc void DMATransDataStart(uint8_t CHn,uint32_t address,uint32_t Val) 
//{ 
//  DMA_DADDR(CHn) = address;                              //Ŀ�ĵ�ַ 
//  DMA_CITER_ELINKNO(CHn) = DMA_CITER_ELINKNO_CITER(Val); //��ǰ��ѭ������ 
//  DMA_BITER_ELINKNO(CHn) = DMA_BITER_ELINKYES_BITER(Val);//��ʼ��ѭ������ 
//  //�����ж� 
//  DMA_EN(CHn);                                           //ʹ��ͨ��CHn Ӳ������
//  //DMA_IRQ_EN(CHn);                                     //����DMAͨ������
//} 
//
////void Draw_Road2(unsigned int x,unsigned int y)
////{ 	 
////  u8 i = 0, j = 0,temp=0;
////  
////  //����֡ͷ��־
////  for(i=0;i<94;i++)
////    //for(i=8;i<128;i+=8)//6*8=48�� 
////  {
////    //TFTSPI_Set_Pos(18,i+10,18+188-1,i+10+8-1);//��ʼλ��
////    for(j=0;j<60;j++)  //����
////    { 
////      /*
////      temp=0;
////      if(Image_Data[0+i][j]) temp|=1;
////      if(Image_Data[1+i][j]) temp|=2;
////      if(Image_Data[2+i][j]) temp|=4;
////      if(Image_Data[3+i][j]) temp|=8;
////      if(Image_Data[4+i][j]) temp|=0x10;
////      if(Image_Data[5+i][j]) temp|=0x20;
////      if(Image_Data[6+i][j]) temp|=0x40;
////      if(Image_Data[7+i][j]) temp|=0x80;
////      TFTSPI_Write_Byte(temp); 	  	
////      */
////      //      for(m=0;m<1;m++)
////      //        for(n=0;n<1;n++)
////      
////      Gui_DrawPoint(i+x,j+y,Image_Use[j][i]*250);
////      
////    }
////    
////  }  
////  
////}
////�������Ϸ���Χ��������
////void Pixle_Filter(void)
////{  
////  int nr; //��
////  int nc; //��
////  
////  for(nr=1; nr<LCDH-1; nr++)
////  {  	    
////    for(nc=1; nc<LCDW-1; nc=nc+1)
////    {
////      if((Pixle[nr][nc]==0)&&((Pixle[nr-1][nc]==65535)+(Pixle[nr+1][nc]==65535)+(Pixle[nr][nc+1]==65535)+(Pixle[nr][nc-1]==65535)>2))         
////      {
////        Pixle[nr][nc]=65535;
////      }	
////      else if((Pixle[nr][nc]==65535)&&((Pixle[nr-1][nc]==0)+(Pixle[nr+1][nc]==0)+(Pixle[nr][nc+1]==0)+(Pixle[nr][nc-1]==0)<22))       
////      {
////        Pixle[nr][nc]=0;
////      }	
////    }	  
////  }  
////}
////
/////***************************************************************************
////*                                                                          *
////*  �������ƣ�int Seek_Road(void)                                           *
////*  ����˵����Ѱ�Ұ�ɫ����ƫ��ֵ                                            *
////*  ����˵������                                                            *
////*  �������أ�ֵ�Ĵ�С                                                      *
////*  �޸�ʱ�䣺2017-07-16                                                    *
////*  ��    ע�����м�Ϊ0������һ���Ҳ��һ����ֵ����1�����                *
////*            ��������ӵ�һ�п�ʼ�������ڶ��н�����                        *
////*            ������Ϊ��������ֵԽ��˵��Խƫ��ߣ�                        *
////*            ������Ϊ��������ֵԽ��˵��Խƫ�ұߡ�                        *
////***************************************************************************/ 
////void Seek_Road(void)
////{  
////  int nr; //��
////  int nc; //��
////  int temp=0;//��ʱ��ֵ
////  //for(nr=1; nr<MAX_ROW-1; nr++)
////  temp=0;
////  for(nr=8; nr<24; nr++)
////  {  	    
////    for(nc=MAX_COL/2;nc<MAX_COL;nc=nc+1)
////    {
////      if(Pixle[nr][nc])
////      {
////        ++temp;
////      }			   
////    }
////    for(nc=0; nc<MAX_COL/2; nc=nc+1)
////    {
////      if(Pixle[nr][nc])
////      {
////        --temp;
////      }			   
////    }		  
////  }
////  OFFSET0=temp;
////  temp=0;
////  for(nr=24; nr<40; nr++)
////  {  	    
////    for(nc=MAX_COL/2;nc<MAX_COL;nc=nc+1)
////    {
////      if(Pixle[nr][nc])
////      {
////        ++temp;
////      }			   
////    }
////    for(nc=0; nc<MAX_COL/2; nc=nc+1)
////    {
////      if(Pixle[nr][nc])
////      {
////        --temp;
////      }			   
////    }		  
////  }
////  OFFSET1=temp;    	
////  temp=0;
////  for(nr=40; nr<56; nr++)
////  {  	    
////    for(nc=MAX_COL/2;nc<MAX_COL;nc=nc+1)
////    {
////      if(Pixle[nr][nc])
////      {
////        ++temp;
////      }			   
////    }
////    for(nc=0; nc<MAX_COL/2; nc=nc+1)
////    {
////      if(Pixle[nr][nc])
////      {
////        --temp;
////      }			   
////    }		  
////  }
////  OFFSET2=temp;   	
////  return;  
////}
////
////u8 zb[48],yb[48];
////void FindTiXing(void)
////{
////  int nr; //��
////  int nc; //��     
////  
////  for(nr=0; nr<48; nr++)
////  {  	    
////    zb[nr]=0;
////    yb[nr]=100;   
////  }  	
////  for(nr=0; nr<48; nr++)
////  {  	    
////    for(nc=2;nc<MAX_COL-2;nc++)
////    {
////      if((Pixle[nr+8][nc-1]==0)&&(Pixle[nr+8][nc]==0)&&(Pixle[nr+8][nc+1]==1)&&(Pixle[nr+8][nc+2]==1))
////      {
////        zb[nr]=nc;//����أ�Խ��Խ��
////      }
////      if((Pixle[nr+8][nc-1]==1)&&(Pixle[nr+8][nc]==1)&&(Pixle[nr+8][nc+1]==0)&&(Pixle[nr+8][nc+2]==0))
////      {
////        yb[nr]=nc;//�ұ��أ�Խ��ԽС
////      }                   
////    }	    
////  }
////  TXV=0;
////  for(nr=0; nr<47; nr++)
////  {  	    
////    if((zb[nr]>=zb[nr+1])&&(zb[nr]>0))   TXV++;          
////    if((yb[nr]<=yb[nr+1])&&(yb[nr]<100)) TXV--;          
////  }  	   
////  return;  
////}
//
///*************************************************************************
//* �����������ܿƼ� KV58���ܳ�ĸ��           
//*
//*  �������ƣ�void SCCB_Init(void)
//*  ����˵��������SCCB��������ΪGPIO���ܣ���ʱ���������ݷ���
//*  ����˵������
//*  �������أ���
//*  �޸�ʱ�䣺2017��12��5��
//*  ��    ע��
//*************************************************************************/
//void SCCB_Init(void)
//{
//  GPIO_Init(GPIOC, 0,GPO,0);//����ΪGPIO����
//  GPIO_Init(GPIOC, 1,GPO,0);//����ΪGPIO����  
//}
//
///*************************************************************************
//* �����������ܿƼ� KV58���ܳ�ĸ��           
//*
//*  �������ƣ�void SCCB_Wait(void)
//*  ����˵����SCCB�ȴ���ʾ
//*  ����˵������
//*  �������أ���
//*  �޸�ʱ�䣺2017��12��5��
//*  ��    ע��
//*************************************************************************/
//void SCCB_Wait(void)
//{
//  uint8 i=0;
//  for(i=0;i<100;i++)
//  { 
//    asm ("nop");
//  }  
//}
//
///*************************************************************************
//* �����������ܿƼ� KV58���ܳ�ĸ��           
//*
//*  �������ƣ�void SCCB_Star(void)
//*  ����˵������������
//*  ����˵������
//*  �������أ���
//*  �޸�ʱ�䣺2017��12��5��
//*  ��    ע��
//*************************************************************************/
//void SCCB_Star(void)
//{
//  SCL_Out;
//  SDA_Out;
//  SCCB_Wait();
//  SDA_High;
//  SCL_High; 
//  SCCB_Wait();
//  SDA_Low;
//  SCCB_Wait();
//  SCL_Low; 
//  SCCB_Wait();
//}
///*************************************************************************
//* �����������ܿƼ� KV58���ܳ�ĸ��           
//*
//*  �������ƣ�void SCCB_Stop(void)
//*  ����˵����ֹͣ����
//*  ����˵������
//*  �������أ���
//*  �޸�ʱ�䣺2017��12��5��
//*  ��    ע��
//*************************************************************************/
//void SCCB_Stop(void)
//{
//  SCL_Out;
//  SDA_Out;
//  SCCB_Wait();
//  SDA_Low;
//  SCCB_Wait();
//  SCL_High; 
//  SCCB_Wait();
//  SDA_High;
//  SCCB_Wait();  
//}
///*************************************************************************
//* �����������ܿƼ� KV58���ܳ�ĸ��           
//*
//*  �������ƣ�uint8 SCCB_SendByte(uint8 Data)
//*  ����˵����SCCB�����ֽں���
//*  ����˵����Ҫ���͵��ֽ�
//*  �������أ�Ӧ���ź�
//*  �޸�ʱ�䣺2017��12��5��
//*  ��    ע��
//*************************************************************************/
//uint8 SCCB_SendByte(uint8 Data)
//{
//  uint8 i;
//  uint8 Ack;
//  SDA_Out;
//  for( i=0; i<8; i++)
//  {
//    if(Data & 0x80) SDA_High;
//    else            SDA_Low;    
//    Data <<= 1;
//    SCCB_Wait();
//    SCL_High;      
//    SCCB_Wait();
//    SCL_Low;
//    SCCB_Wait();
//  }
//  SDA_High;
//  SDA_In;
//  SCCB_Wait();
//  
//  SCL_High;
//  SCCB_Wait();
//  Ack = SDA_Data;
//  SCL_Low;
//  SCCB_Wait();
//  return Ack;
//}
///*************************************************************** 
//
//* 
//* �������ƣ�uint8 SCCB_ReadByte(void) 
//* ����˵����SCCB��ȡ�ֽں��� 
//* ����˵���� 
//* �������أ���ȡ�ֽ� 
//* �޸�ʱ�䣺2017��12��5�� 
//* �� ע�� 
//***************************************************************/ 
//uint8 SCCB_ReadByte(void) 
//{ 
//  uint8 i; 
//  uint8_t byte; 
//  SCL_Out; 
//  SDA_In; //ʹ������
//  for( i=0; i<8; i++) 
//  { 
//    SCL_Low;
//    SCCB_Wait(); 
//    SCL_High;
//    SCCB_Wait();
//    byte = (byte<<1)|(SDA_Data & 1);
//  }
//  SCL_Low; 
//  SDA_Out;
//  SCCB_Wait(); 
//  return byte; 
//} 
///*************************************************************** 
//
//* 
//* �������ƣ�static void SCCB_Ack(void) 
//* ����˵����IIC�лظ��ź� 
//* ����˵���� 
//* �������أ�void 
//* �޸�ʱ�䣺2017��12��5�� 
//* �� ע�� 
//***************************************************************/ 
//static void SCCB_Ack(void) 
//{ 
//  SCL_Out; 
//  SDA_Out;
//  SCL_Low;
//  SDA_Low;
//  SCCB_Wait();
//  SCL_High;
//  SCCB_Wait();
//  SCL_Low;
//  SCCB_Wait();
//} 
///*************************************************************** 
//
//* 
//* �������ƣ�static void SCCB_NAck(void) 
//* ����˵����IIC�޻ظ��ź� 
//* ����˵���� 
//* �������أ�void 
//* �޸�ʱ�䣺2017��12��5�� 
//* �� ע�� 
//***************************************************************/ 
//static void SCCB_NAck(void) 
//{ 
//  SCL_Out; 
//  SDA_Out;
//  SCL_Low;
//  SCCB_Wait();
//  SDA_High;
//  SCCB_Wait();
//  SCL_High;
//  SCCB_Wait();
//  SCL_Low;
//  SCCB_Wait();
//} 
//
///*************************************************************************
//* �����������ܿƼ� KV58���ܳ�ĸ��           
//*
//*  �������ƣ�void SCCB_RegWrite(uint8 Device,uint8 Address,uint16 Data)
//*  ����˵�������豸д����
//*  ����˵����Ҫ���͵��ֽ�
//*  �������أ�Ӧ���ź�
//*  �޸�ʱ�䣺2017��12��5��
//*  ��    ע��
//*************************************************************************/
//void SCCB_RegWrite(uint8 Device,uint8 Address,uint16 Data)
//{
//  uint8 i;
//  uint8 Ack;
//  
//  for( i=0; i<3; i++)
//  {
//    SCCB_Star();
//    Ack = SCCB_SendByte(Device);
//    if( Ack == 1 )
//    {
//      continue;
//    }
//    
//    Ack = SCCB_SendByte(Address);
//    if( Ack == 1 )
//    {
//      continue;
//    }
//    
//    Ack = SCCB_SendByte((uint8)(Data>>8));
//    Ack = SCCB_SendByte((uint8)Data);
//    if( Ack == 1 )
//    {
//      continue;
//    }
//    
//    SCCB_Stop();
//    if( Ack == 0 ) break;
//  }
//}
///*************************************************************** 
//
//* 
//* �������ƣ�uint8_t SCB_RegRead(uint8_t Device,uint8_t Address,uint16_t *Data) 
//* ����˵������ȡ���� 
//* ����˵���� 
//* �������أ�void 
//* �޸�ʱ�䣺2017��12��5�� 
//* �� ע�� 
//***************************************************************/ 
//uint8_t SCCB_RegRead(uint8_t Device,uint8_t Address,uint16_t *Data) 
//{ 
//  uint8 i; 
//  uint8 Ack = 0;
//  Device = Device<<1;
//  SCCB_Star();
//  Ack += SCCB_SendByte(Device);
//  
//  Ack += SCCB_SendByte(Address);
//  
//  SCCB_Star();
//  Ack += SCCB_SendByte(Device + 1);
//  
//  *Data = SCCB_ReadByte();
//  SCCB_Ack();
//  *Data = *Data<<8;
//  
//  *Data += SCCB_ReadByte();
//  SCCB_NAck();
//  
//  SCCB_Stop();
//  
//  return  Ack;
//} 
///***************************************************************  
//* 
//* �������ƣ�int SCCB_Probe(uint8_t chipAddr) 
//* ����˵������ѯ�õ�ַ���豸�Ƿ���� 
//* ����˵���� 
//* �������أ�void 
//* �޸�ʱ�䣺2017��12��5�� 
//* �� ע�� 
//***************************************************************/ 
//int SCCB_Probe(uint8_t chipAddr) 
//{ 
//  uint8_t err;
//  err = 0;
//  chipAddr <<= 1;
//  
//  SCCB_Star();
//  err = SCCB_SendByte(chipAddr);
//  SCCB_Stop();
//  
//  return err;
//}
//
