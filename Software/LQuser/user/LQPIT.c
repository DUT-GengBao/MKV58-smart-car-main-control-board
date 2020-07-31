/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨������KV58F24���ܳ�VDĸ��
����    д���ܽŸ��ò�����CHIUSIR�޸�ȷ�ϣ�Դ������SDK��Ұ����մ��������ֲ
��E-mail  ��chiusir@163.com
������汾��V1.0
�������¡�2017��12��15��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
------------------------------------------------
��dev.env.��IAR7.80.4
��Target  ��MKV58F1M0VLQ24
��Crystal �� 50.000Mhz
��busclock��137.500MHz
��pllclock��275.000MHz
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"
extern int cam;
extern int lcd;
extern int pix;
//-------------------------------------------------------------------------*
//������: pit_init
//��  ��: ��ʼ��PIT
//��  ��: pitn:ģ����PIT0��PIT1��PIT2��PIT3
//        cnt �ж�ʱ�䣬��λ1ms
//��  ��: ��
//��  ��: pit_init(PIT0,1000); PIT0�жϣ�1000ms����1s����PIT0_interrupt()һ��
//-------------------------------------------------------------------------*
void PIT_Init(PITn pitn, u32 cnt)
{
  //PIT �õ��� Bus Clock ����Ƶ��
  
  /* ����ʱ��*/
  SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;                            //ʹ��PITʱ��
  
  /* PITģ����� PIT Module Control Register (PIT_MCR) */
  PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );      //ʹ��PIT��ʱ��ʱ�� ������ģʽ�¼�������
  
  /* ��ʱ������ֵ���� Timer Load Value Register (PIT_LDVALn) */
  PIT_LDVAL(pitn)  = cnt*40*1000;                                   //��������ж�ʱ��
  
  //��ʱʱ�䵽�˺�TIF �� 1 ��д1��ʱ��ͻ���0
  PIT_Flag_Clear(pitn);                                             //���жϱ�־λ
  
  /* ��ʱ�����ƼĴ��� Timer Control Register (PIT_TCTRL0) */
  PIT_TCTRL(pitn) |= ( PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK );   //ʹ�� PITn��ʱ��,����PITn�ж�
  
  NVIC_EnableIRQ((IRQn_Type)(pitn + 48));			      //���������ŵ�IRQ�ж�
}



//-------------------------------------------------------------------------*
//������: PIT0_interrupt
//��  ��: PIT�жϺ���
//��  ��: ��
//��  ��: ��
//��  ��: �ɳ�ʼ���������೤ʱ�����һ��
//-------------------------------------------------------------------------*
void PIT0_IRQHandler()
{
  PIT_Flag_Clear(PIT0);       //���жϱ�־λ
  
}
void PIT1_IRQHandler()
{
  PIT_Flag_Clear(PIT1);
  Speed_meauser();
  NVIC_DisableIRQ(PORTE_IRQn);
  hongwaiDat();
  NVIC_EnableIRQ(PORTE_IRQn);
  SpeedMeasureFlag=1;
  
}

void PIT2_IRQHandler()
{
  PIT_Flag_Clear(PIT1);       //���жϱ�־λ
  /*�û�����������*/
}

void PIT3_IRQHandler()
{
  PIT_Flag_Clear(PIT3);       //���жϱ�־λ
  /*�û�����������*/
}

