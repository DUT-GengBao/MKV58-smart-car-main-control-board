#ifndef _CONTROL_H_
#define _CONTROL_H_
////���嵱ǰ���ö���Ĳ���
//#define     Servo_Left        2300 //��ת���ޣ�ÿ�������ֵ��ͬ
//#define     Servo_Middle      1700 //����λ�ã�ÿ�������ֵ��ͬ
//#define     Servo_Right       460 //��ת���ޣ�ÿ�������ֵ��ͬ
extern int LF;
extern int LB;
extern int Dir;
extern unsigned int angle;
extern char speedsent[];
extern  char speed[6];
extern  int hongwaidat[16];
extern u8 SpeedMeasureFlag;
void Speed_meauser(void);
void Measure_Init(void);
void Servo_Ctrl(u32 duty);
void Servo_Init(void);
void hongwai_Init(void);
void hongwaiDat(void);
#endif /* _CONTROL_H_ */