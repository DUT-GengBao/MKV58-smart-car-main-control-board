#ifndef _CONTROL_H_
#define _CONTROL_H_
////定义当前所用舵机的参数
//#define     Servo_Left        2300 //左转极限，每个舵机数值不同
//#define     Servo_Middle      1700 //中心位置，每个舵机数值不同
//#define     Servo_Right       460 //右转极限，每个舵机数值不同
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