/*
 * LCD.h
 *
 *  Created on: Feb 18, 2018
 *      Author: ZBT
 */

#ifndef USER_DRIVER_LCD_LCD_H_
#define USER_DRIVER_LCD_LCD_H_



#define COLOR_RED  	0xf800
#define COLOR_GREEN	0x07e0
#define COLOR_BLUE 	0x001f
#define COLOR_WHITE	0xffff
#define COLOR_BLACK	0x0000
#define COLOR_YELLOW    0xFFE0
#define COLOR_NAVY      0x000F
#define COLOR_DGREEN    0x03E0
#define COLOR_DCYAN     0x03EF
#define COLOR_MAROON    0x7800
#define COLOR_PURPLE    0x780F
#define COLOR_LGRAY     0xC618
#define COLOR_DGRAY     0x7BEF
#define COLOR_CYAN      0x07FF
#define COLOR_GRAY0     0xEF7D   	//灰色0 3165 00110 001011 00101
#define COLOR_GRAY1     0x8410      	//灰色1      00000 000000 00000
#define COLOR_GRAY2     0x4208      	//灰色2  1111111111011111

//位域
#define GET_BITFIELD(addr) (volatile bit_field *)(&addr)//返回的是指针
typedef struct bit_S
{
	unsigned char bit0:1;
	unsigned char bit1:1;
	unsigned char bit2:1;
	unsigned char bit3:1;
	unsigned char bit4:1;
	unsigned char bit5:1;
	unsigned char bit6:1;
	unsigned char bit7:1;
}bit_field;

//硬件底层
void LCD_Reset();
void LCD_init(unsigned char direction);
void LCD_PORT_init(void);
void write_command(unsigned char c);
void LCD_write_data(unsigned char  d);
void write_word(unsigned int dat);

void RamAdressSet();
void LCD_SetPos(unsigned int x0,unsigned int x1,unsigned int y0,unsigned int y1);


void LCD_PutChar(unsigned int x, unsigned int y,unsigned char c, unsigned int fColor, unsigned int bColor);
void LCD_PutString(unsigned int x, unsigned int y, unsigned char *s, unsigned int fColor, unsigned int bColor);
void LCD_PutChar8x16(unsigned int x, unsigned int y,unsigned char c, unsigned int fColor, unsigned int bColor);
void PutGB3232(unsigned int x, unsigned int  y, unsigned char c[2], unsigned int fColor,unsigned int bColor);
void LCD_Put_Unsigned_Int(unsigned int x,unsigned int y,unsigned char s0[],unsigned int d,unsigned int fColor, unsigned int bColor);
void LCD_Put_Float(unsigned int x,unsigned int y,unsigned char s0[],float pnum,unsigned int fColor, unsigned int bColor);

void Disp_single_colour(unsigned int color);
void Draw_single_line(unsigned char axis,unsigned int line,unsigned int color);
extern const unsigned char logo_bit[9600];

//自建函数
void Gui_DrawPoint(unsigned int x,unsigned int y,unsigned int Color);
void Gui_Circle(u16 X,u16 Y,u16 R,u16 fc);
void Gui_DrawLine(u16 x0, u16 y0,u16 x1, u16 y1,u16 Color);
void Gui_box(u16 x, u16 y, u16 w, u16 h,u16 bc);
void Gui_box2(u16 x,u16 y,u16 w,u16 h, u8 mode);
void DisplayButtonDown(u16 x1,u16 y1,u16 x2,u16 y2);
void DisplayButtonUp(u16 x1,u16 y1,u16 x2,u16 y2);
void Gui_DrawFont_GBK16(u16 x, u16 y, u16 fc, u16 bc, u8 *s);
void Gui_DrawFont_GBK24(u16 x, u16 y, u16 fc, u16 bc, u8 *s);
void Gui_DrawFont_Num32(u16 x, u16 y, u16 fc, u16 bc, u16 num);
#endif /* USER_DRIVER_LCD_LCD_H_ */
