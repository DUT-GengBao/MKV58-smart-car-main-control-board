/*
 * LCD.c
 *
 *  Created on: Feb 18, 2018
 *      Author: ZBT
 */

#include "include.h"
#include "LCD.h"
#include "Font.h"	//32*32汉字字模
#include "GUI.h"
#include "logo.h"

#define USE_HARDWARE_SPI 1
void LCD_PORT_init()
{
  GPIO_Init(LCD_RST_PORT, GPO, HIGH);//rst
  GPIO_Init(LCD_RS_PORT, GPO, HIGH);//dc
#if USE_HARDWARE_SPI
  spi_init(LCD_SPIn,LCD_DC_PCSn,MASTER,100*1000*1000);//12500*1000);LCD_OTH_PCSn | 
#else
  GPIO_Init(LCD_SDA_PORT, GPO, HIGH);//sda
  GPIO_Init(LCD_SCL_PORT, GPO, HIGH);//scl
#endif
  
  LCD_init(1);
}

/************************************************
*  函数名称：LCD_Reset
*  功能说明：240*320的SPI LCD复位
*  参数说明：无
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void LCD_Reset()
{
  reset = 0;
  time_delay_ms(20);
  reset = 1;
  time_delay_ms(20);
}


/************************************************
*  函数名称：LCD_Reset
*  功能说明：240*320的SPI LCD模块初始化
*  参数说明：direction取TRUE和FALSE，屏幕正反之分
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void LCD_init(unsigned char direction)
{
  //先端口输入输出配置
  reset=0;
  time_delay_ms(20);
  reset=1;
  time_delay_ms(20);
  //------------------------------------------------------------------//
  //-------------------Software Reset-------------------------------//
  write_command(0xCB);
  LCD_write_data(0x39);
  LCD_write_data(0x2C);
  LCD_write_data(0x00);
  LCD_write_data(0x34);
  LCD_write_data(0x02);

  write_command(0xCF);
  LCD_write_data(0x00);
  LCD_write_data(0XC1);
  LCD_write_data(0X30);

  write_command(0xE8);
  LCD_write_data(0x85);
  LCD_write_data(0x00);
  LCD_write_data(0x78);

  write_command(0xEA);
  LCD_write_data(0x00);
  LCD_write_data(0x00);

  write_command(0xED);
  LCD_write_data(0x64);
  LCD_write_data(0x03);
  LCD_write_data(0X12);
  LCD_write_data(0X81);

  write_command(0xF7);
  LCD_write_data(0x20);

  write_command(0xC0);    //Power control
  LCD_write_data(0x23);   //VRH[5:0]

  write_command(0xC1);    //Power control
  LCD_write_data(0x10);   //SAP[2:0];BT[3:0]

  write_command(0xC5);    //VCM control
  LCD_write_data(0x3e); //对比度调节
  LCD_write_data(0x28);

  write_command(0xC7);    //VCM control2
  LCD_write_data(0x86);  //--

  write_command(0x36);    // Memory Access Control
  if(direction)
    LCD_write_data(0x28); //C8	   //48 68竖屏//28 E8 横屏
  else
    LCD_write_data(0x48);//0b00101000


  write_command(0x3A);
  LCD_write_data(0x55);

  write_command(0xB1);
  LCD_write_data(0x00);
  LCD_write_data(0x18);

  write_command(0xB6);    // Display Function Control
  LCD_write_data(0x08);
  LCD_write_data(0x82);
  LCD_write_data(0x27);

  write_command(0xF2);    // 3Gamma Function Disable
  LCD_write_data(0x00);

  write_command(0x26);    //Gamma curve selected
  LCD_write_data(0x01);

  write_command(0xE0);    //Set Gamma
  LCD_write_data(0x0F);
  LCD_write_data(0x31);
  LCD_write_data(0x2B);
  LCD_write_data(0x0C);
  LCD_write_data(0x0E);
  LCD_write_data(0x08);
  LCD_write_data(0x4E);
  LCD_write_data(0xF1);
  LCD_write_data(0x37);
  LCD_write_data(0x07);
  LCD_write_data(0x10);
  LCD_write_data(0x03);
  LCD_write_data(0x0E);
  LCD_write_data(0x09);
  LCD_write_data(0x00);

  write_command(0XE1);    //Set Gamma
  LCD_write_data(0x00);
  LCD_write_data(0x0E);
  LCD_write_data(0x14);
  LCD_write_data(0x03);
  LCD_write_data(0x11);
  LCD_write_data(0x07);
  LCD_write_data(0x31);
  LCD_write_data(0xC1);
  LCD_write_data(0x48);
  LCD_write_data(0x08);
  LCD_write_data(0x0F);
  LCD_write_data(0x0C);
  LCD_write_data(0x31);
  LCD_write_data(0x36);
  LCD_write_data(0x0F);

  write_command(0x11);    //Exit Sleep
  time_delay_ms(50);

  write_command(0x29);    //Display on
  //write_command(0x2c);
  rs = 1;
  time_delay_ms(5);
}

/************************************************
*  函数名称：write_command
*  功能说明：LCD写指令函数
*  参数说明：c为指令
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void  write_command(unsigned char  c)
{

#if USE_HARDWARE_SPI
  spi_mosi(LCD_SPIn,LCD_DC_PCSn,&c,NULL,1);
#else
  rs=0;
  sda=(GET_BITFIELD(c))->bit7;scl=0;scl=1;
  sda=(GET_BITFIELD(c))->bit6;scl=0;scl=1;
  sda=(GET_BITFIELD(c))->bit5;scl=0;scl=1;
  sda=(GET_BITFIELD(c))->bit4;scl=0;scl=1;
  sda=(GET_BITFIELD(c))->bit3;scl=0;scl=1;
  sda=(GET_BITFIELD(c))->bit2;scl=0;scl=1;
  sda=(GET_BITFIELD(c))->bit1;scl=0;scl=1;
  sda=(GET_BITFIELD(c))->bit0;scl=0;scl=1;
  rs=1;
#endif
}

/************************************************
*  函数名称：write_data
*  功能说明：LCD写数据函数
*  参数说明：d为数据，为一个BYTE的数据
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void  LCD_write_data(unsigned char  d)
{
#if USE_HARDWARE_SPI
  spi_mosi(LCD_SPIn,LCD_OTH_PCSn,&d,NULL,1);
#else
  rs=1;
  sda=(GET_BITFIELD(d))->bit7;scl=0;scl=1;
  sda=(GET_BITFIELD(d))->bit6;scl=0;scl=1;
  sda=(GET_BITFIELD(d))->bit5;scl=0;scl=1;
  sda=(GET_BITFIELD(d))->bit4;scl=0;scl=1;
  sda=(GET_BITFIELD(d))->bit3;scl=0;scl=1;
  sda=(GET_BITFIELD(d))->bit2;scl=0;scl=1;
  sda=(GET_BITFIELD(d))->bit1;scl=0;scl=1;
  sda=(GET_BITFIELD(d))->bit0;scl=0;scl=1;
#endif
}

/************************************************
*  函数名称：write_word
*  功能说明：LCD写数据函数
*  参数说明：dat为数据，为两个BYTE的数据
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void write_word(unsigned int dat)
{
  LCD_write_data(dat>>8);
  LCD_write_data(dat);
}

/************************************************
*  函数名称：RamAdressSet
*  功能说明：LCD的设置RAM地址
*  参数说明：无
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void  RamAdressSet()
{
  write_command(0x2A);
  LCD_write_data(0x00);
  LCD_write_data(0x00);
  LCD_write_data(0x00);
  LCD_write_data(0xEF);
  write_command(0x2B);
  LCD_write_data(0x00);
  LCD_write_data(0x00);
  LCD_write_data(0x01);
  LCD_write_data(0x3F);
}


/************************************************
*  函数名称：LCD_SetPos
*  功能说明：LCD的设置写入屏幕的地址
*  参数说明：横坐标X0~X1，纵坐标为Y0~Y1
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void LCD_SetPos(unsigned int x0,unsigned int x1,unsigned int y0,unsigned int y1)
{
  write_command(0x2A);
  LCD_write_data(x0>>8);
  LCD_write_data(x0);
  LCD_write_data(x1>>8);
  LCD_write_data(x1);

  write_command(0x2B);
  LCD_write_data(y0>>8);
  LCD_write_data(y0);
  LCD_write_data(y1>>8);
  LCD_write_data(y1);
  write_command(0x2c);
}

/************************************************
*  函数名称：LCD_PutChar
*  功能说明：LCD的写一个Char，默认为8*16的字模
*  参数说明：横坐标x，纵坐标为y，c为数据，fColor为字颜色，bColor为背景颜色
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void LCD_PutChar(unsigned int x, unsigned int y,unsigned char c, unsigned int fColor, unsigned int bColor)
{
  LCD_PutChar8x16( x, y, c, fColor, bColor );
}


/************************************************
*  函数名称：LCD_PutString
*  功能说明：LCD的写字符串
*  参数说明：横坐标x，纵坐标为y，s为字符串指针，fColor为字颜色，bColor为背景颜色
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void LCD_PutString(unsigned int x, unsigned int y, unsigned char *s, unsigned int fColor, unsigned int bColor)
{
  unsigned char l=0;
  while(*s) {
    if( *s < 0x80)
    {
      LCD_PutChar(x+l*8,y,*s,fColor,bColor);
      s++;l++;
    }
    else
    {
      PutGB3232(x+l*8,y,(unsigned char*)s,fColor,bColor);
      s+=2;l+=2;
    }
  }
}


/************************************************
*  函数名称：LCD_PutChar8x16
*  功能说明：LCD的写字符串8*16字模
*  参数说明：横坐标x，纵坐标为y，c为ASCII码，fColor为字颜色，bColor为背景颜色
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void LCD_PutChar8x16(unsigned int x, unsigned int y,unsigned char c, unsigned int fColor, unsigned int bColor)
{
  unsigned int i,j;
  LCD_SetPos(x,x+8-1,y,y+16-1);
  for(i=0; i<16;i++)
  {
    unsigned char m=Font8x16[c*16+i];
    for(j=0;j<8;j++)
    {
      if((m&0x80)==0x80)
        write_word(fColor);
      else
        write_word(bColor);
      m<<=1;
    }
  }
}

/************************************************
*  函数名称：PutGB3232
*  功能说明：LCD的写汉字32*32字模
*  参数说明：横坐标x，纵坐标为y，c[2]为汉子的机内码，fColor为字颜色，bColor为背景颜色
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*  备注    ：需要自建汉子库codeGB_32
*************************************************/
void PutGB3232(unsigned int x, unsigned int  y, unsigned char c[2], unsigned int fColor,unsigned int bColor)
{
  unsigned int i,j,k;
  LCD_SetPos(x,  x+32-1,y, y+32-1);
  for (k=0;k<15;k++) { //15标示自建汉字库中的个数，循环查询内码
    if ((codeGB_32[k].Index[0]==c[0])&&(codeGB_32[k].Index[1]==c[1])){
      for(i=0;i<128;i++) {
        unsigned short m=codeGB_32[k].Msk[i];
        for(j=0;j<8;j++) {
          if((m&0x80)==0x80) {
            write_word(fColor);
          }
          else {
            write_word(bColor);
          }
          m<<=1;
        }
      }
    }
  }
}


/************************************************
*  函数名称：Disp_single_colour
*  功能说明：LCD的刷屏函数
*  参数说明：Color为背景颜色
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void Disp_single_colour(unsigned int color)
{
  unsigned int i,j;
  LCD_SetPos(0,320-1,0,240-1);
  for (i=0;i<240;i++)
    for (j=0;j<320;j++)
      write_word(color);
}


/************************************************
*  函数名称：Draw_single_line
*  功能说明：LCD的画单线函数
*  参数说明：axis为行或者列，取'H'或者'L'，line为第几行或者第几列，Color为颜色
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void Draw_single_line(unsigned char axis,unsigned int line,unsigned int color)
{
  unsigned int i;
  if(axis=='L')
  {
    LCD_SetPos(line,line,0,240-1);
    for (i=0;i<240;i++)
      write_word(color);
  }
  else if(axis=='H')
  {
    LCD_SetPos(0,320-1,line,line);
    for (i=0;i<320;i++)
      write_word(color);
  }
}


/*
*以下为自加函数
*画点及画图函数。
*
*
*——ZBT
*/
void Gui_DrawPoint(unsigned int x,unsigned int y,unsigned int Color)
{
  LCD_SetPos(x,x,y,y);
  write_word(Color);
}

void Gui_Circle(u16 X,u16 Y,u16 R,u16 fc)
{//Bresenham算法
  unsigned short  a,b;
  int c;
  a=0;
  b=R;
  c=3-2*R;
  while (a<b)
  {
    Gui_DrawPoint(X+a,Y+b,fc);     //        7
    Gui_DrawPoint(X-a,Y+b,fc);     //        6
    Gui_DrawPoint(X+a,Y-b,fc);     //        2
    Gui_DrawPoint(X-a,Y-b,fc);     //        3
    Gui_DrawPoint(X+b,Y+a,fc);     //        8
    Gui_DrawPoint(X-b,Y+a,fc);     //        5
    Gui_DrawPoint(X+b,Y-a,fc);     //        1
    Gui_DrawPoint(X-b,Y-a,fc);     //        4

    if(c<0) c=c+4*a+6;
    else
    {
      c=c+4*(a-b)+10;
      b-=1;
    }
    a+=1;
  }
  if (a==b)
  {
    Gui_DrawPoint(X+a,Y+b,fc);
    Gui_DrawPoint(X+a,Y+b,fc);
    Gui_DrawPoint(X+a,Y-b,fc);
    Gui_DrawPoint(X-a,Y-b,fc);
    Gui_DrawPoint(X+b,Y+a,fc);
    Gui_DrawPoint(X-b,Y+a,fc);
    Gui_DrawPoint(X+b,Y-a,fc);
    Gui_DrawPoint(X-b,Y-a,fc);
  }

}

//画线函数，使用Bresenham 画线算法
void Gui_DrawLine(u16 x0, u16 y0,u16 x1, u16 y1,u16 Color)
{
  int dx,             // difference in x's
  dy,             // difference in y's
  dx2,            // dx,dy * 2
  dy2,
  x_inc,          // amount in pixel space to move during drawing
  y_inc,          // amount in pixel space to move during drawing
  error,          // the discriminant i.e. error i.e. decision variable
  index;          // used for looping

  LCD_SetPos(x0,x0,y0,y0);
  dx = x1-x0;//计算x距离
  dy = y1-y0;//计算y距离

  if (dx>=0)
  {
    x_inc = 1;
  }
  else
  {
    x_inc = -1;
    dx    = -dx;
  }

  if (dy>=0)
  {
    y_inc = 1;
  }
  else
  {
    y_inc = -1;
    dy    = -dy;
  }

  dx2 = dx << 1;
  dy2 = dy << 1;

  if (dx > dy)//x距离大于y距离，那么每个x轴上只有一个点，每个y轴上有若干个点
  {//且线的点数等于x距离，以x轴递增画点
    // initialize error term
    error = dy2 - dx;

    // draw the line
    for (index=0; index <= dx; index++)//要画的点数不会超过x距离
    {
      //画点
      Gui_DrawPoint(x0,y0,Color);

      // test if error has overflowed
      if (error >= 0) //是否需要增加y坐标值
      {
        error-=dx2;

        // move to next line
        y0+=y_inc;//增加y坐标值
      } // end if error overflowed

      // adjust the error term
      error+=dy2;

      // move to the next pixel
      x0+=x_inc;//x坐标值每次画点后都递增1
    } // end for
  } // end if |slope| <= 1
  else//y轴大于x轴，则每个y轴上只有一个点，x轴若干个点
  {//以y轴为递增画点
    // initialize error term
    error = dx2 - dy;

    // draw the line
    for (index=0; index <= dy; index++)
    {
      // set the pixel
      Gui_DrawPoint(x0,y0,Color);

      // test if error overflowed
      if (error >= 0)
      {
        error-=dy2;

        // move to next line
        x0+=x_inc;
      } // end if error overflowed

      // adjust the error term
      error+=dx2;

      // move to the next pixel
      y0+=y_inc;
    } // end for
  } // end else |slope| > 1
}

void Gui_box(u16 x, u16 y, u16 w, u16 h,u16 bc)
{
  Gui_DrawLine(x,y,x+w,y,0xEF7D);
  Gui_DrawLine(x+w-1,y+1,x+w-1,y+1+h,0x2965);
  Gui_DrawLine(x,y+h,x+w,y+h,0x2965);
  Gui_DrawLine(x,y,x,y+h,0xEF7D);
  Gui_DrawLine(x+1,y+1,x+1+w-2,y+1+h-2,bc);
}
void Gui_box2(u16 x,u16 y,u16 w,u16 h, u8 mode)
{
  if (mode==0)	{
    Gui_DrawLine(x,y,x+w,y,0xEF7D);
    Gui_DrawLine(x+w-1,y+1,x+w-1,y+1+h,0x2965);
    Gui_DrawLine(x,y+h,x+w,y+h,0x2965);
    Gui_DrawLine(x,y,x,y+h,0xEF7D);
  }
  if (mode==1)	{
    Gui_DrawLine(x,y,x+w,y,0x2965);
    Gui_DrawLine(x+w-1,y+1,x+w-1,y+1+h,0xEF7D);
    Gui_DrawLine(x,y+h,x+w,y+h,0xEF7D);
    Gui_DrawLine(x,y,x,y+h,0x2965);
  }
  if (mode==2)	{
    Gui_DrawLine(x,y,x+w,y,0xffff);
    Gui_DrawLine(x+w-1,y+1,x+w-1,y+1+h,0xffff);
    Gui_DrawLine(x,y+h,x+w,y+h,0xffff);
    Gui_DrawLine(x,y,x,y+h,0xffff);
  }
}

/**************************************************************************************
功能描述: 在屏幕显示一凸起的按钮框
输    入: u16 x1,y1,x2,y2 按钮框左上角和右下角坐标
输    出: 无
**************************************************************************************/
void DisplayButtonDown(u16 x1,u16 y1,u16 x2,u16 y2)
{
  Gui_DrawLine(x1,  y1,  x2,y1, COLOR_GRAY2);  //H
  Gui_DrawLine(x1+1,y1+1,x2,y1+1, COLOR_GRAY1);  //H
  Gui_DrawLine(x1,  y1,  x1,y2, COLOR_GRAY2);  //V
  Gui_DrawLine(x1+1,y1+1,x1+1,y2, COLOR_GRAY1);  //V
  Gui_DrawLine(x1,  y2,  x2,y2, COLOR_WHITE);  //H
  Gui_DrawLine(x2,  y1,  x2,y2, COLOR_WHITE);  //V
}

/**************************************************************************************
功能描述: 在屏幕显示一凹下的按钮框
输    入: u16 x1,y1,x2,y2 按钮框左上角和右下角坐标
输    出: 无
**************************************************************************************/
void DisplayButtonUp(u16 x1,u16 y1,u16 x2,u16 y2)
{
  Gui_DrawLine(x1,  y1,  x2,y1, COLOR_WHITE); //H
  Gui_DrawLine(x1,  y1,  x1,y2, COLOR_WHITE); //V

  Gui_DrawLine(x1+1,y2-1,x2,y2-1, COLOR_GRAY1);  //H
  Gui_DrawLine(x1,  y2,  x2,y2, COLOR_GRAY2);  //H
  Gui_DrawLine(x2-1,y1+1,x2-1,y2, COLOR_GRAY1);  //V
  Gui_DrawLine(x2  ,y1  ,x2,y2, COLOR_GRAY2); //V
}

void Gui_DrawFont_GBK16(u16 x, u16 y, u16 fc, u16 bc, u8 *s)
{
  unsigned char i,j;
  unsigned short k,x0;
  x0=x;

  while(*s)
  {
    if((*s) < 128)
    {
      k=*s;
      if (k==13)
      {
        x=x0;
        y+=16;
      }
      else
      {
        if (k>32) k-=32; else k=0;

        for(i=0;i<16;i++)
          for(j=0;j<8;j++)
          {
            if(asc16[k*16+i]&(0x80>>j))	Gui_DrawPoint(x+j,y+i,fc);
            else
            {
              if (fc!=bc) Gui_DrawPoint(x+j,y+i,bc);
            }
          }
        x+=8;
      }
      s++;
    }

    else
    {
      for (k=0;k<hz16_num;k++)
      {
        if ((hz16[k].Index[0]==*(s))&&(hz16[k].Index[1]==*(s+1)))
        {
          for(i=0;i<16;i++)
          {
            for(j=0;j<8;j++)
            {
              if(hz16[k].Msk[i*2]&(0x80>>j))	Gui_DrawPoint(x+j,y+i,fc);
              else {
                if (fc!=bc) Gui_DrawPoint(x+j,y+i,bc);
              }
            }
            for(j=0;j<8;j++)
            {
              if(hz16[k].Msk[i*2+1]&(0x80>>j))	Gui_DrawPoint(x+j+8,y+i,fc);
              else
              {
                if (fc!=bc) Gui_DrawPoint(x+j+8,y+i,bc);
              }
            }
          }
        }
      }
      s+=2;x+=16;
    }
  }
}
unsigned short k;
void Gui_DrawFont_GBK24(u16 x, u16 y, u16 fc, u16 bc, u8 *s)
{
  unsigned char i,j;

  while(*s)
  {
    if( *s < 0x80 )
    {
      k=*s;
      if (k>32) k-=32; else k=0;

      for(i=0;i<16;i++)
        for(j=0;j<8;j++)
        {
          if(asc16[k*16+i]&(0x80>>j))
            Gui_DrawPoint(x+j,y+i,fc);
          else
          {
            if (fc!=bc) Gui_DrawPoint(x+j,y+i,bc);
          }
        }
      s++;x+=8;
    }
    else
    {

      for (k=0;k<hz24_num;k++)
      {
        if ((hz24[k].Index[0]==*(s))&&(hz24[k].Index[1]==*(s+1)))
        {
          for(i=0;i<24;i++)
          {
            for(j=0;j<8;j++)
            {
              if(hz24[k].Msk[i*3]&(0x80>>j))
                Gui_DrawPoint(x+j,y+i,fc);
              else
              {
                if (fc!=bc) Gui_DrawPoint(x+j,y+i,bc);
              }
            }
            for(j=0;j<8;j++)
            {
              if(hz24[k].Msk[i*3+1]&(0x80>>j))	Gui_DrawPoint(x+j+8,y+i,fc);
              else {
                if (fc!=bc) Gui_DrawPoint(x+j+8,y+i,bc);
              }
            }
            for(j=0;j<8;j++)
            {
              if(hz24[k].Msk[i*3+2]&(0x80>>j))
                Gui_DrawPoint(x+j+16,y+i,fc);
              else
              {
                if (fc!=bc) Gui_DrawPoint(x+j+16,y+i,bc);
              }
            }
          }
        }
      }
      s+=2;x+=24;
    }
  }
}
void Gui_DrawFont_Num32(u16 x, u16 y, u16 fc, u16 bc, u16 num)
{
  unsigned char i,j,k,c;
  for(i=0;i<32;i++)
  {
    for(j=0;j<4;j++)
    {
      c=*(sz32+num*32*4+i*4+j);
      for (k=0;k<8;k++)
      {
        if(c&(0x80>>k))	Gui_DrawPoint(x+j*8+k,y+i,fc);
        else {
          if (fc!=bc) Gui_DrawPoint(x+j*8+k,y+i,bc);
        }
      }
    }
  }
}
