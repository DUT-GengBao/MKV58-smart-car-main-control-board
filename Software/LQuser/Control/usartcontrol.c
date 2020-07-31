#include "include.h"
void int2str(int16 n, char *str)

{
  char buf[10] = "";
  int i = 0;
  int len = 0;
  int16 temp = n < 0 ? -n: n;  // temp为n的绝对值
  if (str == NULL)
  {
    return;
  }
   if (n==0)
  {
    *str=0;
    return;
  }
  while(temp)
  {
    buf[i++] = (temp % 10) + '0';  //把temp的每一位上的数存入buf
    temp = temp / 10;
  }
  len = n < 0 ? ++i: i;  //如果n是负数，则多需要一位来存储负号
  str[i] = 0;            //末尾是结束符0
  while(1)
  {
    i--;
    if (buf[len-i-1] ==0)
    {
      break;
    }
    str[i] = buf[len-i-1];  //把buf数组里的字符拷到字符串
  }
  if (i == 0 )
  {
    str[i] = '-';          //如果是负数，添加一个负号 
  }
}
