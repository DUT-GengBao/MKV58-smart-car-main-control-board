#include "include.h"
void int2str(int16 n, char *str)

{
  char buf[10] = "";
  int i = 0;
  int len = 0;
  int16 temp = n < 0 ? -n: n;  // tempΪn�ľ���ֵ
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
    buf[i++] = (temp % 10) + '0';  //��temp��ÿһλ�ϵ�������buf
    temp = temp / 10;
  }
  len = n < 0 ? ++i: i;  //���n�Ǹ����������Ҫһλ���洢����
  str[i] = 0;            //ĩβ�ǽ�����0
  while(1)
  {
    i--;
    if (buf[len-i-1] ==0)
    {
      break;
    }
    str[i] = buf[len-i-1];  //��buf��������ַ������ַ���
  }
  if (i == 0 )
  {
    str[i] = '-';          //����Ǹ��������һ������ 
  }
}
