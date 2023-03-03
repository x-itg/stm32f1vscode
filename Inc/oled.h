
#ifndef __OLED_H
#define __OLED_H			  	 
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h" 
#include "stdlib.h"	    	
#include "string.h"


extern  I2C_HandleTypeDef hi2c1;



#define OLED_ADDRESS	0x78 
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据
#define Max_Column	128
#define DELAYUS 5


void ChangeShow2(unsigned short L0show,unsigned short R0show);
void OLED_Init(void);
void OLED_Set_Pos(unsigned char x,unsigned char y);
void OLED_Fill(unsigned char fill_Data);
void OLED_CLS(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_ShowStr(unsigned char x,unsigned char y,unsigned char ch[],unsigned char TextSize);
void OLED_ShowString(unsigned char x,unsigned char y, char *chr,unsigned char Char_Size);
 
 
void OLED_ShowCharNomAndRev(unsigned char x,unsigned char y,unsigned char chr,unsigned char rev);

#endif 