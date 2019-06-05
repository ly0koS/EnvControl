/*
 * lcd.c
 *
 *  Created on: 2019年4月30日
 *      Author: ly0ko
 */
#include "LCD.H"

_lcd_control lcdcon;

/*Begin Custom Function*/

/*Begin Write Register Number*/
void LCD_W_REGNUM(vu16 regvalue)
{
	regvalue=regvalue;
	LCD->LCD_REG=regvalue;			//Choose #regvalue Register
}
/*End Write Register Number*/

/*Begin Write Register*/
void LCD_W_REG(vu16 LCD_REG_NUM,vu16 LCD_REG_VAL)
{
	LCD->LCD_REG=LCD_REG_NUM;		//Choose #LCD_REG_NUM Register
	LCD->LCD_RAM=LCD_REG_VAL;		//Write DATA=LED_REG_VAL
}
/*End Write Register*/

/*Begin Write DATA */
void LCD_W_DAT(vu16 data)
{
	data=data;
	LCD->LCD_RAM=data;
}
/*End Write DATA */

/*Begin Read DATA */
u16 LCD_R_DAT()
{
	vu16 ram;
	ram=LCD->LCD_RAM;
	return ram;
}
/*End Read DATA */

/*Begin Read Register Data */
u16 LCD_R_REG(vu16 LCD_REG_NUM)
{
	LCD_W_REGNUM(LCD_REG_NUM);		//Write The Register Number Which we want to Read
	HAL_Delay(5);					//Delay 5 us
	return LCD_R_DAT();
}
/*End Read Register Data*/

/*Begin Preparation For Writing GRAM*/
void LCD_W_GRAM_Prepare()
{
	LCD->LCD_REG=lcdcon.warmcmd;
}
/*End Preparation For Writing GRAM*/

/*Begin Writing GRAM*/
void LCD_W_GRAM(u16 RGB_Code)
{
	LCD->LCD_RAM=RGB_Code;
}
/*End Writing GRAM*/

/*Begin Set Cursor*/
void LCD_SetCursor(u16 XPOS,u16 YPOS)
{
	if(lcdcon.id==0x9241||lcdcon.id==0x5310)
	{
		LCD_W_REGNUM(lcdcon.setxcmd);
		LCD_W_DAT(XPOS>>8);
		LCD_W_DAT(XPOS&0xFF);
		LCD_W_REGNUM(lcdcon.setycmd);
		LCD_W_DAT(YPOS>>8);
		LCD_W_DAT(YPOS&0xFF);
	}
	else if(lcdcon.id==0x6804)
	{
		if(lcdcon.dir==1)					//landscape
			XPOS=lcdcon.width-1-XPOS;
		LCD_W_REGNUM(lcdcon.setxcmd);
		LCD_W_DAT(XPOS>>8);
		LCD_W_DAT(XPOS&&0xFF);
		LCD_W_REGNUM(lcdcon.setycmd);
		LCD_W_DAT(YPOS>>8);
		LCD_W_DAT(YPOS&0xFF);
	}
	else if(lcdcon.id==0x5510)
	{
		LCD_W_REGNUM(lcdcon.setxcmd);
		LCD_W_DAT(XPOS>>8);
		LCD_W_REGNUM(lcdcon.setxcmd+1);
		LCD_W_DAT(XPOS&&0xFF);
		LCD_W_REGNUM(lcdcon.setycmd);
		LCD_W_DAT(YPOS>>8);
		LCD_W_REGNUM(lcdcon.setycmd+1);
		LCD_W_DAT(YPOS&0xFF);
	}
	else
	{
		if(lcdcon.dir==1)
			XPOS=lcdcon.width-1-XPOS;
		LCD_W_REG(lcdcon.setxcmd,XPOS);
		LCD_W_REG(lcdcon.setycmd,YPOS);
	}
}
/*End Set Cursor*/

/*Begin DrawPoint*/
void LCD_DrawPoint(u16 x,u16 y,u16 point_color)
{
	LCD_SetCursor(x,y);
	LCD_W_GRAM_Prepare();
	LCD->LCD_RAM=point_color;
}
/*End DrawPoint*/

/*End Custom Function*/
void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color)
{
	if(lcdcon.id==0X9341||lcdcon.id==0X5310)
	{
		LCD_W_REGNUM(lcdcon.setxcmd);
		LCD_W_DAT(x>>8);LCD_W_DAT(x&0XFF);
		LCD_W_REGNUM(lcdcon.setycmd);
		LCD_W_DAT(y>>8);LCD_W_DAT(y&0XFF);
	}else if(lcdcon.id==0X5510)
	{
		LCD_W_REGNUM(lcdcon.setxcmd);LCD_W_DAT(x>>8);
		LCD_W_REGNUM(lcdcon.setxcmd+1);LCD_W_DAT(x&0XFF);
		LCD_W_REGNUM(lcdcon.setycmd);LCD_W_DAT(y>>8);
		LCD_W_REGNUM(lcdcon.setycmd+1);LCD_W_DAT(y&0XFF);
	}else if(lcdcon.id==0X1963)
	{
		if(lcdcon.dir==0)x=lcdcon.width-1-x;
		LCD_W_REGNUM(lcdcon.setxcmd);
		LCD_W_DAT(x>>8);LCD_W_DAT(x&0XFF);
		LCD_W_DAT(x>>8);LCD_W_DAT(x&0XFF);
		LCD_W_REGNUM(lcdcon.setycmd);
		LCD_W_DAT(y>>8);LCD_W_DAT(y&0XFF);
		LCD_W_DAT(y>>8);LCD_W_DAT(y&0XFF);
	}else if(lcdcon.id==0X6804)
	{
		if(lcdcon.dir==1)x=lcdcon.width-1-x;//横屏时处理
		LCD_W_REGNUM(lcdcon.setxcmd);
		LCD_W_DAT(x>>8);LCD_W_DAT(x&0XFF);
		LCD_W_REGNUM(lcdcon.setycmd);
		LCD_W_DAT(y>>8);LCD_W_DAT(y&0XFF);
	}else
	{
 		if(lcdcon.dir==1)x=lcdcon.width-1-x;//横屏其实就是调转x,y坐标
		LCD_W_REG(lcdcon.setxcmd,x);
		LCD_W_REG(lcdcon.setycmd,y);
	}
	LCD->LCD_REG=lcdcon.warmcmd;
	LCD->LCD_RAM=color;
}
