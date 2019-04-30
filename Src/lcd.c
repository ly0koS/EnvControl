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
void LCD_W_RRAM_Prepare()
{
	LCD->LCD_REG=lcdcon.warmcmd;
}
/*End Preparation For Writing GRAM*/

/*Begin Writing GRAM*/
void LCD_W_GRAM(u16 RGB_Code)
{

}
/*End Writing GRAM*/
/*End Custom Function*/
