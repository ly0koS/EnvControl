/*
 * LCD.H
 *
 *  Created on: 2019年4月30日
 *      Author: ly0ko
 */

#ifndef CUSTOM_LCD_H_
#define CUSTOM_LCD_H_
#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "FONT.H"
/*Begin Variable typedef*/
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;
/*End Variable typedef*/

/*Begin Scan Direction*/
#define L2R_U2D  0 //Left to Right & Up to Down
#define L2R_D2U  1 //Left to Right & Down to Up
#define R2L_U2D  2 //Right to Left & Up to Down
#define R2L_D2U  3 //Right to Left & Down to Up

#define U2D_L2R  4 //Up to Down & Left to Right
#define U2D_R2L  5 //Up to Down & Right to Left
#define D2U_L2R  6 //Down to Up & Left to Right
#define D2U_R2L  7 //Down to Up & Right to Left

#define DFT_SCAN_DIR  L2R_U2D 	//Default Scan Direction
/*END Scan Direction*/
/*颜色声明开始*/
//画笔颜色
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000
#define BLUE         	 0x001F
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //棕色
#define BRRED 			 0XFC07 //棕红色
#define GRAY  			 0X8430 //灰色
//GUI颜色

#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色
#define GRAYBLUE       	 0X5458 //灰蓝色
/*颜色声明结束*/

/*Begin RS Control*/
typedef struct
{
	vu16 LCD_REG;
	vu16 LCD_RAM;
} LCD_TypeDef;

#define LCD_BASE        ((u32)(0x6C000000 | 0x0000007E))
#define LCD             ((LCD_TypeDef *) LCD_BASE)

/*End RS Control
 *LCD->LCD_REG = RS=0
 *LCD->LCD_RAM = RS=1  */

typedef struct
{
	u16 width;
	u16 height;
	u16 id;
	u8  dir;
	u16	warmcmd;		//Prepare Write GRAM Command
	u16 setxcmd;		//Set X axis
	u16 setycmd;		//Set Y axis
}_lcd_control;

extern _lcd_control lcdcon;
/*自定义开始*/
void LCD_W_REGNUM(vu16 regvalue);
void LCD_W_REG(vu16 LCD_REG_NUM,vu16 LCD_REG_VAL);
void LCD_W_DAT(vu16 data);
u16 LCD_R_DAT();
u16 LCD_R_REG(vu16 LCD_REG_NUM);
void LCD_W_GRAM_Prepare();
void LCD_W_GRAM(u16 RGB_Code);
void LCD_SetCursor(u16 XPOS,u16 YPOS);
void LCD_DrawPoint(u16 x,u16 y,u16 point_color);
/*自定义结束*/

#endif /* CUSTOM_LCD_H_ */
