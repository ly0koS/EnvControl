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
	u16	wramcmd;		//Prepare Write GRAM Command
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
void LCD_DrawPoint(u16 x,u16 y);
void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color);
void LCD_Clear(u16 Color);
void LCD_DisplayOn(void);													//开显示
void LCD_DisplayOff(void);
void LCD_D_Cirncle(u16 x0,u16 y0,u8 r);						 				//画圆
void LCD_D_Line(u16 x1, u16 y1, u16 x2, u16 y2);							//画线
void LCD_D_Rectangle(u16 x1, u16 y1, u16 x2, u16 y2);		   				//画矩形
void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color);		   				//填充单色
void LCD_C_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 *color);				//填充指定颜色
void LCD_DisplayChar(u16 x,u16 y,u8 num,u8 size,u8 mode);						//显示一个字符
void LCD_DisplayNum(u16 x,u16 y,u32 num,u8 len,u8 size);  						//显示一个数字
void LCD_DisplayxNum(u16 x,u16 y,u32 num,u8 len,u8 size,u8 mode);				//显示 数字
void LCD_DisplayString(u16 x,u16 y,u16 width,u16 height,u8 size,u8 *p);		//显示一个字符串,12/16字体
								//设置屏扫描方向
void LCD_Display_Dir(u8 dir);								//设置屏幕显示方向

/*自定义结束*/
#define SSD_HOR_RESOLUTION		800		//LCD水平分辨率
#define SSD_VER_RESOLUTION		480		//LCD垂直分辨率
//LCD驱动参数设置
#define SSD_HOR_PULSE_WIDTH		1		//水平脉宽
#define SSD_HOR_BACK_PORCH		46		//水平前廊
#define SSD_HOR_FRONT_PORCH		210		//水平后廊

#define SSD_VER_PULSE_WIDTH		1		//垂直脉宽
#define SSD_VER_BACK_PORCH		23		//垂直前廊
#define SSD_VER_FRONT_PORCH		22		//垂直前廊
//如下几个参数，自动计算
#define SSD_HT	(SSD_HOR_RESOLUTION+SSD_HOR_BACK_PORCH+SSD_HOR_FRONT_PORCH)
#define SSD_HPS	(SSD_HOR_BACK_PORCH)
#define SSD_VT 	(SSD_VER_RESOLUTION+SSD_VER_BACK_PORCH+SSD_VER_FRONT_PORCH)
#define SSD_VPS (SSD_VER_BACK_PORCH)
#endif /* CUSTOM_LCD_H_ */
