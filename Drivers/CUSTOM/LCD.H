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



#endif /* CUSTOM_LCD_H_ */
