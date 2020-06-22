/*
 * LcdTask.c
 *
 *  Created on: Sep 9, 2019
 *      Author: fil
 */
#include "Lcd.h"

#include "main.h"

extern	TIM_HandleTypeDef 	htim16;

#define VMARGIN 1
#define HMARGIN (12*7)-1
uint8_t	horizontal_line_space;
uint8_t	vertical_line_space;
uint8_t	current_highlight_line;
#define	NUMLINES	7

Video VideoMem[NUMLINES]=
{
		{		0,
				0,
				"SurgyBlue Init OK",
				ST7735_RED,
				ST7735_BLACK,
		},
		{		0,
				1,
				"Line1 : ",
				ST7735_BLUE,
				ST7735_BLACK,
		},
		{		0,
				2,
				"Line2 : ",
				ST7735_BLUE,
				ST7735_BLACK,
		},
		{		0,
				3,
				"Line3 : ",
				ST7735_BLUE,
				ST7735_BLACK,
		},
		{		0,
				4,
				"Line4 : ",
				ST7735_BLUE,
				ST7735_BLACK,
		},
		{		0,
				5,
				"Line5 : ",
				ST7735_BLUE,
				ST7735_BLACK,
		},
		{		0,
				6,
				"Line6 : ",
				ST7735_BLUE,
				ST7735_BLACK,
		},
};

static void initVideo(void)
{
/*int i;
	horizontal_line_space = ST7735_GetFontWidth(Font_7x10);
	vertical_line_space = ST7735_GetFontHeigth(Font_7x10) + VMARGIN;
	for (i=0;i<NUMLINES;i++)
	{
		VideoMem[i].ypos *= vertical_line_space;
		ST7735_WriteString(VideoMem[i].xpos, VideoMem[i].ypos,VideoMem[i].line,Font_7x10,VideoMem[i].fore_color,VideoMem[i].bkg_color);
	}*/
	ST7735_WriteString(30, 30, "SurgyBlue", Font_11x18, ST7735_BLUE, ST7735_BLACK);
}

void LcdSetBrightness(uint16_t brightness)
{
	if ( brightness <= FULL_BRIGHTNESS)
		htim16.Instance->CCR1 = brightness;
}

void LcdInit(void)
{
	ST7735_Unselect();
	ST7735_Init();
    ST7735_FillScreen(ST7735_BLACK);
    LcdSetBrightness(HALF_BRIGHTNESS);
    initVideo();
	HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1);
}

void LcdWrite11x18(Video *wr_struct)
{
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_WriteString(wr_struct->xpos, wr_struct->ypos, wr_struct->line, Font_11x18, wr_struct->fore_color, wr_struct->bkg_color);
}
