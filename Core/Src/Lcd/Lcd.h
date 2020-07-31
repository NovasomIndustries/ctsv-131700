/*
 * LcdTask.h
 *
 *  Created on: Sep 9, 2019
 *      Author: fil
 */

#ifndef LCDTASK_H_
#define LCDTASK_H_

#include "main.h"
#include "st7735.h"
#include "fonts.h"

#include <stdio.h>

typedef struct {
    uint8_t xpos;
	uint8_t	ypos;
    char line[32];
    uint16_t fore_color;
	uint16_t bkg_color;
} Video;

#define	ZERO_BRIGHTNESS	0
#define	LOW_BRIGHTNESS	30
#define	HALF_BRIGHTNESS	500
#define	FULL_BRIGHTNESS	1000
extern	void LcdInit(void);
extern void LcdWrite11x18(Video *data);
extern void LcdClearScreen(Video *data);
extern	void LcdSetBrightness(uint16_t brightness);

#endif /* LCDTASK_H_ */
