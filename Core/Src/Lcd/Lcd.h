/*
 * LcdTask.h
 *
 *  Created on: Sep 9, 2019
 *      Author: fil
 */

#ifndef LCDTASK_H_
#define LCDTASK_H_
#include "main.h"

#define	ZERO_BRIGHTNESS	20
#define	HALF_BRIGHTNESS	500
#define	FULL_BRIGHTNESS	1000
extern	void LcdInit(void);
extern	void LcdSetBrightness(uint16_t brightness);


#endif /* LCDTASK_H_ */
