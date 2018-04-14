#ifndef __REMOTE_ANALYSIS_H__
#define __REMOTE_ANALYSIS_H__

#include "main.h"

#define REMOTE_DATA_DEFAULT              {1024,1024,1024,1024,3,3};

#define RC_SWITCH_UP 1
#define RC_SWITCH_MIDDLE 3
#define RC_SWITCH_DOWN 2

typedef struct
{
	struct
	{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t switch_left;
		uint8_t switch_right;
	}rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	struct
	{
		uint8_t v_l;
		uint8_t v_h;
	}key;
}RC_Ctl_t;

#define RC_DATA_DEFAULT \
{\
	{1024,1024,1024,1024,3,3},\
	{0},\
	{0},\
}\

extern RC_Ctl_t RC_Ctl;

void RemoteData_analysis(uint8_t *djiData);

#endif

