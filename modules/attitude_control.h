#ifndef _ATTITUDE_CONTROL
#define _ATTITUDE_CONTROL



#include "attitude_estimate.h"
#include <stm32f10x.h>



void TaskAttCtrl(void* pdata);

typedef struct
{
	float output[4];
	u16 motors[4];
}od;


#endif
