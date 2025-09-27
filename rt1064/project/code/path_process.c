/*
This document is used for vehicle lane following control

API:
****User****
Lane following control initialization
 get_err
************

****Low-level****
Path line scanning
************
*/

#include "path_process.h"
#include "data.h"

/* Lane following control initialization */
void path_process_init(void)
{
	mt9v03x_init();
}

/* path line scanning */
void path_search(void)
{
	int16 x,y;
	static int16 num = 0;
	if(num < 10)
	{
		path[0][0] = MT9V03X_W/2;
		num++;
	}
	else
	{
		path[0][0] = path[prediction_point][0];
	}
	path[0][1] = MT9V03X_H-1-path_start;
	for(y = MT9V03X_H-1-path_start-1;y >= MT9V03X_H-1-path_end;y--)
	{
		for(x = path[MT9V03X_H-1-path_start-y-1][0];x < MT9V03X_W;x++)
		{
			if(image_OTSU[y][x] == 0)
			{
				path[MT9V03X_H-1-path_start-y][0] = x;
				break;
			}
			if(x == MT9V03X_W-1)
			{
				path[MT9V03X_H-1-path_start-y][0] = x;
			}
		}
		for(x = path[MT9V03X_H-1-path_start-y-1][0];x >= 0;x--)
		{
			if(image_OTSU[y][x] == 0)
			{
				path[MT9V03X_H-1-path_start-y][0] += x;
				break;
			}
			if(x == 0)
			{
				path[MT9V03X_H-1-path_start-y][0] += x;
			}
		}
		path[MT9V03X_H-1-path_start-y][0] = path[MT9V03X_H-1-path_start-y][0]/2;
		path[MT9V03X_H-1-path_start-y][1] = y;
	}
}
void get_err(void)
{
path_err = path[control_point - 10][0] - MT9V03X_W/2;
}