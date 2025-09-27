#ifndef _DATA_H_
#define _DATA_H_

#include "zf_common_headfile.h"
/* 屏幕类型 */
#define SCREEN_KIND 1	// 屏幕选择（ 0:tft180 1:ips200 ）
/*********************************************************************/


/****************************** 图像处理 ******************************/

/**********************************************************************/

/* processed picture */
extern uint8 image_OTSU[MT9V03X_H][MT9V03X_W];

/* PID error */
extern int16 path_err;

/* way point */
extern int16 path[MT9V03X_H][2];	//way point x y coordinate 

/* tracking */
extern float path_linear_speed_target;	// track speed
extern int16 path_start;	// path start height
extern int16 path_end;	// path end height
extern int16 control_point;	// control point height (speed 3 30 speed 8 45)
extern int16 prediction_point;	// prdiction height 
#endif