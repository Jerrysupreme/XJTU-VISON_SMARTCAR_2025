#include "data.h"

/* 循迹参数 */
int16 path_err;
int16 path[MT9V03X_H][2] = {0};	// 路径线x、y坐标

/* 循线 */
float path_linear_speed_target = 6;	// 循迹线速度
int16 path_start = 10;			// 路径线寻找开始高度
int16 path_end = 70;			// 路径线寻找结束高度
int16 control_point = 40;		// 控制点高度（速度：3：30 速度：6：50 速度：8：55）
int16 prediction_point = 50;	// 预测点高度：其横坐标将作为下一帧的搜线起点
int16 detection_box_center_limit = 80;	// 摄像头识别框中心误差阈值