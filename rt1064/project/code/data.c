#include "data.h"

/* ѭ������ */
int16 path_err;
int16 path[MT9V03X_H][2] = {0};	// ·����x��y����

/* ѭ�� */
float path_linear_speed_target = 6;	// ѭ�����ٶ�
int16 path_start = 10;			// ·����Ѱ�ҿ�ʼ�߶�
int16 path_end = 70;			// ·����Ѱ�ҽ����߶�
int16 control_point = 40;		// ���Ƶ�߶ȣ��ٶȣ�3��30 �ٶȣ�6��50 �ٶȣ�8��55��
int16 prediction_point = 50;	// Ԥ���߶ȣ�������꽫��Ϊ��һ֡���������
int16 detection_box_center_limit = 80;	// ����ͷʶ������������ֵ