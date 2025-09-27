#ifndef __TRACK_H
#define __TRACK_H
#include "zf_common_headfile.h"
#define GYRO_Z_FORWARD 1
#define AIM_X    155    // X瞄准值160
#define AIM_Y    178    // Y瞄准值
#define AIM_Y_ROUND    180    // Y瞄准值
#define CAMERA_W               (320)                                           
#define CAMERA_H               (240) 
#define FIND_BOX	135
typedef struct {
    float Kp, Ki, Kd;
	float Gd;
    float prev_error1; 
	float prev_error2;
} TRACK_PID;
typedef struct {
    float Kp, Ki, Kd;	
    float prev_error; 
	float target;
} G_PID;
typedef struct {
    float Kp, Ki, Kd;
    float prev_error; 
	float target;
} P_PID;
typedef struct {
    float Kp, Ki, Kd;
    float prev_error; 
}AIM_PID;
float track_pid_control(TRACK_PID *track,int16 path_err);
float G_pid_control(G_PID *move_z,float g_od);//Z轴位置环
float P_pid_control(P_PID *move_p,float p_od);//移动位置环
float aim_z_control(G_PID *aim_z,float x);//z瞄准位置环
float aim_x_control(AIM_PID *move_x,float x);//x瞄准位置环
float aim_y_control(AIM_PID *move_y,float y,float target_y);//y瞄准位置环
int G_control(G_PID *move_z,float g_od,float target);//串级z轴控制
int G_control_absolute(G_PID *move_z,float g_od,float target);//z轴绝对角度控制
int G_move_absolute(G_PID *move_z,float g_od,float target,float p_od,float p_target,float speed);//z轴绝对运动控制;//z轴绝对运动控制
int P_control(P_PID *move_p,float p_od,float yaw,float target);//串级位置控制
int P_contorl_simple(float speed,float p_od,float yaw,float target);//简易位置控制
void G_pid_control_set(float Kp,float Ki,float Kd);
void P_pid_control_set(float Kp,float Ki,float Kd);
void aim_x_pid_set(float Kp,float Ki,float Kd);
void aim_y_pid_set(float Kp,float Ki,float Kd);
void aim_z_pid_set(float Kp,float Ki,float Kd);
int check_box();//找箱子
void calculate_box();//计算箱子坐标
int aim_control();//瞄准控制
int aim_control_2();//瞄准控制track版
int z_control();//z瞄准控制
int push2r_control();//推向右侧
int push2l_control();//推向左侧
int push2r_control_dun();//推向右侧
int push2l_control_dun();//推向左侧
void track_control(float speed);
int track_control_2(); //纠正姿态
void track_pid_set(float Kp,float Ki,float Kd,float Gd);
int out_detect(int track_error); //出界检测
void out_process(); //出界处理
int z_rotate(float speed,float g_od,float target,int dir);//dir为1逆转，否则顺转
#endif