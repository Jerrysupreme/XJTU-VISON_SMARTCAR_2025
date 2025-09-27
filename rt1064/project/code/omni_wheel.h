#ifndef __OMNI_WHEEL_H
#define __OMNI_WHEEL_H
#include "zf_common_headfile.h"
#include "math.h"
// 机械约束（根据图1.2.1圆盘范围定义）
#define WHEEL_RADIUS        0.098145f    // 轮子中心到圆盘中心的距离(m)
//#define MAX_LINEAR_SPEED    1.0f     // 最大线速度(m/s)
//#define MAX_ANGULAR_SPEED   4.0f     // 最大角速度(rad/s)
#define M_PI   3.1415926f     //PI
// 轮子布局参数（车头在两个轮子之间）
#define WHEEL1_ANGLE_DEG    180.0f   // 轮子1与车头X轴夹角(deg)
#define WHEEL2_ANGLE_DEG    -60.0f   // 轮子2与车头X轴夹角
#define WHEEL3_ANGLE_DEG    60.0f    // 轮子3与车头X轴夹角
// 编码器参数
#define ENCODER_RESOLUTION  7400     // 编码器每转脉冲数
//#define GEAR_RATIO          30.0f    // 电机减速比
#define WHEEL_PERIMETER     0.175929f   // 轮子周长(m)
// 运动模式
typedef enum {
    MODE_STOP,
    MODE_FORWARD,
    MODE_BACKWARD,
    MODE_LEFT,
    MODE_RIGHT,
    MODE_ROTATE_CW,
    MODE_ROTATE_CCW,
	MODE_AIM,
	MODE_TRACK
} MotionMode;
// 设置目标运动
void InverseKinematics(float vx, float vy, float omega, float* wheel_speeds);
void SetMotion(MotionMode mode, float speed, float gyro, float yaw);
float speed_M2R(int32 motor);
int32 speed_R2M(float real);
void ForwardKinematics(float* wheel_speeds, float* result);
void Fast_ForwardKinematics(float* wheel_speeds, float* result);
//void SetCustomMotion(float vx, float vy, float omega);
#endif