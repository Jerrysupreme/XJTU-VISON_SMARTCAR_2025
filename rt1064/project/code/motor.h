#ifndef __MOTOR_H
#define __MOTOR_H
#define MOTOR1_DIR               (C9 )
#define MOTOR1_PWM               (PWM2_MODULE1_CHA_C8)//双驱动不动

#define MOTOR2_DIR               (C7 )
#define MOTOR2_PWM               (PWM2_MODULE0_CHA_C6)

#define MOTOR3_DIR               (C10 )
#define MOTOR3_PWM               (PWM2_MODULE2_CHB_C11)

#define MOTOR4_DIR               (D2 )
#define MOTOR4_PWM               (PWM2_MODULE3_CHB_D3)

#define ENCODER_1                   (QTIMER1_ENCODER1)
#define ENCODER_1_A                 (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B                 (QTIMER1_ENCODER1_CH2_C1)

#define ENCODER_2                   (QTIMER1_ENCODER2)
#define ENCODER_2_A                 (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B                 (QTIMER1_ENCODER2_CH2_C24)

#define ENCODER_3                   (QTIMER2_ENCODER1)
#define ENCODER_3_A                 (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_3_B                 (QTIMER2_ENCODER1_CH2_C4)

#define ENCODER_4                   (QTIMER2_ENCODER2)
#define ENCODER_4_A                 (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_4_B                 (QTIMER2_ENCODER2_CH2_C25)

#define PIT_CH                      (PIT_CH0 ) //使用的周期中断编号,如果修改需要同步对应修改周期中断编号与isr.c中的调用
#define I_MAX 0.3*PWM_DUTY_MAX
#include "zf_common_headfile.h"
//@brief 电机数据结构
typedef struct {
    // 速度环PID参数
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float speed_error_sum;//采用增量式，该值不用
    float speed_output; //输出量out
    int32 out_increment;//增量式PID输出增量     
    int32 ek,ek1,ek2;   //前后三次误差
    int32 motor_duty;   //电机占空比 motor_duty = (int32)out;
    // 位置环PID参数
    float position_kp;
    float position_ki;
    float position_kd;
    float position_error;
    float position_error_prev;
    float position_error_sum;
    float position_output;
    // 系统状态
    int32 current_speed; //真实速度 int16 speed; 
    float current_position;
    int32 target_speed; //期望速度 int16 set_speed;    
    float target_position;
} MotorTypeDef;
//@brief  里程计数据结构
typedef struct {
    // 编码器参数
    float encoder_resolution;  // 编码器分辨率(PPR)
    float gear_ratio;          // 减速比
    float wheel_radius;        // 轮半径(米)
    // 速度数据(原始与滤波后)
    float raw_speed;           // 原始速度数据(转/秒)
    float filtered_speed;      // 滤波后速度(转/秒)
    float filtered_velocity;   // 线速度(米/秒)
    
    // 里程数据
    float distance;            // 累计里程(米)
    
    // 滑动平均滤波参数
    float speed_buffer[10];    // 速度缓存数组
    int buffer_index;          // 缓存索引
    float buffer_sum;          // 缓存总和
    
    // 卡尔曼滤波参数
//    float q;                   // 过程噪声协方差
//    float r;                   // 测量噪声协方差
//    float x;                   // 状态估计
//    float p;                   // 估计误差协方差
//    float k;                   // 卡尔曼增益
} OdometerTypeDef;
void Motor_Init(); //电机初始化
void Motor_Set(int32 motor1,int32 motor2,int32 motor3);
int32 Motor_Speed(MotorTypeDef* motor); //速度环
float Motor_Position(MotorTypeDef* motor); //位置环
void Motor_Speedcontrol(MotorTypeDef* motor1,MotorTypeDef* motor2,MotorTypeDef* motor3);
void speed_pid_set(float speed_kp,float speed_ki,float speed_kd);
void speed_set(int32 target_speed1,int32 target_speed2,int32 target_speed3);
void Motor_Positioncontrol();
float MovingAverageFilter_g(float new_speed); //滑动平均滤波器
float MovingAverageFilter_p(float new_speed);
float MovingAverageFilter_z(float new_speed);
void g_Odometer_Init(void);
void p_Odometer_Init(void);
void z_Odometer_Init(void);
void Odometer_Init();
#endif /* __MOTOR_H */