#include "motor.h"
int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;
int16 encoder_data_3 = 0;
int16 encoder_data_4 = 0;
MotorTypeDef m1,m2,m3;
OdometerTypeDef g_odometer,p_odometer,z_odometer;
void g_Odometer_Init(void)
{
    // 编码器与机械参数
    g_odometer.encoder_resolution = 1024;
    g_odometer.gear_ratio = 30.0f;
    g_odometer.wheel_radius = 0.05f; 
    // 里程与速度初始化
    g_odometer.distance = 0.0f;
    g_odometer.raw_speed = 0.0f;
    g_odometer.filtered_speed = 0.0f;   
    // 滑动平均滤波初始化
    for (int i = 0; i < 10; i++) {
        g_odometer.speed_buffer[i] = 0.0f;
    }
    g_odometer.buffer_index = 0;
    g_odometer.buffer_sum = 0.0f;  
//    // 卡尔曼滤波初始化
//    g_odometer.q = 0.01f;      // 过程噪声(可调整)
//    g_odometer.r = 0.1f;       // 测量噪声(可调整)
//    g_odometer.x = 0.0f;
//    g_odometer.p = 1.0f;
}
void p_Odometer_Init(void)
{
	// 编码器与机械参数
    p_odometer.encoder_resolution = 1024;
    p_odometer.gear_ratio = 30.0f;
    p_odometer.wheel_radius = 0.05f; 
    // 里程与速度初始化
    p_odometer.distance = 0.0f;
    p_odometer.raw_speed = 0.0f;
    p_odometer.filtered_speed = 0.0f;   
    // 滑动平均滤波初始化
    for (int i = 0; i < 10; i++) {
        p_odometer.speed_buffer[i] = 0.0f;
    }
    p_odometer.buffer_index = 0;
    p_odometer.buffer_sum = 0.0f;
}
void z_Odometer_Init(void)
{
	// 编码器与机械参数
    z_odometer.encoder_resolution = 1024;
    z_odometer.gear_ratio = 30.0f;
    z_odometer.wheel_radius = 0.05f; 
    // 里程与速度初始化
    z_odometer.distance = 0.0f;
    z_odometer.raw_speed = 0.0f;
    z_odometer.filtered_speed = 0.0f;   
    // 滑动平均滤波初始化
    for (int i = 0; i < 10; i++) {
        z_odometer.speed_buffer[i] = 0.0f;
    }
    z_odometer.buffer_index = 0;
    z_odometer.buffer_sum = 0.0f;
}
void Odometer_Init()
{
	g_Odometer_Init();
	z_Odometer_Init();
	p_Odometer_Init();
}
void Motor_Init()
{
	encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);                     // init encoder
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);                    
    encoder_quad_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B);                    
    encoder_quad_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B);
	
	
	gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            
	pwm_init(MOTOR1_PWM, 17000, 0); 
	gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            
	pwm_init(MOTOR2_PWM, 17000, 0);
	gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            
	pwm_init(MOTOR3_PWM, 17000, 0);	
	gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                     
    pwm_init(MOTOR4_PWM, 17000, 0); 
	
}

void Motor_Set(int32 motor1, int32 motor2, int32 motor3)
{
    // Motor 1 control
    if (motor1 >= 0)
    {
        gpio_set_level(MOTOR1_DIR, GPIO_HIGH);     // Set DIR to high level
        pwm_set_duty(MOTOR1_PWM, motor1);  // Set positive duty cycle
    }
    else
    {
        gpio_set_level(MOTOR1_DIR, GPIO_LOW);      // Set DIR to low level
        pwm_set_duty(MOTOR1_PWM, -motor1);  // Set negative duty cycle
    }
    // Motor 2 control
    if (motor2 >= 0)
    {
        gpio_set_level(MOTOR2_DIR, GPIO_HIGH);     // Set DIR to high level
        pwm_set_duty(MOTOR2_PWM, motor2);  // Set positive duty cycle
    }
    else
    {
        gpio_set_level(MOTOR2_DIR, GPIO_LOW);      // Set DIR to low level
        pwm_set_duty(MOTOR2_PWM,-motor2);  // Set negative duty cycle
    }
    // Motor 3 control
    if (motor3 >= 0)
    {
        gpio_set_level(MOTOR3_DIR, GPIO_HIGH);     // Set DIR to high level
        pwm_set_duty(MOTOR3_PWM, motor3);  // Set positive duty cycle
    }
    else
    {
        gpio_set_level(MOTOR3_DIR, GPIO_LOW);      // Set DIR to low level
        pwm_set_duty(MOTOR3_PWM, -motor3);  // Set negative duty cycle
    }
}

float limit_PWM(float input)
{
	return input > PWM_DUTY_MAX?PWM_DUTY_MAX:(input < -PWM_DUTY_MAX?-PWM_DUTY_MAX:input); 
}

float limit_I(float input)
{
	return input > I_MAX?I_MAX:(input < -I_MAX?-I_MAX:input); 
}
int32 Motor_Speed(MotorTypeDef* motor) //速度环
{
    motor->ek2 =  motor->ek1;//保存上上次误差
    motor->ek1 =  motor->ek; //保存上次误差
    motor->ek =  motor->target_speed - motor->current_speed;//计算当前误差
    //进行增量式PID运算
    motor->out_increment = (int32)(motor->speed_kp*(motor->ek-motor->ek1) + limit_I(motor->speed_ki*motor->ek) + motor->speed_kd*(motor->ek-2*motor->ek1+motor->ek2));  //计算增量
    motor->speed_output += motor->out_increment;       //输出增量
	motor->speed_output = limit_PWM(motor->speed_output);      //输出限幅 不能超过占空比最大值
	motor->motor_duty = (int32)motor->speed_output;    //强制转换为整数后赋值给电机占空比变量
	return motor->motor_duty;
}

void speed_pid_set(float speed_kp,float speed_ki,float speed_kd)
{
	m1.speed_kp = speed_kp;
	m1.speed_ki = speed_ki;
	m1.speed_kd = speed_kd;
	m2.speed_kp = speed_kp;
	m2.speed_ki = speed_ki;
	m2.speed_kd = speed_kd;
	m3.speed_kp = speed_kp;
	m3.speed_ki = speed_ki;
	m3.speed_kd = speed_kd;
}

void speed_set(int32 target_speed1,int32 target_speed2,int32 target_speed3)
{
	m1.target_speed = limit_PWM(target_speed1);
	m2.target_speed = limit_PWM(target_speed2);
	m3.target_speed = limit_PWM(target_speed3);
}

void Motor_Speedcontrol(MotorTypeDef* motor1,MotorTypeDef* motor2,MotorTypeDef* motor3)
{
	motor1->current_speed = encoder_data_1;
	motor2->current_speed = encoder_data_2;
	motor3->current_speed = encoder_data_3;
	int16 s1 = Motor_Speed(motor1);
	int16 s2 = Motor_Speed(motor2);;
	int16 s3 = Motor_Speed(motor3);
	Motor_Set(s1,s2,s3);	
}


float Motor_Position(MotorTypeDef* motor)//位置环
{
	return 0;
}
void Motor_Positioncontrol()
{
	
}

/**
  * @brief  滑动平均滤波器
  * @param  new_speed: 新的速度采样值
  * @return 滤波后的速度值
  */
float MovingAverageFilter_g(float new_speed)
{
    // 移除最早的值
    g_odometer.buffer_sum -= g_odometer.speed_buffer[g_odometer.buffer_index];
    
    // 添加新值
    g_odometer.speed_buffer[g_odometer.buffer_index] = new_speed;
    g_odometer.buffer_sum += new_speed;
    
    // 更新索引
    g_odometer.buffer_index = (g_odometer.buffer_index + 1) % 10;
    
    // 返回平均值
    return g_odometer.buffer_sum / 10.0f;
}
float MovingAverageFilter_p(float new_speed)
{
    // 移除最早的值
    p_odometer.buffer_sum -= p_odometer.speed_buffer[p_odometer.buffer_index];
    
    // 添加新值
    p_odometer.speed_buffer[p_odometer.buffer_index] = new_speed;
    p_odometer.buffer_sum += new_speed;
    
    // 更新索引
    p_odometer.buffer_index = (p_odometer.buffer_index + 1) % 10;
    
    // 返回平均值
    return p_odometer.buffer_sum / 10.0f;
}
float MovingAverageFilter_z(float new_speed)
{
    // 移除最早的值
    z_odometer.buffer_sum -= z_odometer.speed_buffer[z_odometer.buffer_index];
    
    // 添加新值
    z_odometer.speed_buffer[z_odometer.buffer_index] = new_speed;
    z_odometer.buffer_sum += new_speed;
    
    // 更新索引
    z_odometer.buffer_index = (z_odometer.buffer_index + 1) % 10;
    
    // 返回平均值
    return z_odometer.buffer_sum / 10.0f;
}