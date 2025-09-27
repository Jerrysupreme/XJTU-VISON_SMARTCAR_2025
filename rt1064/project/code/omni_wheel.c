#include"omni_wheel.h"
#include"motor.h"

float speed_M2R(int32 motor)
{
	float real = motor*100*WHEEL_PERIMETER/ENCODER_RESOLUTION;
	return real;
}
int32 speed_R2M(float real)
{
	int32 motor = real*ENCODER_RESOLUTION/(100*WHEEL_PERIMETER);
	return motor;
}
void InverseKinematics(float vx, float vy, float omega, float* wheel_speeds) {
    // 角度转弧度
	float theta1 = WHEEL1_ANGLE_DEG * M_PI / 180.0f;
    float theta2 = WHEEL2_ANGLE_DEG * M_PI / 180.0f;
    float theta3 = WHEEL3_ANGLE_DEG * M_PI / 180.0f;
    // 计算各轮速度（全向轮模型公式）
    wheel_speeds[0] = -vx * sinf(theta1) + vy * cosf(theta1) + WHEEL_RADIUS * omega;
    wheel_speeds[1] = -vx * sinf(theta2) + vy * cosf(theta2) + WHEEL_RADIUS * omega;
    wheel_speeds[2] = -vx * sinf(theta3) + vy * cosf(theta3) + WHEEL_RADIUS * omega;
		//此处公式错误，线性分量应乘2/3，否则速度偏大
}

void Fast_ForwardKinematics(float* wheel_speeds, float* result) {//快速版
    // 静态逆矩阵预计算
    static float inv_A[3][3];
    static int is_calculated = 0;
    
    if (!is_calculated) {
        // 手动构造角度数组（兼容旧编译器）
        const float theta[3] = {
            WHEEL1_ANGLE_DEG,
            WHEEL2_ANGLE_DEG,
            WHEEL3_ANGLE_DEG
        };
        
        float A[3][3];
        for (int i = 0; i < 3; ++i) {
            float rad = theta[i] * (float)M_PI / 180.0f;
		    A[i][0] = -sinf(rad);//若上面修改线性分量乘2/3，则此处也应乘2/3
            A[i][1] = cosf(rad);//若上面修改线性分量乘2/3，则此处也应乘2/3
            A[i][2] = WHEEL_RADIUS;
        }

        // 行列式计算（与原逻辑一致）
        float det_A = A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1]) 
                    - A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0]) 
                    + A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);

        if (fabsf(det_A) < 1e-7f) {
            memset(inv_A, 0, sizeof(inv_A));
            is_calculated = 1;
            return;
        }

        // 逆矩阵计算（保持原逻辑）
        inv_A[0][0] =  (A[1][1]*A[2][2] - A[1][2]*A[2][1]) / det_A;
        inv_A[0][1] = -(A[0][1]*A[2][2] - A[0][2]*A[2][1]) / det_A;
        inv_A[0][2] =  (A[0][1]*A[1][2] - A[0][2]*A[1][1]) / det_A;
        
        inv_A[1][0] = -(A[1][0]*A[2][2] - A[1][2]*A[2][0]) / det_A;
        inv_A[1][1] =  (A[0][0]*A[2][2] - A[0][2]*A[2][0]) / det_A;
        inv_A[1][2] = -(A[0][0]*A[1][2] - A[0][2]*A[1][0]) / det_A;
        
        inv_A[2][0] =  (A[1][0]*A[2][1] - A[1][1]*A[2][0]) / det_A;
        inv_A[2][1] = -(A[0][0]*A[2][1] - A[0][1]*A[2][0]) / det_A;
        inv_A[2][2] =  (A[0][0]*A[1][1] - A[0][1]*A[1][0]) / det_A;

        is_calculated = 1;
    }

    // 矩阵乘法保持不变
    result[0] = inv_A[0][0]*wheel_speeds[0] + inv_A[0][1]*wheel_speeds[1] + inv_A[0][2]*wheel_speeds[2];
    result[1] = inv_A[1][0]*wheel_speeds[0] + inv_A[1][1]*wheel_speeds[1] + inv_A[1][2]*wheel_speeds[2];
    result[2] = inv_A[2][0]*wheel_speeds[0] + inv_A[2][1]*wheel_speeds[1] + inv_A[2][2]*wheel_speeds[2];
}
void ForwardKinematics(float* wheel_speeds, float* result) {
    // 角度转弧度
    float theta1 = WHEEL1_ANGLE_DEG * (float)(M_PI) / 180.0f;
    float theta2 = WHEEL2_ANGLE_DEG * (float)(M_PI) / 180.0f;
    float theta3 = WHEEL3_ANGLE_DEG * (float)(M_PI) / 180.0f;
    float R = WHEEL_RADIUS;
    // 系数矩阵元素
    float a11 = -sinf(theta1);
    float a12 = cosf(theta1);
    float a13 = R;
    float a21 = -sinf(theta2);
    float a22 = cosf(theta2);
    float a23 = R;
    float a31 = -sinf(theta3);
    float a32 = cosf(theta3);
    float a33 = R;
    float s0 = wheel_speeds[0];
    float s1 = wheel_speeds[1];
    float s2 = wheel_speeds[2];

    // 计算行列式
    float det_A = a11*(a22*a33 - a23*a32) 
                - a12*(a21*a33 - a23*a31) 
                + a13*(a21*a32 - a22*a31);

    if (fabsf(det_A) < 1e-7f) {
        result[0] = result[1] = result[2] = 0.0f;
        return;
    }

    // 计算各变量的行列式
    float det_vx = s0*(a22*a33 - a23*a32) 
                 - a12*(s1*a33 - a23*s2) 
                 + a13*(s1*a32 - a22*s2);

    float det_vy = a11*(s1*a33 - a23*s2) 
                 - s0*(a21*a33 - a23*a31) 
                 + a13*(a21*s2 - s1*a31);

    float det_omega = a11*(a22*s2 - s1*a32) 
                     - a12*(a21*s2 - s1*a31) 
                     + s0*(a21*a32 - a22*a31);

    // 结果存储到数组
    result[0] = det_vx / det_A;      // vx
    result[1] = det_vy / det_A;      // vy
    result[2] = det_omega / det_A;   // omega
}
// 设置运动模式,速度取0~7m/s
void SetMotion(MotionMode mode, float speed, float gyro, float yaw) {
    float vx = 0, vy = 0, omega = 0;
    float wheel_speeds[3] = {0};
	float yaw_rad = yaw * M_PI / 180.0f;
    switch (mode) {
        case MODE_STOP:
            vx = 0; vy = 0; omega = 0;
            break;
            
        case MODE_FORWARD:  // 前移
            vx = speed; vy = 0; omega = 0;
            break;
            
        case MODE_BACKWARD: // 后移
            vx = -speed; vy = 0; omega = 0;
            break;
            
        case MODE_LEFT:     // 左平移
            vx = 0; vy = speed; omega = 0;
            break;
            
        case MODE_RIGHT:    // 右平移
            vx = 0; vy = -speed; omega = 0;
            break;
            
        case MODE_ROTATE_CW:   // 顺时针旋转
            vx = 0; vy = 0; omega = -speed;
            break;
        case MODE_ROTATE_CCW:  // 逆时针旋转
            vx = 0; vy = 0; omega = speed;
            break;
		case MODE_AIM: //瞄准模式
			vx = speed; vy = gyro; omega = 0;	
			break;
		case MODE_TRACK://循迹模式
			vx = speed * cosf(yaw_rad);  // 全局X分量
            vy = speed * sinf(yaw_rad);  // 全局Y分量
            omega = gyro;               // 角速度用于航向微调
			break;		
    }
    // 计算轮速
    InverseKinematics(vx, vy, omega, wheel_speeds);
	//printf("%f %f %f\n",vx, vy, omega);
	//printf("wheel: %f %f %f\n",wheel_speeds[0], wheel_speeds[1], wheel_speeds[2]);
	//printf("inver: %f %f %f\n",result[0], result[1], result[2]);
	//printf("faste: %f %f %f\n",result2[0], result2[1], result2[2]);
    //转电机速度
		for(int i = 0;i < 3;i++)
				wheel_speeds[i] = speed_R2M(wheel_speeds[i]);
    // 设置电机目标速度（调用你的速度环接口）
				speed_set((int16_t)wheel_speeds[0], 
              (int16_t)wheel_speeds[1], 
              (int16_t)wheel_speeds[2]);
}