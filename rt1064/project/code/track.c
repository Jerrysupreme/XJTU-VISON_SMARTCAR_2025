#include "track.h"
#include "data.h"
#include "path_process.h"
#include "omni_wheel.h"
#include "motor.h"
#include "uart.h"
#include "image.h"
extern OdometerTypeDef g_odometer,p_odometer,z_odometer;
extern int16 path_err;
extern int track_error;  
extern uint8 roundabout_entry_flag;
extern int update_data;
extern float angle_adj;
float gyro_z = 0;
int pushflag = 0;
TRACK_PID track;
G_PID move_z,aim_z;
P_PID move_p;
AIM_PID move_x;
AIM_PID move_y;
int box_x,box_y,box_area = 0;
extern int time_flag;
extern od_result_t od_result[10];
float limit_out(float input,float limit)
{
	if(abs(input) > limit) { 
        input = (input > 0) ? limit : -limit;
    }//limit integral
	return input;
}
float track_pid_control(TRACK_PID *track,int16 path_err)
{
	static float integral = 0;
	float gyro_now_err,value; 
	gyro_now_err = GYRO_Z_FORWARD*(imu660ra_gyro_z-0.786);
	integral += -path_err;
	if(abs(integral) > 30) { 
        integral = (integral > 0) ? 30 : -30;
    }//limit integral
	value = track->Kp*(-path_err)+ track->Ki*integral + track->Kd*(-path_err-track->prev_error1)-track->Gd*(gyro_now_err-track->prev_error2);
	track->prev_error1 = -path_err;
	track->prev_error2 = gyro_now_err;
	return value;
}
float aim_z_control(G_PID *aim_z,float x)//z瞄准位置环
{
	static float integral = 0;
	float value; 
	float err = AIM_X - x;
	integral += err;
	if(abs(integral) > 30) { 
        integral = (integral > 0) ? 30 : -30;
    }//limit integral
	value = aim_z->Kp*(err)+ aim_z->Ki*integral + aim_z->Kd*(err-aim_z->prev_error);
	aim_z->prev_error = err;
	return value;	
}
float G_pid_control(G_PID *move_z,float g_od)//Z轴位置环
{
	static float integral = 0;
	float value; 
	float err = move_z->target - g_od;
	integral += err;
	if(abs(integral) > 30) { 
        integral = (integral > 0) ? 30 : -30;
    }//limit integral
	value = move_z->Kp*(err)+ move_z->Ki*integral + move_z->Kd*(err-move_z->prev_error);
	move_z->prev_error = err;
	return value;
}
float P_pid_control(P_PID *move_p,float p_od)//移动位置环
{
	static float integral = 0;
	float value; 
	float err = move_p->target - p_od;
	integral += err;
	if(abs(integral) > 30) { 
        integral = (integral > 0) ? 30 : -30;
    }//limit integral
	value = move_p->Kp*(err)+ move_p->Ki*integral + move_p->Kd*(err-move_p->prev_error);
	move_p->prev_error = err;
	return value;
}
float aim_x_control(AIM_PID *move_x,float x)//x瞄准位置环
{
	static float integral = 0;
	float value; 
	float err = AIM_X - x;
	integral += err;
	if(abs(integral) > 30) { 
        integral = (integral > 0) ? 30 : -30;
    }//limit integral
	value = move_x->Kp*(err)+ move_x->Ki*integral + move_x->Kd*(err-move_x->prev_error);
	move_x->prev_error = err;
	return value;	
}
float aim_y_control(AIM_PID *move_y,float y,float target_y)//y瞄准位置环
{
	static float integral = 0;
	float value; 
	float err = target_y - y;
	integral += err;
	if(abs(integral) > 30) { 
        integral = (integral > 0) ? 30 : -30;
    }//limit integral
	value = move_y->Kp*(err)+ move_y->Ki*integral + move_y->Kd*(err-move_y->prev_error);
	move_y->prev_error = err;
	return value;	
}
//使用前初始化里程计
int G_control(G_PID *move_z,float g_od,float target)//Z轴位置环串级pid控制
{
	move_z->target = target;
	float err = move_z->target - g_od;
	if(abs(err)<5)return 1;
	float G_out = G_pid_control(move_z,g_od);
	G_out = limit_out(G_out,3);
	SetMotion(MODE_TRACK,0,G_out,0);
	return 0;
}


int G_control_straight(G_PID *move_z,float g_od,float target)//Z轴位置环串级pid控制
{
	move_z->target = target;
	float err = move_z->target - g_od;
	if(abs(err)<5)return 1;
	float G_out = G_pid_control(move_z,g_od);
	G_out = limit_out(G_out,3);
	SetMotion(MODE_TRACK,0,G_out,0);
	return 0;
}


int G_control_absolute(G_PID *move_z,float g_od,float target)//z轴绝对角度控制
{
	move_z->target = target;
	float err = move_z->target - g_od;
	if(abs(err)<5)return 1;
	float G_out = G_pid_control(move_z,g_od);
	G_out = limit_out(G_out,5);
	SetMotion(MODE_TRACK,0,G_out,0);
	return 0;
}
int G_move_absolute(G_PID *move_z,float g_od,float target,float p_od,float p_target,float speed)//z轴绝对运动控制
{
	move_z->target = target;
	float err2 = p_target - p_od;
	if(abs(err2)<0)return 1;
	float err = move_z->target - g_od;
	if(abs(err)<5){SetMotion(MODE_TRACK,speed,0,target);}
	else{SetMotion(MODE_TRACK,speed,G_pid_control(move_z,g_od),target);}
	return 0;
}
int P_control(P_PID *move_p,float p_od,float yaw,float target)//移动位置环串级pid控制
{
	move_p->target = target;
	float err = move_p->target - p_od;
	if(abs(err)<5)return 1;
	if(err<0)return 1;
	SetMotion(MODE_TRACK,P_pid_control(move_p,p_od),0,yaw);
	return 0;
}
int P_control_simple(float speed,float p_od,float yaw,float target)
{
	float err = target - p_od;
	if(abs(err)<5){SetMotion(MODE_STOP,0,0,0);return 1;}
	if(err<0){SetMotion(MODE_STOP,0,0,0);return 1;}
	SetMotion(MODE_TRACK,speed,0,yaw);
	return 0;
}
int z_rotate(float speed,float g_od,float target,int dir)//dir为1逆转，否则顺转
{
	float r = 0.35;
	float err = target - dir*g_od;
	if(err<0){SetMotion(MODE_STOP,0,0,0);return 1;}
	SetMotion(MODE_TRACK,dir*speed,dir*speed/r,-90);
	return 0;
}


void calculate_box()//计算箱子坐标
{
	if(update_data){
	static int y_pre = 0,x_pre = 0,box_area_pre = 0;
	int max = 0,valid = 1;
	int y_max = (od_result[0].res_y1+od_result[0].res_y2)/2;
	int x_max = (od_result[max].res_x1 + od_result[max].res_x2)/2;
	if(y_max || x_max){
	if(valid){
	for(int i = 1; i < 5; i++)
	{
		if(od_result[i].res_y1 && od_result[i].res_y2 && od_result[i].res_x1 && od_result[i].res_x2)
		{
			int y_now = (od_result[i].res_y1+od_result[i].res_y2)/2;
			if(y_now>y_max && y_now<CAMERA_H )
			{
				y_max = y_now;
				max = i;
			}	
		}
		
	}
	int x = (od_result[max].res_x1 + od_result[max].res_x2)/2;
	if(y_max<CAMERA_H && x<CAMERA_W &&(od_result[max].res_y1 && od_result[max].res_y2 && od_result[max].res_x1 && od_result[max].res_x2))
	{
		y_pre = box_y;
		x_pre = box_x;
		box_area_pre = box_area;
		box_x = x;
		box_y = y_max;
		box_area = abs(od_result[max].res_x1-od_result[max].res_x2)*abs(od_result[max].res_y1-od_result[max].res_y2);
	}
	else
	{
		box_x = x_pre;
		box_y = y_pre;
		box_area = box_area_pre;
	}
	
	}}
	else{
		box_x = 0;
		box_y = 0;
		box_area = 0;
	}}
	//printf("box_x box_y: %d %d\n",(od_result[max].res_x1 + od_result[max].res_x2)/2, y_max);
}
int check_box()
{
	calculate_box();
//	&& box_x >100 && box_x<220s
	if(box_y>FIND_BOX && box_y<CAMERA_H && box_area>2000&&box_x<270&&box_x>50)return 1;//4000 area 80-214  47 98-221  wai 41-282 72-   297
	return 0;
}

int aim_control()
{
	calculate_box();
	float target_y = AIM_Y;
	float err_x = AIM_X - box_x; 
	if(roundabout_entry_flag == 5 || roundabout_entry_flag == 6){target_y = AIM_Y_ROUND;}
	float err_y = target_y - box_y;
	
	//if(abs(err_x)<5 && abs(err_y)<5)return 1;
	if(abs(err_y)<10 && abs(err_x)<10)return 1;
	//SetMotion(MODE_AIM,aim_y_control(&move_y,box_y),aim_x_control(&move_x,box_x),0);
	float out_x = aim_x_control(&move_x,box_x);
	float out_y = aim_y_control(&move_y,box_y,target_y);
	//printf("err out_x box_x AIM_X:%f %f %d %d\n",move_x.prev_error,out_x,box_x,AIM_X);
	out_x = limit_out(out_x,2);
	out_y = limit_out(out_y,2);
	SetMotion(MODE_AIM,out_y,out_x,0);
	return 0;
}


int aim_control_2()
{
	calculate_box();
	if(box_y<FIND_BOX){SetMotion(MODE_TRACK,0,2,0);}
	else{
	float target_y = AIM_Y;
	float err_x = AIM_X - box_x; 
	//if(roundabout_entry_flag == 5 || roundabout_entry_flag == 6){target_y = AIM_Y_ROUND;}
	float err_y = target_y - box_y;
	if(!box_y || !box_x || (abs(err_x)<8 && abs(err_y)<8))return 1;//8  8
	//SetMotion(MODE_AIM,aim_y_control(&move_y,box_y),aim_x_control(&move_x,box_x),0);
	float out_x = aim_x_control(&move_x,box_x);
	float out_y = aim_y_control(&move_y,box_y,target_y);
	//printf("err out_x box_x AIM_X:%f %f %d %d\n",move_x.prev_error,out_x,box_x,AIM_X);
	out_x = limit_out(out_x,2);
	out_y = limit_out(out_y,2);
	if(abs(err_y)<5)SetMotion(MODE_TRACK,0,out_x,0);
	else if(abs(err_x)<5)SetMotion(MODE_TRACK,out_y,0,0);
	else SetMotion(MODE_TRACK,out_y,out_x,0);
	}
	return 0;
}

int z_control() //单瞄准
{
	static int count = 0;
	calculate_box();
	float err_x = AIM_X - box_x; 
	if(!box_y || !box_x)count++;
	if(abs(err_x)<5||count>50){count = 0;return 1;}
	float out_x = aim_z_control(&aim_z,box_x);
	out_x = limit_out(out_x,2);
	SetMotion(MODE_TRACK,0,out_x,0);
	return 0;
}

int z_move(float speed,float od,float target)  //瞄准加前进
{
	static int count = 0;
	calculate_box();
	float err_x = AIM_X - box_x; 
	float err_d = target - od;
	if(err_d<0){count = 0;return 1;}
	if(od>25)SetMotion(MODE_TRACK,speed,0,0);
	else if(abs(err_x)>5){
		float out_x = aim_z_control(&aim_z,box_x);
		out_x = limit_out(out_x,2);
		if(!box_y || !box_x)out_x = 0;
		SetMotion(MODE_TRACK,speed,out_x,0);
	}
	else{SetMotion(MODE_TRACK,speed,0,0);}
	return 0;
}

int push2r_control()//推向右侧
{
	switch(pushflag){
		case 0:
//			if(P_control_simple(0.5,p_odometer.distance,90,25))
//			{
//				SetMotion(MODE_STOP,0,0,0);
//				p_Odometer_Init();
//				g_Odometer_Init();
//				pushflag = 1;
//			}
			if(z_rotate(0.6,g_odometer.distance,200,-1))//dir为1逆转，否则顺转
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 3;
			}
			break;			
			break;
		case 1:
			if(P_control_simple(0.5,p_odometer.distance,0,40))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 2;
			}
			break;
		case 2:
			if(G_control(&move_z,g_odometer.distance,-145))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 3;
			}
			break;
		case 3:
			if(z_move(0.6,p_odometer.distance,70))//42
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 5;        
			}
			break;
		case 4:
			if(P_control_simple(0.5,p_odometer.distance,0,78))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 5;
				//return 1;
			}
			break;
		case 5:
			if(P_control_simple(0.4,p_odometer.distance,180,15))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 6;
				//return 1;
			}
			break;
		case 6:
			if(G_control_absolute(&move_z,z_odometer.distance,angle_adj))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 0;
				return 1;
			}
			break;
		default:break;	
	}
	return 0;
}
int push2l_control()//推向左侧
{
	switch(pushflag){
		case 0:
//			if(P_control_simple(0.5,p_odometer.distance,-90,25))
//			{
//				SetMotion(MODE_STOP,0,0,0);
//				p_Odometer_Init();
//				g_Odometer_Init();
//				pushflag = 1;
//			}
			if(z_rotate(0.6,g_odometer.distance,200,1))//dir为1逆转，否则顺转
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 3;
			}
			break;
		case 1:
			if(P_control_simple(0.5,p_odometer.distance,0,40))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 2;
			}
			break;
		case 2:
			if(G_control(&move_z,g_odometer.distance,145))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 3;
			}
			break;
		case 3:
			if(z_move(0.6,p_odometer.distance,70))//0.4 80
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 5;        
			}
			break;
		case 4:
			if(P_control_simple(0.8,p_odometer.distance,0,60))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 5;
				//return 1;
			}
			break;
		case 5:
			if(P_control_simple(0.8,p_odometer.distance,180,15))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 6;
				//return 1;
			}
			break;
		case 6:
			if(G_control_absolute(&move_z,z_odometer.distance,angle_adj))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 0;
				return 1;
			}
			break;
		default:break;	
	}
	return 0;
}

int push2r_control_dun()//推向右侧 钝角
{
	switch(pushflag){
		case 0:
//			if(P_control_simple(0.5,p_odometer.distance,90,25))
//			{
//				SetMotion(MODE_STOP,0,0,0);
//				p_Odometer_Init();
//				g_Odometer_Init();
//				pushflag = 1;
//			}
			if(z_rotate(0.6,g_odometer.distance,150,-1))//dir为1逆转，否则顺转
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 3;
			}
			break;			
			break;
		case 1:
			if(P_control_simple(0.5,p_odometer.distance,0,40))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 2;
			}
			break;
		case 2:
			if(G_control(&move_z,g_odometer.distance,-145))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 3;
			}
			break;
		case 3:
			if(z_move(0.6,p_odometer.distance,70))//42
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 5;        
			}
			break;
		case 4:
			if(P_control_simple(0.5,p_odometer.distance,0,78))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 5;
				//return 1;
			}
			break;
		case 5:
			if(P_control_simple(0.4,p_odometer.distance,180,20))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 6;
				//return 1;
			}
			break;
		case 6:
			if(G_control_absolute(&move_z,z_odometer.distance,angle_adj))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 0;
				return 1;
			}
			break;
		default:break;	
	}
	return 0;
}
int push2l_control_dun()//推向左侧 钝角
{
	switch(pushflag){
		case 0:
//			if(P_control_simple(0.5,p_odometer.distance,-90,25))
//			{
//				SetMotion(MODE_STOP,0,0,0);
//				p_Odometer_Init();
//				g_Odometer_Init();
//				pushflag = 1;
//			}
			if(z_rotate(0.6,g_odometer.distance,150,1))//dir为1逆转，否则顺转
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 3;
			}
			break;
		case 1:
			if(P_control_simple(0.5,p_odometer.distance,0,40))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 2;
			}
			break;
		case 2:
			if(G_control(&move_z,g_odometer.distance,145))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 3;
			}
			break;
		case 3:
			if(z_move(0.6,p_odometer.distance,70))//0.4 80
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 5;        
			}
			break;
		case 4:
			if(P_control_simple(0.8,p_odometer.distance,0,60))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 5;
				//return 1;
			}
			break;
		case 5:
			if(P_control_simple(0.8,p_odometer.distance,180,20))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 6;
				//return 1;
			}
			break;
		case 6:
			if(G_control_absolute(&move_z,z_odometer.distance,angle_adj))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				pushflag = 0;
				return 1;
			}
			break;
		default:break;	
	}
	return 0;
}

void G_pid_control_set(float Kp,float Ki,float Kd)
{
	move_z.Kp = Kp;
	move_z.Ki = Ki;
	move_z.Kd = Kd;
}
void P_pid_control_set(float Kp,float Ki,float Kd)
{
	move_p.Kp = Kp;
	move_p.Ki = Ki;
	move_p.Kd = Kd;
}
void aim_x_pid_set(float Kp,float Ki,float Kd)
{
	move_x.Kp = Kp;
	move_x.Ki = Ki;
	move_x.Kd = Kd;
}
void aim_y_pid_set(float Kp,float Ki,float Kd)
{
	move_y.Kp = Kp;
	move_y.Ki = Ki;
	move_y.Kd = Kd;
}
void aim_z_pid_set(float Kp,float Ki,float Kd)
{
	aim_z.Kp = Kp;
	aim_z.Ki = Ki;
	aim_z.Kd = Kd;
}
void track_control(float speed)
{
	//get_err();
	if(time_flag){
	float value = track_pid_control(&track,track_error);
	value = limit_out(value,8);
	if(abs(track_error) > 10)//10 13
	{
		SetMotion(MODE_TRACK,speed,value,0);
	}
	else
	{
		SetMotion(MODE_TRACK,speed,0,-1.2*track_error);//1.2
	}
	time_flag = 0;
	}
}
int track_control_2() //纠正姿态
{
	if(time_flag){
	float value = track_pid_control(&track,track_error);
	value = limit_out(value,8);
	if(abs(track_error) > 20)//10 13
	{
		SetMotion(MODE_LEFT, limit_out(-0.02*track_error,0.8),0,0);//需要测试参数
	}
	else if(abs(track_error) > 10)
	{
		SetMotion(MODE_TRACK,0,value,0);
	}
	else{return 1;}
	time_flag = 0;
	}
	return 0;
}
void track_pid_set(float Kp,float Ki,float Kd,float Gd)
{
	track.Kp = Kp;
	track.Ki = Ki; 
	track.Kd = Kd;
	track.Kd = Kd;	
}

int out_detect(int track_error) //出界检测
{
    static int error_count = 0;  // 静态变量，用于记录连续超限的次数
    if (abs(track_error) > 40) {
        error_count++;  // 误差绝对值大于40，计数器加1
    } else {
        error_count = 0;  // 误差在允许范围内，计数器重置
    }
    
    if (error_count >= 10) {
        error_count = 0;  // 重置计数器以便下次检测
        return 1;  // 返回出界标志
    }
    
    return 0;  // 未出界
}
extern int moveflag;
void out_process() //出界处理
{
	if(out_detect(track_error))
	{
		moveflag = 6;
	}
}
