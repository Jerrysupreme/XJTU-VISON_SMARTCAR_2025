/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ????RT1064DVL6A ???????h???????? SDK ???j??????????
* Copyright (c) 2022 SEEKFREE ?????
* 
* ???l??? RT1064DVL6A ??????h????
* 
* RT1064DVL6A ????? ?????????
* ?????????????????????????? GPL??GNU General Public License???? GNU??ù????????????????
* ?? GPL ?j?3???? GPL3.0?????????g????????I??????·?????/???????
* 
* ???????k??????????????????ã?????d????????ei??
* ????û????????????????"??????;?i??
* ?????????µ? GPL
* 
* ???????????????????????h?? GPL ?????
* ???û????????<https://www.gnu.org/licenses/>
* 
* ?????????
* ???????'?? GPL3.0 ??????????? ????????????????I?
* ??????????I??? libraries/doc ?l????µ? GPL3_permission_statement.txt ?l???
* ??????????? libraries ?l????? ?????l????µ? LICENSE ?l?
* ??????'?ò??????????? ??????????????????????I??????????????????
* 
* ?l?????          main
* ???????          ??????????????
* ?????          ?? libraries/doc ?l????? version ?l? ?????
* ????????          IAR 8.32.4 or MDK 5.33
* ??????          RT1064DVL6A
* ????????          https://seekfree.taobao.com/
* 
* ??l?¼
* ????              ????                ???
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "motor.h"
#include "omni_wheel.h"
#include "data.h"
#include "path_process.h"
#include "picture_process.h"
#include "wifi.h"
#include "track.h"
#include "uart.h"
#include "image.h"
#include "btn.h"
// ???µL??????????z???????????????²???
// ??h?? ???????????????l?
// ????? project->clean  ????·???????????

// ????????????????ÿ????
extern int16 encoder_data_1,encoder_data_2, encoder_data_3, encoder_data_4;
extern MotorTypeDef m1,m2,m3;
extern OdometerTypeDef g_odometer,p_odometer,z_odometer;
extern uint8 image_copy[MT9V03X_H][MT9V03X_W];
extern TRACK_PID track;
extern int moveflag;
extern G_PID move_z;
extern P_PID move_p;
extern int box_x,box_y,box_area;
extern uint8 roundabout_entry_flag;
extern uint8 crossroad_flag;
float control_speed = 0.8;
float angle_adj = 0;
int time_flag = 0;
int cal_flag = 0;
int lock_box = 0;
extern uint8 index;
uint8_t current_result = 0;
uint8_t results[20] = {0};
uint8_t Is_right = 1;
uint8_t pre_index = 200;
int count_box = 0;
// ?????k????????Flash?????????????
const char labels[][16] = {  // ??????k"electrodrill"(12???) + ???????????16???
    "keyboard", "mobile_phone", "mouse", "headphones", 
    "monitor", "speaker", "number.random20", "wrench",
    "soldering_iron", "electrodrill", "tape_measure", "screwdriver",
	"pliers", "oscillograph", "multimeter", "printer"
};
// ?????k???????Flash?????????????
const uint8_t label_class[16] = { 
    0,0,0,0,
	0,0,0,1,
	1,1,1,1,
	1,1,1,0
};

// ?????±?????k????
const char* get_label(uint8_t index) {
    if (index < sizeof(labels) / sizeof(labels[0])) {
        return labels[index];  // ????????????
    }
    return "Invalid Index";  // ??????
}
// ?????±?????k????
const uint8_t get_number(uint8_t index) {
    return index - 16;
}


void pid_init()
{
	g_Odometer_Init();
	p_Odometer_Init();
	speed_pid_set(5.9,0.7,0);//speed_pid_set(0.55,0.07,0.01);
	//track_pid_set(0.25,0,0.5,0);
	track_pid_set(0.2,0,1,0);//0.2,0,0.5,0
	G_pid_control_set(0.05,0.01,0.01);//0.05 0.01 0.01 encoder
	P_pid_control_set(0.02,0,0);
	aim_x_pid_set(0.013,0,0.03);//0.003,0,0 tradition 0.015
	aim_y_pid_set(0.010,0,0.02);//0.012
	aim_z_pid_set(0.024,0,0);//0.015
	//10000 pwm_max,2800 encoder max,7485 one circle 10ms period 
	//Motor_Set(0,0.1*PWM_DUTY_MAX,0); speed_set(1000,1000,1000);
	//SetMotion(MODE_FORWARD,0.5,0);
}
void encoder_update()
{
	encoder_data_1 = encoder_get_count(ENCODER_1);                              // ?????????????
    encoder_clear_count(ENCODER_1);                                             // ????????????
    encoder_data_2 = encoder_get_count(ENCODER_2);                              // ?????????????
    encoder_clear_count(ENCODER_2);                                             // ????????????
    encoder_data_3 = encoder_get_count(ENCODER_3);                              // ?????????????
    encoder_clear_count(ENCODER_3);                                             // ????????????
    encoder_data_4 = encoder_get_count(ENCODER_4);                              // ?????????????
	encoder_clear_count(ENCODER_4);                                             // ????????????
}
void odometer_update()
{
	float result[3] = {0},wheel_speeds[3] = {speed_M2R(encoder_data_1),speed_M2R(encoder_data_2),speed_M2R(encoder_data_3)};
	Fast_ForwardKinematics(wheel_speeds,result);
	float linear_speed = sqrtf(result[0] * result[0] + result[1] * result[1]);
	float z_speed = result[2];
	p_odometer.raw_speed = linear_speed;
	p_odometer.filtered_speed = MovingAverageFilter_p(p_odometer.raw_speed);
	p_odometer.distance += p_odometer.filtered_speed;
	g_odometer.raw_speed = z_speed;
	g_odometer.filtered_speed = MovingAverageFilter_g(g_odometer.raw_speed);
	g_odometer.distance += g_odometer.filtered_speed;
}
int push_flag_update()
{
	//push_flag 1 ->right
	if(current_result>15)return (current_result%2==0)?1:0;
	else{
		return label_class[current_result];
	}
}
void PI_test()
{
	speed_set(500,500,500);
	system_delay_ms(3000);
	speed_set(-500,-500,-500);
	system_delay_ms(5000);
}
int art_identify();

void show_results_new()
{
	ips200_show_string(10,3,"num");
	ips200_show_string(120,3,"kind");
	char data[20] = {0};
	for (int i = 0; i < 16;i++)
	{
		ips200_draw_line(5,20 * i,220,20 * i,RGB565_BLUE);
	}
	for (int j = 0; j < 3;j++)
	{
		ips200_draw_line(5 + 110*j,0,5 + 110*j,319,RGB565_BLUE);
	}
	for(int i = 0; i < sizeof(results)/sizeof(results[0]);i++)
	{
		if(results[i] == 200)return;
		uint8_t id = results[i];
		if(id>15)
		{
			sprintf(data,"%d",get_number(id));
		}
		else{
			sprintf(data,"%s",get_label(id));
		}
		ips200_show_uint(10,3 + 20*(i+1), i+1, 3);
		ips200_show_string(120,3 + 20*(i+1),data);
	}
}

void show_results()
{
	ips200_show_string(0,0,"num");
	ips200_show_string(100,0,"kind");
	char data[20] = {0};
	for(int i = 0; i < sizeof(results)/sizeof(results[0]);i++)
	{
		if(results[i] == 200)return;
		uint8_t id = results[i];
		if(id>15)
		{
			sprintf(data,"%d",get_number(id));
		}
		else{
			sprintf(data,"%s",get_label(id));
		}
		ips200_show_uint(0,30*(i+1), i+1, 3);
		ips200_show_string(100,30*(i+1),data);
	}
}


int main(void)

{
    clock_init(SYSTEM_CLOCK_600M);  // ???????
    debug_init();                   // ???????'??
	uart4_init();
	system_delay_ms(300);           //?????????????????????
    // ???????û????? ?????????'???????
    Motor_Init();
	mt9v03x_init();
	imu660ra_init();
	btn_init();
	//wifi_init();
	ips200_init(IPS200_TYPE_SPI);  
	pid_init();
	Odometer_Init();
	pit_ms_init(PIT_CH, 10); // init isr PIT_CH0 ????
	pit_ms_init(PIT_CH1, 40); // 40
	pit_ms_init(PIT_CH2, 25);//?????
	interrupt_global_enable(0); //??????????
	//speed_set(500,500,500);
//	MODE_FORWARD,
//    MODE_BACKWARD,
//    MODE_LEFT,
//    MODE_RIGHT,
	//SetMotion(MODE_LEFT,0.8,0,0);
	//Motor_Set(1000,1000,1000);
	                                                                                                                       
	Motor_Set(0,0,0);
	for(int i = 0; i < sizeof(results)/sizeof(results[0]);i++)
	{results[i] = 200;}//200
//	for(int i = 0; i < 8;i++)
//	{results[i] = 100;}//200
//	results[8] = 200;
	//show_results();
    while(1)
    {
//		ImageProcess();
//		//test1
//		draw_enhanced_borders();       // ??????
//		draw_centerline();       // ???????
//		ips200_displayimage03x((uint8 *)bin_image, image_w, image_h);
//		ips200_show_gray_image(0, 130, mt9v03x_image[0] , MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
////		//test2
//		draw_enhanced_borders();       // ??????
//		draw_centerline();       // ???????
//		ips200_show_gray_image(0, 130, mt9v03x_image[0] , MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
//		display_status_flags(); // ????????
		if(!moveflag)
		key_process();
		//art_identify();
//		ips200_show_string(0,0,"                     ");
//		ips200_show_string(0,0,get_label(index));
//		ips200_show_uint(0,0,0,3);
//		system_delay_ms(50);
		//ips200_clear();
		//ImageProcess();
		//if(roundabout_entry_flag == 1)moveflag = 2;
		
		//printf("IMU660RA z:%5f,%5d\r\n",z_odometer.distance,imu660ra_gyro_z);
		//printf("z_angle: %5f %4d\n", z_odometer.distance,roundabout_entry_flag);
		//SetMotion(MODE_FORWARD,0.5,0,0);
		//printf("%d,%d\n",box_x,box_y);
		//ips200_displayimage03x((uint8 *)mt9v03x_image, image_w, image_h);
        // ???????????????J???	
		
//		 if(mt9v03x_finish_flag)
//        {
//            mt9v03x_finish_flag = 0;         
//            // ?????j????????????????????????????????????????? mt9v03x_image[0] image_OTSU[0]
//            memcpy(image_copy[0],bin_image[0], MT9V03X_IMAGE_SIZE); 		
//            // ???????
//            seekfree_assistant_camera_send(); 
//        }	
		
        // ???????????????J???
    }
}

//-------------------------------------------------------------------------------------------------------------------
// ???????     PIT ???????????? ??????????? PIT ????K?????????? ??? isr.c
// ???????     void
// ???????     void
// '?????     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (void)
{	
	static int cnt = 0,cnt2 = 0,cnt3 = 0,cnt4 = 0,cnt5 = 0,danger = 0;
	static uint8 push_flag = 1;//1 ->right
    encoder_update();
	odometer_update();
	//calculate_box();
	switch (moveflag){
		case 1://W
//			if(G_control(&move_z,g_odometer.distance,60))
//			SetMotion(MODE_STOP,0,0,0);break;
			out_process();
			if(cnt3<80){control_speed = 0.5;cnt3++;}
			else{
//			if(crossroad_flag && ! roundabout_entry_flag){control_speed = 1;}//1  1.4
			if(roundabout_entry_flag == 5 || roundabout_entry_flag == 6){control_speed = 1.1;}// 0.9    0.8
			else{control_speed = 1.2;}// 1    1.2
			}
			
			//printf("%d\n",roundabout_entry_flag);
			track_control(control_speed);
			if(cal_flag && !lock_box)
			{
			if(check_box())
			{	
				cnt++;
				if(cnt>1)
				{
					SetMotion(MODE_STOP,0,0,0);
//					if(roundabout_entry_flag == 5 || roundabout_entry_flag == 6){moveflag = 4;}
//					else {moveflag = 5;}
					moveflag = 5;
					cnt = 0;
				}
				
			}
			}
			if(final)moveflag = 2;
			if(p_odometer.distance>50){lock_box = 0;}
			break;
		case 2://S
			SetMotion(MODE_STOP,0,0,0);
			show_results();
			break;
		case 3://A
			if(push_flag&&!cnt2)
			{	
				if(Is_right)
				{
					if(push2r_control_dun())
					{
						cnt2++;	
					}
				}
				else
				{
					if(push2r_control())
					{
						cnt2++;	
					}
					
				}
				
			}
			else if(!cnt2){
				if(Is_right)
				{
					if(push2l_control_dun())
					{
						cnt2++;	
					}
				}
				else
				{
					if(push2l_control())
					{
						cnt2++;	
					}
				}
			}		
			if(cnt2)
			{
				SetMotion(MODE_STOP,0,0,0);
				cnt2++;
				if(cnt2>25)//    original 50
				{
					moveflag = 1;lock_box = 1;
					cnt3 = 0;
					cnt2 = 0;
				}
			}
//			if(P_control(&move_p,p_odometer.distance,-120,50))
//			{
//				SetMotion(MODE_STOP,0,0,0);
//			}
			break;
		case 4://D identify
			if(art_identify())
			{
				SetMotion(MODE_STOP,0,0,0);
				moveflag = 3;
				push_flag = push_flag_update();
				p_Odometer_Init();
				g_Odometer_Init();
				cnt4 = 0;
				danger = 0;
			}
			else
			{
				cnt4++;
				danger++;
				if(cnt4 > 300)
				{	
					cnt4 = 0;
					moveflag = 5;//aim again or go next?
				}
				if(danger > 300*3)
				{
					moveflag = 1;
					danger = 0;
					p_Odometer_Init();
					g_Odometer_Init();
					lock_box = 1;
					cnt4 = 0;
					results[count_box++] = 0;
					pre_index = 200;
				}
			}
			//SetMotion(MODE_FORWARD,1,0,0);
			break;
		case 5:		
			if(aim_control_2())
			{
				SetMotion(MODE_STOP,0,0,0);
				angle_adj = z_odometer.distance;
				moveflag =4;
			}
			//SetMotion(MODE_FORWARD,1,0,0);
			break;
		case 6:
			if(cnt5<50)
			{SetMotion(MODE_STOP,0,0,0);cnt5++;}
			else{
			if(track_control_2())
			{
				moveflag =1;
				cnt5 = 0;
				cnt3 = 0;
			}
			}
			break;	
		case 7:
			if(G_control(&move_z,g_odometer.distance,-145))
			{
				SetMotion(MODE_STOP,0,0,0);
				p_Odometer_Init();
				g_Odometer_Init();
				roundabout_entry_flag = 5;
			}
			break;
		break;
		default:break;
	}
	//track_control(0.8);
	//calculate_box();
	Motor_Speedcontrol(&m1,&m2,&m3); //pid controller 
	
	//printf("%d,%d,%d\n",box_x,box_y,box_area);
	//printf("err:%4d\n", track_error);
	//printf("S:%d,%d,%d,%d,%f,%f\n", encoder_data_1,encoder_data_2,encoder_data_3,m1.target_speed,g_odometer.distance);                 // ????????????????
	//printf("S:%d,%d,%d,%d\n",encoder_data_1,encoder_data_2,encoder_data_3,m1.target_speed); 
}
void pit1_handler (void)
{
	if(moveflag == 1 || moveflag == 6)
    {
	cal_flag = 0;
	uart_rx_interrupt(UART_INDEX, 0);
	ImageProcess();
	uart4_init();
	cal_flag = 1;	
	}
	else{
	uart_rx_interrupt(UART_INDEX, 1);
	cal_flag = 1;
	}
	time_flag = 1;
	//calculate_box();
}
void pit2_handler (void)
{
	imu660ra_get_acc();  
	imu660ra_get_gyro();
	z_odometer.raw_speed = (imu660ra_gyro_z-0.786)/2000*360/488*180/219.5*5;//??????t
	z_odometer.filtered_speed = MovingAverageFilter_z(z_odometer.raw_speed);
	z_odometer.distance += z_odometer.filtered_speed;
}
int art_identify()
{
	calculate_box();
	if(box_y<FIND_BOX){moveflag = 5;}
	if(index == 200 && pre_index!= 200)
	{
		if(pre_index == 201)Is_right = 1;
		else if(pre_index == 202)Is_right = 0;
		else{
		current_result = pre_index;
		results[count_box++] = pre_index;
		pre_index = 200;
		return 1;}
	}
	if(index != 200){pre_index = index;}
	return 0;
}