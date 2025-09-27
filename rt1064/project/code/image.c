//-------------------------------------------------------------------------------------------------------------------
//  功能：Otsu算法实现 + 二值化图像显示
//------------------------------------------------------------------------------------------------------------------
#include "image.h"
#include "zf_common_headfile.h"
#include "motor.h"
#include <math.h>
#include <limits.h>  // ????????INT_MIN
// 全局变量定义
uint8 bin_image[image_h][image_w];      // 二值化图像
uint8 morph_image[image_h][image_w]; // 形态学处理中间图像
BorderPoint left_border[image_h] ;  // 左边界点集合
BorderPoint right_border[image_h]; // 右边界点集合
CenterLine center_line[image_h] ; // 存储中线坐标
uint8 crossroad_flag = 0;          // 十字路口标志位
int track_error = 0;  // 存储计算得到的误差
float left_qulve = 0; // 左轮曲率
float right_qulve = 0; // 右轮曲率
int left_loss = 0;
int right_loss = 0;
int lock_cross = 0; //环岛锁
uint8 roundabout_entry_flag = 0;    // 环岛状态标志
static uint8 right_lost_counter = 0; // 右边界丢失计数器
#define RIGHT_LOST_THRESHOLD   5    // 右边界连续丢线阈值（帧数）
extern OdometerTypeDef g_odometer,p_odometer,z_odometer;

 // 大津法计算最佳阈值
 uint8 otsu_threshold() {
     int histogram[256] = {0};
     float sum = 0.0, sumB = 0.0;
     int total = image_h * image_w;

     // 计算直方图
     for (int y = 0; y < image_h; y++) {
         for (int x = 0; x < image_w; x++) {
             histogram[mt9v03x_image[y][x]]++;
         }
     }

     // 计算总灰度值
     for (int i = 0; i < 256; i++) {
         sum += i * histogram[i];
     }

     float varMax = 0.0;
     uint8 best_th = 0;
     float wB = 0.0;  // 前景权重
     float wF = 0.0;  // 背景权重

     for (int t = 0; t < 256; t++) {
         wB += histogram[t];          // 前景像素数累加
         if (wB == 0) continue;

         wF = total - wB;             // 背景像素数
         if (wF == 0) break;

         sumB += (float)(t * histogram[t]);

         float mB = sumB / wB;        // 前景平均灰度
         float mF = (sum - sumB) / wF;// 背景平均灰度

         // 计算类间方差
         float varBetween = wB * wF * (mB - mF) * (mB - mF);
        
         // 更新最大方差和阈值
         if (varBetween > varMax) {
             varMax = varBetween;
             best_th = t;
         }
     }
     return best_th;
 }
 
 // 平均灰度法二值化处理
 void threshold(void)
 {
 	int img_pixel_value_avg = 0;
 	int16 sampling_num = 0;
 	int16 x,y;
 	for(x = 0;x < MT9V03X_W;)
 	{
 		for(y = 0;y < MT9V03X_H;)
 		{
 			img_pixel_value_avg += mt9v03x_image[y][x];
 			sampling_num++;
 			y+=THRESHOLD_SAMPLING_DISTANCE;
 		}
 		x+=THRESHOLD_SAMPLING_DISTANCE;
 	}
		
 	img_pixel_value_avg = img_pixel_value_avg/sampling_num;
		
 	for(x = 0;x < MT9V03X_W;x++)
 	{
 		for(y = 0;y < MT9V03X_H;y++)
 		{
 			if(mt9v03x_image[y][x] >= img_pixel_value_avg)
 			{
 				bin_image[y][x] = 255;
 			}
 			else
 			{
 				bin_image[y][x] = 0;
 			}
 		}
 	}
 }

 // 图像二值化处理
 void image_binarization(uint8 threshold) {
     for (int y = 0; y < image_h; y++) {
         for (int x = 0; x < image_w; x++) {
             bin_image[y][x] = (mt9v03x_image[y][x] > threshold) ? 255 : 0;
         }
     }
 }

uint8_t otsu_threshold_new() {
    int histogram[256] = {0};
    float sum = 0.0, sumB = 0.0;
    int total_valid = 0;  // ???????(??>240???)

    // ?????(??>240???)
    for (int y = 0; y < image_h; y++) {
        for (int x = 0; x < image_w; x++) {
            uint8_t pixel = mt9v03x_image[y][x];
            if (pixel <= 240 ) {  // ???<=240???
                histogram[pixel]++;
                total_valid++;
            }
        }
    }

    // ???????????
    for (int i = 0; i <= 240; i++) {  // ???0-240????
        sum += i * histogram[i];
    }

    float varMax = 0.0;
    uint8_t best_th = 0;
    float wB = 0.0;  // ????

    for (int t = 0; t < 256; t++) {
        wB += histogram[t];  // ???????
        
        if (wB == 0) continue;
        float wF = total_valid - wB;  // ????
        if (wF <= 0) break;          // ???????

        sumB += (float)(t * histogram[t]);  // ??????

        float mB = sumB / wB;         // ??????
        float mF = (sum - sumB) / wF; // ??????

        // ??????
        float varBetween = wB * wF * (mB - mF) * (mB - mF);
        
        // ???????????
        if (varBetween > varMax) {
            varMax = varBetween;
            best_th = t;
        }
    }
    return best_th;
}

void erosion(uint8 src[image_h][image_w], uint8 dst[image_h][image_w]) {
    for(int y = BORDER; y < image_h - BORDER; y++) {
        for(int x = BORDER; x < image_w - BORDER; x++) {
            uint8 min_val = 255;
            
            // 遍历结构元素邻域
            for(int ky = -1; ky <= 1; ky++) {
                for(int kx = -1; kx <= 1; kx++) {
                    if(src[y+ky][x+kx] < min_val) {
                        min_val = src[y+ky][x+kx];
                    }
                }
            }
            dst[y][x] = min_val;
        }
    }
}

// 膨胀处理函数 
void dilation(uint8 src[image_h][image_w], uint8 dst[image_h][image_w]) {
    for(int y = BORDER; y < image_h - BORDER; y++) {
        for(int x = BORDER; x < image_w - BORDER; x++) {
            uint8 max_val = 0;
            
            // 遍历结构元素邻域
            for(int ky = -1; ky <= 1; ky++) {
                for(int kx = -1; kx <= 1; kx++) {
                    if(src[y+ky][x+kx] > max_val) {
                        max_val = src[y+ky][x+kx];
                    }
                }
            }
            dst[y][x] = max_val;
        }
    }
}

//边框预处理函数
void before_process()
{
	int y,x;
	for (x = 0; x < image_w; x++)
	{
		bin_image[0][x] = 0;
		bin_image[image_h - 1][x] = 0;
	}
	for (y = 0; y < image_h; y++)
	{
		bin_image[y][0] = 0;
		bin_image[y][image_w - 1] = 0;
	}
	
}

// 整合后的边界与中线计算函数
void find_borders_and_centerline() {
    const int start_y = image_h - 5;               // 起始行：底部向上5行
    const int end_y = image_h / 8;                 // 结束行：图像上部1/4处
    static int prev_center_x = image_w / 2;        // 上一行的中线位置（初始为图像中心）
    static int16 num = 0;
	if(num < 10)
	{
		prev_center_x = image_w/2;
		num++;
	}
	else
	{
		prev_center_x = center_line[35].x;
	}
    // 遍历从底部向上5行到图像1/4处的所有行
    for (int y = start_y; y >= end_y; y--) {
        //--- 初始化边界和中线 ---
        center_line[y].x = prev_center_x; // 中线初始化为上一行位置
        center_line[y].valid = 0;         // 默认中线无效
        
        //--- 动态搜索起点：基于上一行中线 ---
        int search_start = prev_center_x;
        search_start = (search_start < 0) ? 0 : 
                      (search_start >= image_w) ? image_w - 1 : search_start;

        //--- 搜索左边界：从起点向左扫描 ---
        int left_found = 0;
        for (int x = search_start; x >= 0; x--) {
            if (bin_image[y][x] == 0) {
                left_border[y].x = x;
                left_found = 1;
                break;
            }
        }
        // 未找到左边界时设为最左侧
        if (!left_found) left_border[y].x = 0;

        //--- 搜索右边界：从起点向右扫描 ---
        int right_found = 0;
        for (int x = search_start; x < image_w; x++) {
            if (bin_image[y][x] == 0) {
                right_border[y].x = x;
                right_found = 1;
                break;
            }
        }
        // 未找到右边界时设为最右侧
        if (!right_found) right_border[y].x = image_w - 1;

        //--- 计算中线 ---
        if (left_found && right_found) {
            // 情况1：双边界有效，中线为平均值
            center_line[y].x = (left_border[y].x + right_border[y].x) / 2;
            center_line[y].valid = 1;
        } else {
            // 情况2：单边无效，中线继承上一行有效值（不更新prev_center_x）
             center_line[y].x = image_w / 2;
            center_line[y].valid = 0;
        }

        //--- 更新下一行搜索起点 ---
        if (center_line[y].valid) {
            prev_center_x = center_line[y].x; // 仅有效时更新
        }
    }
}

//====================== 丢线判断函数 ======================//
int8 is_border_lost(BorderPoint border[image_h]) {
    int lost_count = 0;
    int total_rows = ELEMENT_START_Y - ELEMENT_END_Y + 1; // 总行数
    
    // 遍历元素识别区（从下往上）
    for (int y = ELEMENT_START_Y; y >= ELEMENT_END_Y; y--) {
        if (border[y].x <= 8 || border[y].x >= image_w - 9 ) { // 边界无效3/6 7/8
            lost_count++;
        }
    }
    
    // 判断丢线比例是否超过2/3（四舍五入）
    int threshold = (total_rows + 2) / 3; // 等价于ceil(total_rows/4.0)
    return (lost_count >= 2*threshold) ? 1 : 0;
}

int8 is_border_lost2(BorderPoint border[image_h]) {
    int lost_count = 0;
    int total_rows = DUIXIAN2_START_Y - DUIXIAN2_END_Y + 1; // 总行数
    
    // 遍历元素识别区（从下往上）
    for (int y = DUIXIAN2_START_Y; y >= DUIXIAN2_END_Y; y--) {
        if (border[y].x <= 5 || border[y].x >= image_w - 6 ) { // 边界无效
		//if (border[y].x <= 5 || border[y].x >= image_w - 6 ) { // 边界无效
            lost_count++;
        }
    }
    
    // 判断丢线比例是否超过2/3（四舍五入）
    int threshold = (total_rows + 2) / 3; // 等价于ceil(total_rows/4.0)
    return (lost_count >= 2*threshold) ? 1 : 0;
}

int8 is_border_lost3(BorderPoint border[image_h]) {
    int lost_count = 0;
    int total_rows = DUIXIAN3_START_Y - DUIXIAN3_END_Y + 1; // 总行数
    
    // 遍历元素识别区（从下往上）
    for (int y = DUIXIAN3_START_Y; y >= DUIXIAN3_END_Y; y--) {
        //if (border[y].x <= 3 || border[y].x >= image_w - 5 ) { // 边界无效
		if (border[y].x <= 5 || border[y].x >= image_w - 6 ) { // 边界无效
            lost_count++;
        }
    }
    
    // 判断丢线比例是否超过2/3（四舍五入）
    int threshold = (total_rows + 2) / 3; // 等价于ceil(total_rows/4.0)
    return (lost_count >= 2*threshold) ? 1 : 0;
}

int8 is_border_lost_left(BorderPoint border[image_h]) {
    const int consecutive_points = 3;  // 连续检测点数
    const int max_diff = 15;           // 最大允许差值
    
    // 遍历元素识别区（从下往上，确保有足够的上方点数）
    for (int y = DUIXIAN_START_Y; y >= DUIXIAN_END_Y + consecutive_points - 1; y--) {
        // 跳过无效基准点
        if (border[y].x == 0xFFFF) continue;

        int base_x = border[y].x; // 基准点a的x坐标
        int valid = 1;
        
        // 检查后续连续4个点（共5个点：y到y-4）
        for (int i = 1; i < consecutive_points; i++) {
            int current_y = y - i;
            
            // 有效性检查
            if (current_y < DUIXIAN_END_Y) {
                valid = 0;
                break;
            }
            
            // 差值检查
            if ( base_x - border[current_y].x <= max_diff) {
                valid = 0;
                break;
            }
        }
        
        // 满足连续5点差值均超阈值则判定丢线
        if (valid) {
            return 1;
        }
    }
    return 0;
}

int8 is_border_lost_right(BorderPoint border[image_h]) {
    const int consecutive_points = 3;  // 连续检测点数
    const int max_diff = 15;           // 最大允许差值
    
    // 遍历元素识别区（从下往上，确保有足够的上方点数）
    for (int y = DUIXIAN_START_Y; y >= DUIXIAN_END_Y + consecutive_points - 1; y--) {
        // 跳过无效基准点
        if (border[y].x == 0xFFFF) continue;

        int base_x = border[y].x; // 基准点a的x坐标
        int valid = 1;
        
        // 检查后续连续4个点（共5个点：y到y-4）
        for (int i = 1; i < consecutive_points; i++) {
            int current_y = y - i;
            
            // 有效性检查
            if (current_y < DUIXIAN_END_Y) {
                valid = 0;
                break;
            }
            
            // 差值检查
            if (border[current_y].x - base_x <= max_diff) {
                valid = 0;
                break;
            }
        }
        
        // 满足连续5点差值均超阈值则判定丢线
        if (valid) {
            return 1;
        }
    }
    return 0;
}


//====================== 边界直线判断函数 ======================//
int8 is_border_straight(BorderPoint border[image_h]) {
    const int num_points = 6;        // 取6个点
    int valid_segments = 0;          // 有效差值计数
    int total_rows = ELEMENT_START_Y - ELEMENT_END_Y + 1;

    // 行数不足时直接返回非直线
    if (total_rows < 5) return 0;

    // 计算采样间隔（均匀分布5个间隔）
    int interval = (ELEMENT_START_Y - ELEMENT_END_Y) / 5;
    int y_coords[num_points];

    // 生成采样点y坐标（从下往上）
    for (int i = 0; i < num_points; i++) {
        y_coords[i] = ELEMENT_START_Y - i * interval;
        // 边界保护
        if (y_coords[i] < ELEMENT_END_Y) y_coords[i] = ELEMENT_END_Y;
    }

    // 检查所有采样点是否有效
     if (is_border_lost(border)) return 0;
    

    // 计算相邻点差值并统计
    for (int i = 0; i < num_points - 1; i++) {
        int dx = abs(border[y_coords[i]].x - border[y_coords[i+1]].x);
        if (dx < 6) valid_segments++;
    }

    // 判断是否满足5段中至少4段有效
    return (valid_segments >= 4) ? 1 : 0;
}

//====================== 曲率计算函数 ======================//
float calculate_border_curvature_false(BorderPoint border[image_h]) {
    const int sample_points = 6;    // 采样点数量
    int valid_samples = 0;          // 有效点计数
    float total_curvature = 0.0f;   // 曲率累加值
    int total_rows = ELEMENT_START_Y - ELEMENT_END_Y + 1;

    // 行数不足时返回0（无法计算）
    if (total_rows < sample_points) return 0.0f;

    // 计算采样间隔（均匀分布）
    int interval = (ELEMENT_START_Y - ELEMENT_END_Y) / (sample_points - 1);
    int y_coords[sample_points];

    // 生成采样点坐标（从下往上）
    for (int i = 0; i < sample_points; i++) {
        y_coords[i] = ELEMENT_START_Y - i * interval;
        y_coords[i] = (y_coords[i] < ELEMENT_END_Y) ? ELEMENT_END_Y : y_coords[i];
    }

    // 检查前三个点有效性（至少需要3个点计算曲率）
    for (int i = 0; i < 3; i++) {
        if (border[y_coords[i]].x == 0xFFFF) return 1000.0f; // 返回大值表示无效
    }

    // 三点法计算曲率（遍历中间点）
    for (int i = 1; i < sample_points - 1; i++) {
        int y_prev = y_coords[i-1];
        int y_curr = y_coords[i];
        int y_next = y_coords[i+1];
        
        // 跳过无效点
        if (border[y_prev].x == 0xFFFF || 
            border[y_curr].x == 0xFFFF || 
            border[y_next].x == 0xFFFF) continue;

        // 计算二阶差分近似曲率（简化版）
        int dx_prev = border[y_curr].x - border[y_prev].x;
        int dx_next = border[y_next].x - border[y_curr].x;
        float curvature = fabs(dx_next - dx_prev);

        total_curvature += curvature;
        valid_samples++;
    }

    // 计算平均曲率（至少需要2个有效段）
    if (valid_samples >= 2) {
        return total_curvature / valid_samples;
    }
    return 0.0f; // 数据不足时视为直线
}

float calculate_border_curvature(BorderPoint border[]) {
    // 根据实际数据范围确定循环边界（示例值需替换）
 
    // 边界检查
    if (DUIXIAN_START_Y <= DUIXIAN_END_Y + 1) {
        return 0.0f; // 无效范围
    }
    
    float total_curvature = 0.0f;
    int valid_points = 0;
    
    // 移动平均窗口参数
    const int window_size = 3; // 移动平均窗口大小
    float curvature_buffer[3] = {0}; // 曲率缓冲区
    int buffer_index = 0;
    
    // 逆向循环
    for (int i = DUIXIAN_START_Y - 1; i > DUIXIAN_END_Y + 1; i--) {
        // 获取相邻三个点
        float x_prev = (float)border[i+1].x;
        float x_curr = (float)border[i].x;
        float x_next = (float)border[i-1].x;
        
        // 计算导数
        float dx_di = (x_next - x_prev) / 2.0f;
        float d2x_di2 = x_next - 2*x_curr + x_prev;
        
        // 曲率计算
        float denominator = powf(dx_di*dx_di + 1.0f, 1.5f);
        if (fabsf(denominator) > 1e-5f) {
            float curvature = fabsf(d2x_di2) / denominator;
            
            // 使用移动平均平滑曲率
            curvature_buffer[buffer_index] = curvature;
            buffer_index = (buffer_index + 1) % window_size;
            
            // 计算移动平均值
            float smoothed_curvature = 0.0f;
            int count = 0;
            for (int j = 0; j < window_size; j++) {
                if (curvature_buffer[j] > 0) { // 仅使用有效值
                    smoothed_curvature += curvature_buffer[j];
                    count++;
                }
            }
            
            if (count > 0) {
                smoothed_curvature /= count;
                
                // 曲率阈值过滤：忽略异常突变
                if (smoothed_curvature < 0.5f) { // 阈值可根据实际情况调整
                    total_curvature += smoothed_curvature;
                    valid_points++;
                }
            }
        }
    }
    
    return (valid_points > 0) ? (total_curvature / valid_points) : 0.0f;
}
/*-------------------补线函数-----------------------*/
/**
 * @brief 在二值化图像上绘制3像素宽的黑线（抗锯齿处理）
 * @param x0,y0 起点坐标
 * @param x1,y1 终点坐标
 */
void draw_thick_line(uint8 x0, uint8 y0, uint8 x1, uint8 y1) {
    int dx = abs(x1 - x0);
    int dy = -abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int e2;

    // 主循环：Bresenham算法核心
    while (1) {
        // 绘制3像素宽点（核心点+两侧扩展）
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                int px = x0 + i;
                int py = y0 + j;
                // 边界检查
                if (px >= 0 && px < image_w && py >= 0 && py < image_h) {
                    bin_image[py][px] = 0; // 设置黑色
                }
            }
        }

        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        
        // 计算下一个点
        if (e2 >= dy) { 
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) { 
            err += dx;
            y0 += sy;
        }
    }
}
/*********** 计算跳变点数量 ***********/
uint8 jump_point_sum(uint8 my_line)
{
    uint8 jump_point=0;
    uint8 start,end,i;
    start=1;
    end=image_w-1;
    for(i=start+1;i<end;i++)
    {
        if(bin_image[my_line][i]!=bin_image[my_line][i-1])
        {
            jump_point++;
        }
    }
    return jump_point;
}


/*******判断是否到达终点**************/
int destination_flag = 0;
/**
 * @brief 检测是否到达终点
 * @param start_point 起始行
 * @param end_point 结束行
 */
void detectDestination(int start_point, int end_point){
	uint8 i,point_sum=0;
    int des_num = 0;
    int des_row_num;
	for(i=start_point;i<=end_point;i++){
        if(jump_point_sum(i)> 7){                        //8
            des_num++;
            des_row_num = i;
        }
			
	}
    if(des_num>5){
        destination_flag = 1;
        for(int i = 1;i<image_w;i++){
            bin_image[des_row_num][i] = 0;
        }
    }
    else{
        destination_flag = 0;
    }
	
}


void crossroad_repair() {
    // 寻找左边界角点
    int left_corner_y = -1;
    for (int y = image_h *3/4; y >= image_h / 4; y--) {
        if (left_border[y].x != 0xFFFF) { // 确保边界点有效
            int valid = 1;
            // 检查下面连续5个点
            for (int i = 1; i <= 5; i++) {
                int check_y = y + i + 7;
                if (check_y >= image_h) {
                    valid = 0;
                    break;
                }
                if (abs(left_border[y].x - left_border[check_y].x) < 17) {
                    valid = 0;
                    break;
                }
            }
            if (valid) {
                left_corner_y = y;
                break;
            }
        }
    }
    
    // 寻找右边界角点
    int right_corner_y = -1;
    for (int y = image_h - 1; y >= image_h / 4; y--) {
        if (right_border[y].x != 0xFFFF) { // 确保边界点有效
            int valid = 1;
            // 检查下面连续5个点
            for (int i = 1; i <= 5; i++) {
                int check_y = y + 7 + i;
                if (check_y >= image_h) {
                    valid = 0;
                    break;
                }
                if (abs(right_border[y].x - right_border[check_y].x) < 17) {
                    valid = 0;
                    break;
                }
            }
            if (valid) {
                right_corner_y = y;
                break;
            }
        }
    }
    
    // 补线操作
    if (left_corner_y != -1 && right_corner_y != -1) {
        // 从左边界角点画线到图像下侧中点
        draw_thick_line(left_border[left_corner_y -7].x, left_corner_y-5, 
                        image_w / 2 - 25, image_h - 1);
        draw_thick_line(right_border[right_corner_y -7].x, right_corner_y -5, 
                        image_w / 2 + 25, image_h - 1);
    }

}

void handle_crossroad2() {
    //------------------------ 状态流转 ------------------------//

    switch (crossroad_flag) {
        // 状态0：环岛检测
        case 0: {
            if (is_border_lost2(left_border) && is_border_lost2(right_border) && roundabout_entry_flag == 0) 
            {
                crossroad_flag = 1;
				p_Odometer_Init();
            }
            break;
        }
        // 状态1：入十字路口
        case 1: {
            if (is_border_lost3(left_border) && is_border_lost3(right_border) && roundabout_entry_flag == 0) //出十字路口
            {
				p_Odometer_Init();
                crossroad_flag = 2;
							
            }
            if (!is_border_lost2(left_border) || !is_border_lost2(right_border)) 
            {
                crossroad_flag = 0; //如果左边界或右边界恢复，回到状态0,防止误识别
            }
            break;
        }
		case 2: {
             if (p_odometer.distance>40) //行驶出十字路口
            {
                crossroad_flag = 0;
            }
            break;
        }
       
    }
}

void handle_crossroad_process() {
    //------------------------ 状态流转 ------------------------//
    switch (crossroad_flag) {
        // 状态0：环岛检测
        case 0: {
            break;
        }
        // 状态1：入十字路口
        case 1: {
            draw_thick_line(0, 119,52,15);//画左边线
			draw_thick_line(179, 119,128,15);//画右边线
            break;       
        }
		case 2: {
            crossroad_repair();
            break;
        }
       
    }
}

void handle_roundabout_change() {
    //------------------------ 状态流转 ------------------------//
    left_qulve = is_border_lost(left_border)>0?0:calculate_border_curvature(left_border);
    right_qulve = is_border_lost(right_border)>0?0:calculate_border_curvature(right_border);
    static int cnt1  = 0;
	static int cnt2  = 0;
    switch (roundabout_entry_flag) {
        // 状态0：环岛检测
        case 0: {
//            if (is_border_lost2(left_border) && right_qulve>0.3 && is_border_straight2(right_border)&&!is_border_lost2(right_border))// 右边界直线且左边界丢失
            if (is_border_lost_left(left_border) && !is_border_lost_right(right_border) && right_qulve<0.1 && !crossroad_flag)//左丢线，补左直线，位置环初始化
            {
				cnt1 ++;
            }	
			if (cnt1 > 8){
			roundabout_entry_flag = 1;
            p_Odometer_Init();
			cnt1 = 0;
			}
//            if (is_border_lost2(left_border) && right_qulve>0.3 && is_border_straight2(right_border)&&!is_border_lost2(right_border))// 右边界直线且左边界丢失
            if (is_border_lost_right(right_border) && !is_border_lost_left(left_border) && left_qulve<0.1 && !crossroad_flag)//左丢线，补左直线，位置环初始化
            {
				cnt2 ++;
            }	
			if (cnt2 > 8){
			roundabout_entry_flag = 2;
            p_Odometer_Init();
			cnt2 = 0;
			}
            break;
        }
        // 状态1：入环检测
        case 1: {
			if (is_border_lost_left(left_border) && is_border_lost_right(right_border) && p_odometer.distance < 50)//误入十字路口
			{
				roundabout_entry_flag = 0;
			}
			if (is_border_lost(left_border) && is_border_lost(right_border) && p_odometer.distance < 50)//误入十字路口
			{
				roundabout_entry_flag = 0;
			}
//			if (is_border_lost_right(right_border))
//			 {
//				 roundabout_entry_flag = 0;
//			 }
            if (p_odometer.distance>DISTANCE_1) //行驶到环岛，补左弯道，陀螺仪启动
            {
                roundabout_entry_flag = 3;z_Odometer_Init();
            }   
			 z_Odometer_Init();
            break;
        }
		case 2: {
             if (p_odometer.distance>DISTANCE_1) //行驶到环岛，补左弯道，陀螺仪启动
            {
                roundabout_entry_flag = 4;z_Odometer_Init();
 
            }   
			if (is_border_lost_right(right_border) && is_border_lost_left(left_border) && p_odometer.distance < 40 )//误入十字路口
			{
				roundabout_entry_flag = 0;
			}
			if (is_border_lost(left_border) && is_border_lost(right_border) && p_odometer.distance < 40 )//误入十字路口
			{
				roundabout_entry_flag = 0;
			}
//			if (is_border_lost_left(left_border))
//			 {
//				 roundabout_entry_flag = 0;
//			 }
			 z_Odometer_Init();
            break;
        }
        // 状态3：入弯道内循迹
        case 3: {
			lock_cross = 1; //锁住十字检测
             if(z_odometer.distance>45)
             {roundabout_entry_flag = 5;}
            break;
        }
		case 4: {
			lock_cross = 1; //锁住十字检测
             if(z_odometer.distance< -45)
             {
				 roundabout_entry_flag = 6;
			 }
            break;
        }
        // 状态5：出弯道循迹
        case 5: {
             if (is_border_lost(left_border) && is_border_lost(right_border)) {//左右丢线或陀螺仪重新启动或魔法参数，补右弯道
				roundabout_entry_flag = 7;
				z_Odometer_Init();
            }
            break;
        }
		case 6: {
             if (is_border_lost(left_border) && is_border_lost(right_border)) {//左右丢线或陀螺仪重新启动或魔法参数，补右弯道
				roundabout_entry_flag = 8;
				z_Odometer_Init();
			
            }
            break;
        }
        // 状态7：出环岛
        case 7: {
             if (!is_border_lost(right_border) && is_border_straight(right_border) && z_odometer.distance > 30) {//补左直线
			 //if (!is_border_lost(right_border) && is_border_straight(right_border)) {//补左直线
				roundabout_entry_flag = 9;
				p_Odometer_Init();
            }
            break;
        }
		case 8: {
            if (!is_border_lost(left_border) && is_border_straight(left_border) && z_odometer.distance< -30) {//补左直线
			//if (!is_border_lost(left_border) && is_border_straight(left_border)) {//补左直线
				roundabout_entry_flag = 10;
				p_Odometer_Init();
            }
            break;
        }
        case 9: {
             if (is_border_straight(left_border) && is_border_straight(right_border) && p_odometer.distance>DISTANCE_2 )//出环岛入正常状态，右边界直线且左边界直线 
            {
                lock_cross = 0; //解锁
                roundabout_entry_flag = 0;//退出环岛状态
				cnt1 = 0;
            }
            break;
        }
		case 10: {
             if (is_border_straight(left_border) && is_border_straight(right_border) && p_odometer.distance>DISTANCE_2 )//出环岛入正常状态，右边界直线且左边界直线 
            {
                lock_cross = 0; //解锁
                roundabout_entry_flag = 0;//退出环岛状态
				cnt2 = 0;
            }
            break;
        }
    }
 }

 void handle_roundabout_change_new() {
    //------------------------ 状态流转 ------------------------//
    left_qulve = is_border_lost(left_border)>0?0:calculate_border_curvature(left_border);
    right_qulve = is_border_lost(right_border)>0?0:calculate_border_curvature(right_border);
    static int cnt1  = 0;
	static int cnt2  = 0;
    switch (roundabout_entry_flag) {
        // 状态0：环岛检测
        case 0: {
//            if (is_border_lost2(left_border) && right_qulve>0.3 && is_border_straight2(right_border)&&!is_border_lost2(right_border))// 右边界直线且左边界丢失
            if (is_border_lost_left(left_border) && !is_border_lost_right(right_border) && right_qulve<0.1 && !crossroad_flag)//左丢线，补左直线，位置环初始化
            {
				cnt1 ++;
            }	
			if (cnt1 > 6){
			roundabout_entry_flag = 1;
            p_Odometer_Init();
			cnt1 = 0;
			}
//            if (is_border_lost2(left_border) && right_qulve>0.3 && is_border_straight2(right_border)&&!is_border_lost2(right_border))// 右边界直线且左边界丢失
            if (is_border_lost_right(right_border) && !is_border_lost_left(left_border) && left_qulve<0.1 && !crossroad_flag)//左丢线，补左直线，位置环初始化
            {
				cnt2 ++;
            }	
			if (cnt2 > 6){
			roundabout_entry_flag = 2;
            p_Odometer_Init();
			cnt2 = 0;
			}
            break;
        }
        // 状态1：入环检测
        case 1: {
			if (is_border_lost_left(left_border) && is_border_lost_right(right_border) && p_odometer.distance < 15)//误入十字路口
			{
				roundabout_entry_flag = 0;
			}
			if (is_border_lost(left_border) && is_border_lost(right_border) && p_odometer.distance < 15)//误入十字路口
			{
				roundabout_entry_flag = 0;
			}
//			if (is_border_lost_right(right_border))
//			 {
//				 roundabout_entry_flag = 0;
//			 }
            if (p_odometer.distance>DISTANCE_3 - 15) //行驶到环岛，补左弯道，陀螺仪启动,speed 1 is 10
            {
                roundabout_entry_flag = 3;z_Odometer_Init();
            }   
            break;
        }
		case 2: {
             if (p_odometer.distance>DISTANCE_3 -10) //行驶到环岛，补左弯道，陀螺仪启动
            {
                roundabout_entry_flag = 4;z_Odometer_Init();
 
            }   
			if (is_border_lost_right(right_border) && is_border_lost_left(left_border) && p_odometer.distance < 15 )//误入十字路口
			{
				roundabout_entry_flag = 0;
			}
			if (is_border_lost(left_border) && is_border_lost(right_border) && p_odometer.distance < 15 )//误入十字路口
			{
				roundabout_entry_flag = 0;
			}
//			if (is_border_lost_left(left_border))
//			 {
//				 roundabout_entry_flag = 0;
//			 }
            break;
        }
        // 状态3：入弯道内循迹
        case 3: {
			lock_cross = 1; //锁住十字检测
             if(z_odometer.distance>70)
             {roundabout_entry_flag = 5;}
            break;
        }
		case 4: {
			lock_cross = 1; //锁住十字检测
             if(z_odometer.distance< -70)
             {
				 roundabout_entry_flag = 6;
			 }
            break;
        }
        // 状态5：出弯道循迹
        case 5: {
             if (is_border_lost(left_border) && is_border_lost(right_border)) {//左右丢线或陀螺仪重新启动或魔法参数，补右弯道
				roundabout_entry_flag = 7;
				z_Odometer_Init();
            }
            break;
        }
		case 6: {
             if (is_border_lost(left_border) && is_border_lost(right_border)) {//左右丢线或陀螺仪重新启动或魔法参数，补右弯道
				roundabout_entry_flag = 8;
				z_Odometer_Init();
			
            }
            break;
        }
        // 状态7：出环岛
        case 7: {
             if (!is_border_lost(right_border) && is_border_straight(right_border) && z_odometer.distance > 50) {//补左直线
			 //if (!is_border_lost(right_border) && is_border_straight(right_border)) {//补左直线
				roundabout_entry_flag = 9;
				p_Odometer_Init();
            }
            break;
        }
		case 8: {
            if (!is_border_lost(left_border) && is_border_straight(left_border) && z_odometer.distance< -50) {//补左直线
			//if (!is_border_lost(left_border) && is_border_straight(left_border)) {//补左直线
				roundabout_entry_flag = 10;
				p_Odometer_Init();
            }
            break;
        }
        case 9: {
             if (is_border_straight(left_border) && is_border_straight(right_border) && p_odometer.distance>25 )//出环岛入正常状态，右边界直线且左边界直线 
            {
                lock_cross = 0; //解锁
                roundabout_entry_flag = 0;//退出环岛状态
				cnt1 = 0;
            }
            break;
        }
		case 10: {
             if (is_border_straight(left_border) && is_border_straight(right_border) && p_odometer.distance>25 )//出环岛入正常状态，右边界直线且左边界直线 
            {
                lock_cross = 0; //解锁
                roundabout_entry_flag = 0;//退出环岛状态
				cnt2 = 0;
            }
            break;
        }
    }
 }
 
//======================环岛处理函数=====================//
// void handle_roundabout_process(){
//	static int cnt = 0;
//    switch (roundabout_entry_flag) {
//    case 1:
//        draw_thick_line(0, 119,52,15);//画左边线
//        break;
//	case 2:
//        draw_thick_line(179, 119,128,15);//画右边线
//        break;
//    case 3:
//        draw_thick_line(179, 119,100,15);//补左弯道
//        break;
//	case 4:
//        draw_thick_line(0, 119,80,15);//补右弯道
//        break;
//    case 5:
//    //弯道内循迹不做处理
//        break;
//	case 6:
//    //弯道内循迹不做处理
//        break;
//    case 7:
//        draw_thick_line(179, 119,100,15);//出弯道，补左弯道
//        break;
//    case 8:
//        draw_thick_line(0, 119,70,15);//补右弯道
//        break;
//    case 9:
//        draw_thick_line(0, 119,70,15);//画左边线
//        break;
//	case 10:
//        draw_thick_line(179, 119,130,15);//画右边
//        break;
//    }
//}

void handle_roundabout_process(){
	static int cnt = 0;
    switch (roundabout_entry_flag) {
    case 1:

        draw_thick_line(0, 119,52,15);//画左边线
        break;
	case 2:

        draw_thick_line(179, 119,128,15);//画右边线
        break;
    case 3:
        draw_thick_line(179, 119,100,15);//补左弯道
        break;
	case 4:
        draw_thick_line(0, 119,80,15);//补右弯道
        break;
    case 5:
    //弯道内循迹不做处理
        break;
	case 6:
    //弯道内循迹不做处理
        break;
    case 7:
        draw_thick_line(179, 119,110,15);//出弯道，补左弯道
        break;
    case 8:
        draw_thick_line(0, 119,80,15);//补右弯道
        break;
    case 9:
        draw_thick_line(0, 119,70,15);//画左边线
        break;
	case 10:
        draw_thick_line(179, 119,110,15);//画右边
        break;
    }
}

void handle_roundabout_process_new(){
	static int cnt = 0;
    switch (roundabout_entry_flag) {
    case 1:

        draw_thick_line(0, 119,52,15);//画左边线
        break;
	case 2:

        draw_thick_line(179, 119,128,15);//画右边线
        break;
    case 3:
        draw_thick_line(179, 119,85,15);//补左弯道
        break;
	case 4:
        draw_thick_line(0, 119,95,15);//补右弯道
        break;
    case 5:
    //弯道内循迹不做处理
        break;
	case 6:
		
    //弯道内循迹不做处理
        break;
    case 7:
        draw_thick_line(179, 119,85,15);//出弯道，补左弯道
        break;
    case 8:
        draw_thick_line(0, 119,95,15);//补右弯道
        break;
    case 9:
        break;
	case 10:
        break;
    }
}
//====================== 显示函数 ======================//
// 显示边界函数（带抗锯齿）
void draw_enhanced_borders() {
    // 绘制左边界（绿色）
    for(int i=0; i<image_h; i++) {
        if(left_border[i].x != 0xFFFF) {
            // 横向扩展3像素增强可视性
            for(int dx=-1; dx<=1; dx++) {
                int x = left_border[i].x + dx;
                if(x>=0 && x<image_w) {
                    ips200_draw_point(x, i, BORDER_COLOR);
                }
            }
        }
	}
    
    // 绘制右边界（红色）
    for(int i=0; i<image_h; i++) {
        if(right_border[i].x != 0xFFFF) {
            for(int dx=-1; dx<=1; dx++) {
                int x = right_border[i].x + dx;
                if(x>=0 && x<image_w) {
                    ips200_draw_point(x, i, 0xF800);
                }
            }
        }
	}
}

// 显示中线函数（黄色，更醒目）
void draw_centerline() {
    for (int y = 0; y < image_h; y++) {
        if (center_line[y].valid) {
            // 纵向连续3像素增强可视性
            for (int dy = -1; dy <= 1; dy++) {
                int draw_x = center_line[y].x + dy;
                if (draw_x >= 0 && draw_x < image_h) {
                    //ips200_draw_point(draw_x, center_line[y].y, 0xFFE0); 
                    bin_image[y][draw_x] = 0; // 清除二值化图像
                }
            }
        }
    }
}

//====================== 标志位显示函数 ======================//
void display_status_flags() {
    // 显示位置定义（右上角，x=100, y=0 开始，具体根据屏幕尺寸调整）
    #define FLAGS_X 0  // 起始x坐标
    #define FLAGS_Y 120    // 起始y坐标
    #define ROW_SPACE 20 // 行间距
	left_loss = is_border_lost_left(left_border);
    right_loss = is_border_lost_right(right_border);
    // 显示十字路口标志（格式：CR:1/0）
    ips200_show_int(FLAGS_X, FLAGS_Y + 0 * ROW_SPACE, crossroad_flag, 1);
    
    // 显示环岛入环标志（格式：RE:1/0）
    ips200_show_int(FLAGS_X, FLAGS_Y + 1 * ROW_SPACE, roundabout_entry_flag, 1);
    ips200_show_int(FLAGS_X, FLAGS_Y + 2 * ROW_SPACE, track_error, 3);
	ips200_show_float(FLAGS_X, FLAGS_Y + 3 * ROW_SPACE, left_qulve, 3,3);
	ips200_show_float(FLAGS_X, FLAGS_Y + 4 * ROW_SPACE, right_qulve, 3,3);
	ips200_show_int(FLAGS_X, FLAGS_Y + 5 * ROW_SPACE, left_loss, 1);
	ips200_show_int(FLAGS_X, FLAGS_Y + 6 * ROW_SPACE, right_loss, 1);
    ips200_show_int(FLAGS_X, FLAGS_Y + 7 * ROW_SPACE, destination_flag, 1);
}



//====================== 误差计算函数 ======================//
void calculate_track_error() {
    const int control_point_num = 5 ; // 控制点数量
    int valid_points = 0;           // 有效点计数
    int sum_diff = 0;               // 差值累加和
    const int center_x = image_w / 2; // 图像中心x坐标

    // 确定主控制点位置（图像下1/3处）
    const int y_center = image_h * 3 / 4 - 10 ; // 从顶部开始计算的下1/3位置
    
    // 生成五个控制点的y坐标（上下各两个点）
    int y_coords[5] = {
        y_center - 2,  // 主控点上方第2行
        y_center - 1,  // 主控点上方第1行
        y_center,       // 主控点
        y_center + 1,  // 主控点下方第1行
        y_center + 2   // 主控点下方第2行
    };

    // 遍历所有控制点
    for (int i = 0; i < control_point_num; i++) {
        int y = y_coords[i];
        
        // 检查y坐标是否有效
        if (y < 0 || y >= image_h) continue;      
        // 计算与中心x的差值并累加
        sum_diff += (center_line[y].x - center_x);
        valid_points++;
    }

    // 计算平均误差（至少需要1个有效点）
    if (valid_points > 0 ) {
        track_error = sum_diff / valid_points;
    } else {
        track_error = 0; // 无有效点时误差归零
    }
}


//终点检测 function
int final = 0;
void check_end()
{
	static int end_flag = 0; 
	switch (end_flag){
		case 0:
			if(destination_flag){p_Odometer_Init();end_flag = 1;}
			break;
		case 1:
			if(p_odometer.distance>50)end_flag = 2;
			track_error = 0;
			break;
		case 2:
			if(destination_flag){p_Odometer_Init();end_flag = 3;}
			break;
		case 3:
			if(p_odometer.distance>50)end_flag = 4;
			track_error = 0;
			break;
		case 4:
			final = 1;
			break;	
	}
}
//// 使用宏定义阈值和队列大小，便于修改
//#define ABS_THRESHOLD 45  // 绝对值判断阈值
//#define QUEUE_SIZE 10     // 队列大小

//int buffer_data(int data) {
//    // 静态变量，在多次调用间保持状态
//    static int queue[QUEUE_SIZE]; // 队列数组
//    static int start = 0;         // 队头索引
//    static int count = 0;         // 当前队列中的元素数量
//    static int initialized = 0;   // 初始化标志

//    // 首次调用时初始化队列
//    if (!initialized) {
//        for (int i = 0; i < QUEUE_SIZE; i++) {
//            queue[i] = 0;
//        }
//        initialized = 1;
//    }

//    if (count < QUEUE_SIZE) {
//        // 队列未满，添加新数据到队尾
//        int index = (start + count) % QUEUE_SIZE;
//        queue[index] = data;
//        count++;
//        
//        // 队列刚满时检查所有元素
//        if (count == QUEUE_SIZE) {
//            for (int i = 0; i < QUEUE_SIZE; i++) {
//                int idx = (start + i) % QUEUE_SIZE;
//                if (abs(queue[idx]) <= ABS_THRESHOLD) {
//                    return 0; // 有元素不满足条件
//                }
//            }
//            return 1; // 所有元素绝对值都大于阈值
//        }
//        return 0; // 队列未满，返回0
//    } else {
//        // 队列已满，覆盖最旧的数据
//        queue[start] = data;
//        start = (start + 1) % QUEUE_SIZE; // 循环队列
//        
//        // 检查队列所有元素
//        for (int i = 0; i < QUEUE_SIZE; i++) {
//            int idx = (start + i) % QUEUE_SIZE;
//            if (abs(queue[idx]) <= ABS_THRESHOLD) {
//                return 0; // 有元素不满足条件
//            }
//        }
//        return 1; // 所有元素都满足条件
//    }
//}


extern int moveflag;

// 主处理函数
void ImageProcess() {
	if (mt9v03x_finish_flag)
	{
		mt9v03x_finish_flag = 0;
			/*------------------计算最佳阈值并二值化------------------*/ 
		 uint8 threshold = otsu_threshold();//otsu_threshold_new()
		 
		 image_binarization(threshold);
        //quad_otsu_binarization();
		//threshold();

		/*----------------------图形学处理------------------------*/ 
		erosion(bin_image, morph_image);
		dilation(morph_image, bin_image);
		before_process();//预处理防止搜线出界

		/*-----------------------初始搜线-------------------------*/
		find_borders_and_centerline(); 
		/*-----------------------终点检测-------------------------*/
		detectDestination(40, 70);
		/*-----------------------十字路口处理-------------------------*/
		if( lock_cross == 0)
					{
							handle_crossroad2();
							handle_crossroad_process();
					}
		/*-----------------------环岛处理-------------------------*/
//		//plan1
		handle_roundabout_change();//环岛状态机
		handle_roundabout_process(); //环岛补线操作
		//plan2
//		handle_roundabout_change_new();//环岛状态机
//		handle_roundabout_process_new(); //环岛补线操作
		//环岛处理后二次寻线
		if(roundabout_entry_flag != 0 || crossroad_flag != 0){find_borders_and_centerline();}

		/*-----------------------计算误差-------------------------*/
		if(destination_flag)roundabout_entry_flag = 0;
		calculate_track_error();
//		if(buffer_data(track_error))moveflag = 2;
		check_end();
		/*-----------------------显示函数-------------------------*/   
		//    draw_enhanced_borders();       // 显示边界
		//    draw_centerline();       // 显示中线(黑色)
		//	display_status_flags(); // 显示状态标志
		//	ips200_displayimage03x((uint8 *)bin_image, image_w, image_h);
		/*-----------------------补线操作-------------------------*/
		//draw_thick_line(0, 119,52,15);//画左边线
		//draw_thick_line(179, 119,128,15);//画右边线
		//draw_thick_line(179, 119,52,15);//补左弯道
		//draw_thick_line(0, 119,128,15);//补右弯道
	}

}