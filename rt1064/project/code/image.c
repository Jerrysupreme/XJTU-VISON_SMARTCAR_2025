//-------------------------------------------------------------------------------------------------------------------
//  ���ܣ�Otsu�㷨ʵ�� + ��ֵ��ͼ����ʾ
//------------------------------------------------------------------------------------------------------------------
#include "image.h"
#include "zf_common_headfile.h"
#include "motor.h"
#include <math.h>
#include <limits.h>  // ????????INT_MIN
// ȫ�ֱ�������
uint8 bin_image[image_h][image_w];      // ��ֵ��ͼ��
uint8 morph_image[image_h][image_w]; // ��̬ѧ�����м�ͼ��
BorderPoint left_border[image_h] ;  // ��߽�㼯��
BorderPoint right_border[image_h]; // �ұ߽�㼯��
CenterLine center_line[image_h] ; // �洢��������
uint8 crossroad_flag = 0;          // ʮ��·�ڱ�־λ
int track_error = 0;  // �洢����õ������
float left_qulve = 0; // ��������
float right_qulve = 0; // ��������
int left_loss = 0;
int right_loss = 0;
int lock_cross = 0; //������
uint8 roundabout_entry_flag = 0;    // ����״̬��־
static uint8 right_lost_counter = 0; // �ұ߽綪ʧ������
#define RIGHT_LOST_THRESHOLD   5    // �ұ߽�����������ֵ��֡����
extern OdometerTypeDef g_odometer,p_odometer,z_odometer;

 // ��򷨼��������ֵ
 uint8 otsu_threshold() {
     int histogram[256] = {0};
     float sum = 0.0, sumB = 0.0;
     int total = image_h * image_w;

     // ����ֱ��ͼ
     for (int y = 0; y < image_h; y++) {
         for (int x = 0; x < image_w; x++) {
             histogram[mt9v03x_image[y][x]]++;
         }
     }

     // �����ܻҶ�ֵ
     for (int i = 0; i < 256; i++) {
         sum += i * histogram[i];
     }

     float varMax = 0.0;
     uint8 best_th = 0;
     float wB = 0.0;  // ǰ��Ȩ��
     float wF = 0.0;  // ����Ȩ��

     for (int t = 0; t < 256; t++) {
         wB += histogram[t];          // ǰ���������ۼ�
         if (wB == 0) continue;

         wF = total - wB;             // ����������
         if (wF == 0) break;

         sumB += (float)(t * histogram[t]);

         float mB = sumB / wB;        // ǰ��ƽ���Ҷ�
         float mF = (sum - sumB) / wF;// ����ƽ���Ҷ�

         // ������䷽��
         float varBetween = wB * wF * (mB - mF) * (mB - mF);
        
         // ������󷽲����ֵ
         if (varBetween > varMax) {
             varMax = varBetween;
             best_th = t;
         }
     }
     return best_th;
 }
 
 // ƽ���Ҷȷ���ֵ������
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

 // ͼ���ֵ������
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
            
            // �����ṹԪ������
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

// ���ʹ����� 
void dilation(uint8 src[image_h][image_w], uint8 dst[image_h][image_w]) {
    for(int y = BORDER; y < image_h - BORDER; y++) {
        for(int x = BORDER; x < image_w - BORDER; x++) {
            uint8 max_val = 0;
            
            // �����ṹԪ������
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

//�߿�Ԥ������
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

// ���Ϻ�ı߽������߼��㺯��
void find_borders_and_centerline() {
    const int start_y = image_h - 5;               // ��ʼ�У��ײ�����5��
    const int end_y = image_h / 8;                 // �����У�ͼ���ϲ�1/4��
    static int prev_center_x = image_w / 2;        // ��һ�е�����λ�ã���ʼΪͼ�����ģ�
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
    // �����ӵײ�����5�е�ͼ��1/4����������
    for (int y = start_y; y >= end_y; y--) {
        //--- ��ʼ���߽������ ---
        center_line[y].x = prev_center_x; // ���߳�ʼ��Ϊ��һ��λ��
        center_line[y].valid = 0;         // Ĭ��������Ч
        
        //--- ��̬������㣺������һ������ ---
        int search_start = prev_center_x;
        search_start = (search_start < 0) ? 0 : 
                      (search_start >= image_w) ? image_w - 1 : search_start;

        //--- ������߽磺���������ɨ�� ---
        int left_found = 0;
        for (int x = search_start; x >= 0; x--) {
            if (bin_image[y][x] == 0) {
                left_border[y].x = x;
                left_found = 1;
                break;
            }
        }
        // δ�ҵ���߽�ʱ��Ϊ�����
        if (!left_found) left_border[y].x = 0;

        //--- �����ұ߽磺���������ɨ�� ---
        int right_found = 0;
        for (int x = search_start; x < image_w; x++) {
            if (bin_image[y][x] == 0) {
                right_border[y].x = x;
                right_found = 1;
                break;
            }
        }
        // δ�ҵ��ұ߽�ʱ��Ϊ���Ҳ�
        if (!right_found) right_border[y].x = image_w - 1;

        //--- �������� ---
        if (left_found && right_found) {
            // ���1��˫�߽���Ч������Ϊƽ��ֵ
            center_line[y].x = (left_border[y].x + right_border[y].x) / 2;
            center_line[y].valid = 1;
        } else {
            // ���2��������Ч�����߼̳���һ����Чֵ��������prev_center_x��
             center_line[y].x = image_w / 2;
            center_line[y].valid = 0;
        }

        //--- ������һ��������� ---
        if (center_line[y].valid) {
            prev_center_x = center_line[y].x; // ����Чʱ����
        }
    }
}

//====================== �����жϺ��� ======================//
int8 is_border_lost(BorderPoint border[image_h]) {
    int lost_count = 0;
    int total_rows = ELEMENT_START_Y - ELEMENT_END_Y + 1; // ������
    
    // ����Ԫ��ʶ�������������ϣ�
    for (int y = ELEMENT_START_Y; y >= ELEMENT_END_Y; y--) {
        if (border[y].x <= 8 || border[y].x >= image_w - 9 ) { // �߽���Ч3/6 7/8
            lost_count++;
        }
    }
    
    // �ж϶��߱����Ƿ񳬹�2/3���������룩
    int threshold = (total_rows + 2) / 3; // �ȼ���ceil(total_rows/4.0)
    return (lost_count >= 2*threshold) ? 1 : 0;
}

int8 is_border_lost2(BorderPoint border[image_h]) {
    int lost_count = 0;
    int total_rows = DUIXIAN2_START_Y - DUIXIAN2_END_Y + 1; // ������
    
    // ����Ԫ��ʶ�������������ϣ�
    for (int y = DUIXIAN2_START_Y; y >= DUIXIAN2_END_Y; y--) {
        if (border[y].x <= 5 || border[y].x >= image_w - 6 ) { // �߽���Ч
		//if (border[y].x <= 5 || border[y].x >= image_w - 6 ) { // �߽���Ч
            lost_count++;
        }
    }
    
    // �ж϶��߱����Ƿ񳬹�2/3���������룩
    int threshold = (total_rows + 2) / 3; // �ȼ���ceil(total_rows/4.0)
    return (lost_count >= 2*threshold) ? 1 : 0;
}

int8 is_border_lost3(BorderPoint border[image_h]) {
    int lost_count = 0;
    int total_rows = DUIXIAN3_START_Y - DUIXIAN3_END_Y + 1; // ������
    
    // ����Ԫ��ʶ�������������ϣ�
    for (int y = DUIXIAN3_START_Y; y >= DUIXIAN3_END_Y; y--) {
        //if (border[y].x <= 3 || border[y].x >= image_w - 5 ) { // �߽���Ч
		if (border[y].x <= 5 || border[y].x >= image_w - 6 ) { // �߽���Ч
            lost_count++;
        }
    }
    
    // �ж϶��߱����Ƿ񳬹�2/3���������룩
    int threshold = (total_rows + 2) / 3; // �ȼ���ceil(total_rows/4.0)
    return (lost_count >= 2*threshold) ? 1 : 0;
}

int8 is_border_lost_left(BorderPoint border[image_h]) {
    const int consecutive_points = 3;  // ����������
    const int max_diff = 15;           // ��������ֵ
    
    // ����Ԫ��ʶ�������������ϣ�ȷ�����㹻���Ϸ�������
    for (int y = DUIXIAN_START_Y; y >= DUIXIAN_END_Y + consecutive_points - 1; y--) {
        // ������Ч��׼��
        if (border[y].x == 0xFFFF) continue;

        int base_x = border[y].x; // ��׼��a��x����
        int valid = 1;
        
        // ����������4���㣨��5���㣺y��y-4��
        for (int i = 1; i < consecutive_points; i++) {
            int current_y = y - i;
            
            // ��Ч�Լ��
            if (current_y < DUIXIAN_END_Y) {
                valid = 0;
                break;
            }
            
            // ��ֵ���
            if ( base_x - border[current_y].x <= max_diff) {
                valid = 0;
                break;
            }
        }
        
        // ��������5���ֵ������ֵ���ж�����
        if (valid) {
            return 1;
        }
    }
    return 0;
}

int8 is_border_lost_right(BorderPoint border[image_h]) {
    const int consecutive_points = 3;  // ����������
    const int max_diff = 15;           // ��������ֵ
    
    // ����Ԫ��ʶ�������������ϣ�ȷ�����㹻���Ϸ�������
    for (int y = DUIXIAN_START_Y; y >= DUIXIAN_END_Y + consecutive_points - 1; y--) {
        // ������Ч��׼��
        if (border[y].x == 0xFFFF) continue;

        int base_x = border[y].x; // ��׼��a��x����
        int valid = 1;
        
        // ����������4���㣨��5���㣺y��y-4��
        for (int i = 1; i < consecutive_points; i++) {
            int current_y = y - i;
            
            // ��Ч�Լ��
            if (current_y < DUIXIAN_END_Y) {
                valid = 0;
                break;
            }
            
            // ��ֵ���
            if (border[current_y].x - base_x <= max_diff) {
                valid = 0;
                break;
            }
        }
        
        // ��������5���ֵ������ֵ���ж�����
        if (valid) {
            return 1;
        }
    }
    return 0;
}


//====================== �߽�ֱ���жϺ��� ======================//
int8 is_border_straight(BorderPoint border[image_h]) {
    const int num_points = 6;        // ȡ6����
    int valid_segments = 0;          // ��Ч��ֵ����
    int total_rows = ELEMENT_START_Y - ELEMENT_END_Y + 1;

    // ��������ʱֱ�ӷ��ط�ֱ��
    if (total_rows < 5) return 0;

    // ���������������ȷֲ�5�������
    int interval = (ELEMENT_START_Y - ELEMENT_END_Y) / 5;
    int y_coords[num_points];

    // ���ɲ�����y���꣨�������ϣ�
    for (int i = 0; i < num_points; i++) {
        y_coords[i] = ELEMENT_START_Y - i * interval;
        // �߽籣��
        if (y_coords[i] < ELEMENT_END_Y) y_coords[i] = ELEMENT_END_Y;
    }

    // ������в������Ƿ���Ч
     if (is_border_lost(border)) return 0;
    

    // �������ڵ��ֵ��ͳ��
    for (int i = 0; i < num_points - 1; i++) {
        int dx = abs(border[y_coords[i]].x - border[y_coords[i+1]].x);
        if (dx < 6) valid_segments++;
    }

    // �ж��Ƿ�����5��������4����Ч
    return (valid_segments >= 4) ? 1 : 0;
}

//====================== ���ʼ��㺯�� ======================//
float calculate_border_curvature_false(BorderPoint border[image_h]) {
    const int sample_points = 6;    // ����������
    int valid_samples = 0;          // ��Ч�����
    float total_curvature = 0.0f;   // �����ۼ�ֵ
    int total_rows = ELEMENT_START_Y - ELEMENT_END_Y + 1;

    // ��������ʱ����0���޷����㣩
    if (total_rows < sample_points) return 0.0f;

    // ���������������ȷֲ���
    int interval = (ELEMENT_START_Y - ELEMENT_END_Y) / (sample_points - 1);
    int y_coords[sample_points];

    // ���ɲ��������꣨�������ϣ�
    for (int i = 0; i < sample_points; i++) {
        y_coords[i] = ELEMENT_START_Y - i * interval;
        y_coords[i] = (y_coords[i] < ELEMENT_END_Y) ? ELEMENT_END_Y : y_coords[i];
    }

    // ���ǰ��������Ч�ԣ�������Ҫ3����������ʣ�
    for (int i = 0; i < 3; i++) {
        if (border[y_coords[i]].x == 0xFFFF) return 1000.0f; // ���ش�ֵ��ʾ��Ч
    }

    // ���㷨�������ʣ������м�㣩
    for (int i = 1; i < sample_points - 1; i++) {
        int y_prev = y_coords[i-1];
        int y_curr = y_coords[i];
        int y_next = y_coords[i+1];
        
        // ������Ч��
        if (border[y_prev].x == 0xFFFF || 
            border[y_curr].x == 0xFFFF || 
            border[y_next].x == 0xFFFF) continue;

        // ������ײ�ֽ������ʣ��򻯰棩
        int dx_prev = border[y_curr].x - border[y_prev].x;
        int dx_next = border[y_next].x - border[y_curr].x;
        float curvature = fabs(dx_next - dx_prev);

        total_curvature += curvature;
        valid_samples++;
    }

    // ����ƽ�����ʣ�������Ҫ2����Ч�Σ�
    if (valid_samples >= 2) {
        return total_curvature / valid_samples;
    }
    return 0.0f; // ���ݲ���ʱ��Ϊֱ��
}

float calculate_border_curvature(BorderPoint border[]) {
    // ����ʵ�����ݷ�Χȷ��ѭ���߽磨ʾ��ֵ���滻��
 
    // �߽���
    if (DUIXIAN_START_Y <= DUIXIAN_END_Y + 1) {
        return 0.0f; // ��Ч��Χ
    }
    
    float total_curvature = 0.0f;
    int valid_points = 0;
    
    // �ƶ�ƽ�����ڲ���
    const int window_size = 3; // �ƶ�ƽ�����ڴ�С
    float curvature_buffer[3] = {0}; // ���ʻ�����
    int buffer_index = 0;
    
    // ����ѭ��
    for (int i = DUIXIAN_START_Y - 1; i > DUIXIAN_END_Y + 1; i--) {
        // ��ȡ����������
        float x_prev = (float)border[i+1].x;
        float x_curr = (float)border[i].x;
        float x_next = (float)border[i-1].x;
        
        // ���㵼��
        float dx_di = (x_next - x_prev) / 2.0f;
        float d2x_di2 = x_next - 2*x_curr + x_prev;
        
        // ���ʼ���
        float denominator = powf(dx_di*dx_di + 1.0f, 1.5f);
        if (fabsf(denominator) > 1e-5f) {
            float curvature = fabsf(d2x_di2) / denominator;
            
            // ʹ���ƶ�ƽ��ƽ������
            curvature_buffer[buffer_index] = curvature;
            buffer_index = (buffer_index + 1) % window_size;
            
            // �����ƶ�ƽ��ֵ
            float smoothed_curvature = 0.0f;
            int count = 0;
            for (int j = 0; j < window_size; j++) {
                if (curvature_buffer[j] > 0) { // ��ʹ����Чֵ
                    smoothed_curvature += curvature_buffer[j];
                    count++;
                }
            }
            
            if (count > 0) {
                smoothed_curvature /= count;
                
                // ������ֵ���ˣ������쳣ͻ��
                if (smoothed_curvature < 0.5f) { // ��ֵ�ɸ���ʵ���������
                    total_curvature += smoothed_curvature;
                    valid_points++;
                }
            }
        }
    }
    
    return (valid_points > 0) ? (total_curvature / valid_points) : 0.0f;
}
/*-------------------���ߺ���-----------------------*/
/**
 * @brief �ڶ�ֵ��ͼ���ϻ���3���ؿ�ĺ��ߣ�����ݴ���
 * @param x0,y0 �������
 * @param x1,y1 �յ�����
 */
void draw_thick_line(uint8 x0, uint8 y0, uint8 x1, uint8 y1) {
    int dx = abs(x1 - x0);
    int dy = -abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int e2;

    // ��ѭ����Bresenham�㷨����
    while (1) {
        // ����3���ؿ�㣨���ĵ�+������չ��
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                int px = x0 + i;
                int py = y0 + j;
                // �߽���
                if (px >= 0 && px < image_w && py >= 0 && py < image_h) {
                    bin_image[py][px] = 0; // ���ú�ɫ
                }
            }
        }

        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        
        // ������һ����
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
/*********** ������������� ***********/
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


/*******�ж��Ƿ񵽴��յ�**************/
int destination_flag = 0;
/**
 * @brief ����Ƿ񵽴��յ�
 * @param start_point ��ʼ��
 * @param end_point ������
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
    // Ѱ����߽�ǵ�
    int left_corner_y = -1;
    for (int y = image_h *3/4; y >= image_h / 4; y--) {
        if (left_border[y].x != 0xFFFF) { // ȷ���߽����Ч
            int valid = 1;
            // �����������5����
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
    
    // Ѱ���ұ߽�ǵ�
    int right_corner_y = -1;
    for (int y = image_h - 1; y >= image_h / 4; y--) {
        if (right_border[y].x != 0xFFFF) { // ȷ���߽����Ч
            int valid = 1;
            // �����������5����
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
    
    // ���߲���
    if (left_corner_y != -1 && right_corner_y != -1) {
        // ����߽�ǵ㻭�ߵ�ͼ���²��е�
        draw_thick_line(left_border[left_corner_y -7].x, left_corner_y-5, 
                        image_w / 2 - 25, image_h - 1);
        draw_thick_line(right_border[right_corner_y -7].x, right_corner_y -5, 
                        image_w / 2 + 25, image_h - 1);
    }

}

void handle_crossroad2() {
    //------------------------ ״̬��ת ------------------------//

    switch (crossroad_flag) {
        // ״̬0���������
        case 0: {
            if (is_border_lost2(left_border) && is_border_lost2(right_border) && roundabout_entry_flag == 0) 
            {
                crossroad_flag = 1;
				p_Odometer_Init();
            }
            break;
        }
        // ״̬1����ʮ��·��
        case 1: {
            if (is_border_lost3(left_border) && is_border_lost3(right_border) && roundabout_entry_flag == 0) //��ʮ��·��
            {
				p_Odometer_Init();
                crossroad_flag = 2;
							
            }
            if (!is_border_lost2(left_border) || !is_border_lost2(right_border)) 
            {
                crossroad_flag = 0; //�����߽���ұ߽�ָ����ص�״̬0,��ֹ��ʶ��
            }
            break;
        }
		case 2: {
             if (p_odometer.distance>40) //��ʻ��ʮ��·��
            {
                crossroad_flag = 0;
            }
            break;
        }
       
    }
}

void handle_crossroad_process() {
    //------------------------ ״̬��ת ------------------------//
    switch (crossroad_flag) {
        // ״̬0���������
        case 0: {
            break;
        }
        // ״̬1����ʮ��·��
        case 1: {
            draw_thick_line(0, 119,52,15);//�������
			draw_thick_line(179, 119,128,15);//���ұ���
            break;       
        }
		case 2: {
            crossroad_repair();
            break;
        }
       
    }
}

void handle_roundabout_change() {
    //------------------------ ״̬��ת ------------------------//
    left_qulve = is_border_lost(left_border)>0?0:calculate_border_curvature(left_border);
    right_qulve = is_border_lost(right_border)>0?0:calculate_border_curvature(right_border);
    static int cnt1  = 0;
	static int cnt2  = 0;
    switch (roundabout_entry_flag) {
        // ״̬0���������
        case 0: {
//            if (is_border_lost2(left_border) && right_qulve>0.3 && is_border_straight2(right_border)&&!is_border_lost2(right_border))// �ұ߽�ֱ������߽綪ʧ
            if (is_border_lost_left(left_border) && !is_border_lost_right(right_border) && right_qulve<0.1 && !crossroad_flag)//���ߣ�����ֱ�ߣ�λ�û���ʼ��
            {
				cnt1 ++;
            }	
			if (cnt1 > 8){
			roundabout_entry_flag = 1;
            p_Odometer_Init();
			cnt1 = 0;
			}
//            if (is_border_lost2(left_border) && right_qulve>0.3 && is_border_straight2(right_border)&&!is_border_lost2(right_border))// �ұ߽�ֱ������߽綪ʧ
            if (is_border_lost_right(right_border) && !is_border_lost_left(left_border) && left_qulve<0.1 && !crossroad_flag)//���ߣ�����ֱ�ߣ�λ�û���ʼ��
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
        // ״̬1���뻷���
        case 1: {
			if (is_border_lost_left(left_border) && is_border_lost_right(right_border) && p_odometer.distance < 50)//����ʮ��·��
			{
				roundabout_entry_flag = 0;
			}
			if (is_border_lost(left_border) && is_border_lost(right_border) && p_odometer.distance < 50)//����ʮ��·��
			{
				roundabout_entry_flag = 0;
			}
//			if (is_border_lost_right(right_border))
//			 {
//				 roundabout_entry_flag = 0;
//			 }
            if (p_odometer.distance>DISTANCE_1) //��ʻ���������������������������
            {
                roundabout_entry_flag = 3;z_Odometer_Init();
            }   
			 z_Odometer_Init();
            break;
        }
		case 2: {
             if (p_odometer.distance>DISTANCE_1) //��ʻ���������������������������
            {
                roundabout_entry_flag = 4;z_Odometer_Init();
 
            }   
			if (is_border_lost_right(right_border) && is_border_lost_left(left_border) && p_odometer.distance < 40 )//����ʮ��·��
			{
				roundabout_entry_flag = 0;
			}
			if (is_border_lost(left_border) && is_border_lost(right_border) && p_odometer.distance < 40 )//����ʮ��·��
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
        // ״̬3���������ѭ��
        case 3: {
			lock_cross = 1; //��סʮ�ּ��
             if(z_odometer.distance>45)
             {roundabout_entry_flag = 5;}
            break;
        }
		case 4: {
			lock_cross = 1; //��סʮ�ּ��
             if(z_odometer.distance< -45)
             {
				 roundabout_entry_flag = 6;
			 }
            break;
        }
        // ״̬5�������ѭ��
        case 5: {
             if (is_border_lost(left_border) && is_border_lost(right_border)) {//���Ҷ��߻�����������������ħ���������������
				roundabout_entry_flag = 7;
				z_Odometer_Init();
            }
            break;
        }
		case 6: {
             if (is_border_lost(left_border) && is_border_lost(right_border)) {//���Ҷ��߻�����������������ħ���������������
				roundabout_entry_flag = 8;
				z_Odometer_Init();
			
            }
            break;
        }
        // ״̬7��������
        case 7: {
             if (!is_border_lost(right_border) && is_border_straight(right_border) && z_odometer.distance > 30) {//����ֱ��
			 //if (!is_border_lost(right_border) && is_border_straight(right_border)) {//����ֱ��
				roundabout_entry_flag = 9;
				p_Odometer_Init();
            }
            break;
        }
		case 8: {
            if (!is_border_lost(left_border) && is_border_straight(left_border) && z_odometer.distance< -30) {//����ֱ��
			//if (!is_border_lost(left_border) && is_border_straight(left_border)) {//����ֱ��
				roundabout_entry_flag = 10;
				p_Odometer_Init();
            }
            break;
        }
        case 9: {
             if (is_border_straight(left_border) && is_border_straight(right_border) && p_odometer.distance>DISTANCE_2 )//������������״̬���ұ߽�ֱ������߽�ֱ�� 
            {
                lock_cross = 0; //����
                roundabout_entry_flag = 0;//�˳�����״̬
				cnt1 = 0;
            }
            break;
        }
		case 10: {
             if (is_border_straight(left_border) && is_border_straight(right_border) && p_odometer.distance>DISTANCE_2 )//������������״̬���ұ߽�ֱ������߽�ֱ�� 
            {
                lock_cross = 0; //����
                roundabout_entry_flag = 0;//�˳�����״̬
				cnt2 = 0;
            }
            break;
        }
    }
 }

 void handle_roundabout_change_new() {
    //------------------------ ״̬��ת ------------------------//
    left_qulve = is_border_lost(left_border)>0?0:calculate_border_curvature(left_border);
    right_qulve = is_border_lost(right_border)>0?0:calculate_border_curvature(right_border);
    static int cnt1  = 0;
	static int cnt2  = 0;
    switch (roundabout_entry_flag) {
        // ״̬0���������
        case 0: {
//            if (is_border_lost2(left_border) && right_qulve>0.3 && is_border_straight2(right_border)&&!is_border_lost2(right_border))// �ұ߽�ֱ������߽綪ʧ
            if (is_border_lost_left(left_border) && !is_border_lost_right(right_border) && right_qulve<0.1 && !crossroad_flag)//���ߣ�����ֱ�ߣ�λ�û���ʼ��
            {
				cnt1 ++;
            }	
			if (cnt1 > 6){
			roundabout_entry_flag = 1;
            p_Odometer_Init();
			cnt1 = 0;
			}
//            if (is_border_lost2(left_border) && right_qulve>0.3 && is_border_straight2(right_border)&&!is_border_lost2(right_border))// �ұ߽�ֱ������߽綪ʧ
            if (is_border_lost_right(right_border) && !is_border_lost_left(left_border) && left_qulve<0.1 && !crossroad_flag)//���ߣ�����ֱ�ߣ�λ�û���ʼ��
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
        // ״̬1���뻷���
        case 1: {
			if (is_border_lost_left(left_border) && is_border_lost_right(right_border) && p_odometer.distance < 15)//����ʮ��·��
			{
				roundabout_entry_flag = 0;
			}
			if (is_border_lost(left_border) && is_border_lost(right_border) && p_odometer.distance < 15)//����ʮ��·��
			{
				roundabout_entry_flag = 0;
			}
//			if (is_border_lost_right(right_border))
//			 {
//				 roundabout_entry_flag = 0;
//			 }
            if (p_odometer.distance>DISTANCE_3 - 15) //��ʻ���������������������������,speed 1 is 10
            {
                roundabout_entry_flag = 3;z_Odometer_Init();
            }   
            break;
        }
		case 2: {
             if (p_odometer.distance>DISTANCE_3 -10) //��ʻ���������������������������
            {
                roundabout_entry_flag = 4;z_Odometer_Init();
 
            }   
			if (is_border_lost_right(right_border) && is_border_lost_left(left_border) && p_odometer.distance < 15 )//����ʮ��·��
			{
				roundabout_entry_flag = 0;
			}
			if (is_border_lost(left_border) && is_border_lost(right_border) && p_odometer.distance < 15 )//����ʮ��·��
			{
				roundabout_entry_flag = 0;
			}
//			if (is_border_lost_left(left_border))
//			 {
//				 roundabout_entry_flag = 0;
//			 }
            break;
        }
        // ״̬3���������ѭ��
        case 3: {
			lock_cross = 1; //��סʮ�ּ��
             if(z_odometer.distance>70)
             {roundabout_entry_flag = 5;}
            break;
        }
		case 4: {
			lock_cross = 1; //��סʮ�ּ��
             if(z_odometer.distance< -70)
             {
				 roundabout_entry_flag = 6;
			 }
            break;
        }
        // ״̬5�������ѭ��
        case 5: {
             if (is_border_lost(left_border) && is_border_lost(right_border)) {//���Ҷ��߻�����������������ħ���������������
				roundabout_entry_flag = 7;
				z_Odometer_Init();
            }
            break;
        }
		case 6: {
             if (is_border_lost(left_border) && is_border_lost(right_border)) {//���Ҷ��߻�����������������ħ���������������
				roundabout_entry_flag = 8;
				z_Odometer_Init();
			
            }
            break;
        }
        // ״̬7��������
        case 7: {
             if (!is_border_lost(right_border) && is_border_straight(right_border) && z_odometer.distance > 50) {//����ֱ��
			 //if (!is_border_lost(right_border) && is_border_straight(right_border)) {//����ֱ��
				roundabout_entry_flag = 9;
				p_Odometer_Init();
            }
            break;
        }
		case 8: {
            if (!is_border_lost(left_border) && is_border_straight(left_border) && z_odometer.distance< -50) {//����ֱ��
			//if (!is_border_lost(left_border) && is_border_straight(left_border)) {//����ֱ��
				roundabout_entry_flag = 10;
				p_Odometer_Init();
            }
            break;
        }
        case 9: {
             if (is_border_straight(left_border) && is_border_straight(right_border) && p_odometer.distance>25 )//������������״̬���ұ߽�ֱ������߽�ֱ�� 
            {
                lock_cross = 0; //����
                roundabout_entry_flag = 0;//�˳�����״̬
				cnt1 = 0;
            }
            break;
        }
		case 10: {
             if (is_border_straight(left_border) && is_border_straight(right_border) && p_odometer.distance>25 )//������������״̬���ұ߽�ֱ������߽�ֱ�� 
            {
                lock_cross = 0; //����
                roundabout_entry_flag = 0;//�˳�����״̬
				cnt2 = 0;
            }
            break;
        }
    }
 }
 
//======================����������=====================//
// void handle_roundabout_process(){
//	static int cnt = 0;
//    switch (roundabout_entry_flag) {
//    case 1:
//        draw_thick_line(0, 119,52,15);//�������
//        break;
//	case 2:
//        draw_thick_line(179, 119,128,15);//���ұ���
//        break;
//    case 3:
//        draw_thick_line(179, 119,100,15);//�������
//        break;
//	case 4:
//        draw_thick_line(0, 119,80,15);//�������
//        break;
//    case 5:
//    //�����ѭ����������
//        break;
//	case 6:
//    //�����ѭ����������
//        break;
//    case 7:
//        draw_thick_line(179, 119,100,15);//��������������
//        break;
//    case 8:
//        draw_thick_line(0, 119,70,15);//�������
//        break;
//    case 9:
//        draw_thick_line(0, 119,70,15);//�������
//        break;
//	case 10:
//        draw_thick_line(179, 119,130,15);//���ұ�
//        break;
//    }
//}

void handle_roundabout_process(){
	static int cnt = 0;
    switch (roundabout_entry_flag) {
    case 1:

        draw_thick_line(0, 119,52,15);//�������
        break;
	case 2:

        draw_thick_line(179, 119,128,15);//���ұ���
        break;
    case 3:
        draw_thick_line(179, 119,100,15);//�������
        break;
	case 4:
        draw_thick_line(0, 119,80,15);//�������
        break;
    case 5:
    //�����ѭ����������
        break;
	case 6:
    //�����ѭ����������
        break;
    case 7:
        draw_thick_line(179, 119,110,15);//��������������
        break;
    case 8:
        draw_thick_line(0, 119,80,15);//�������
        break;
    case 9:
        draw_thick_line(0, 119,70,15);//�������
        break;
	case 10:
        draw_thick_line(179, 119,110,15);//���ұ�
        break;
    }
}

void handle_roundabout_process_new(){
	static int cnt = 0;
    switch (roundabout_entry_flag) {
    case 1:

        draw_thick_line(0, 119,52,15);//�������
        break;
	case 2:

        draw_thick_line(179, 119,128,15);//���ұ���
        break;
    case 3:
        draw_thick_line(179, 119,85,15);//�������
        break;
	case 4:
        draw_thick_line(0, 119,95,15);//�������
        break;
    case 5:
    //�����ѭ����������
        break;
	case 6:
		
    //�����ѭ����������
        break;
    case 7:
        draw_thick_line(179, 119,85,15);//��������������
        break;
    case 8:
        draw_thick_line(0, 119,95,15);//�������
        break;
    case 9:
        break;
	case 10:
        break;
    }
}
//====================== ��ʾ���� ======================//
// ��ʾ�߽纯����������ݣ�
void draw_enhanced_borders() {
    // ������߽磨��ɫ��
    for(int i=0; i<image_h; i++) {
        if(left_border[i].x != 0xFFFF) {
            // ������չ3������ǿ������
            for(int dx=-1; dx<=1; dx++) {
                int x = left_border[i].x + dx;
                if(x>=0 && x<image_w) {
                    ips200_draw_point(x, i, BORDER_COLOR);
                }
            }
        }
	}
    
    // �����ұ߽磨��ɫ��
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

// ��ʾ���ߺ�������ɫ������Ŀ��
void draw_centerline() {
    for (int y = 0; y < image_h; y++) {
        if (center_line[y].valid) {
            // ��������3������ǿ������
            for (int dy = -1; dy <= 1; dy++) {
                int draw_x = center_line[y].x + dy;
                if (draw_x >= 0 && draw_x < image_h) {
                    //ips200_draw_point(draw_x, center_line[y].y, 0xFFE0); 
                    bin_image[y][draw_x] = 0; // �����ֵ��ͼ��
                }
            }
        }
    }
}

//====================== ��־λ��ʾ���� ======================//
void display_status_flags() {
    // ��ʾλ�ö��壨���Ͻǣ�x=100, y=0 ��ʼ�����������Ļ�ߴ������
    #define FLAGS_X 0  // ��ʼx����
    #define FLAGS_Y 120    // ��ʼy����
    #define ROW_SPACE 20 // �м��
	left_loss = is_border_lost_left(left_border);
    right_loss = is_border_lost_right(right_border);
    // ��ʾʮ��·�ڱ�־����ʽ��CR:1/0��
    ips200_show_int(FLAGS_X, FLAGS_Y + 0 * ROW_SPACE, crossroad_flag, 1);
    
    // ��ʾ�����뻷��־����ʽ��RE:1/0��
    ips200_show_int(FLAGS_X, FLAGS_Y + 1 * ROW_SPACE, roundabout_entry_flag, 1);
    ips200_show_int(FLAGS_X, FLAGS_Y + 2 * ROW_SPACE, track_error, 3);
	ips200_show_float(FLAGS_X, FLAGS_Y + 3 * ROW_SPACE, left_qulve, 3,3);
	ips200_show_float(FLAGS_X, FLAGS_Y + 4 * ROW_SPACE, right_qulve, 3,3);
	ips200_show_int(FLAGS_X, FLAGS_Y + 5 * ROW_SPACE, left_loss, 1);
	ips200_show_int(FLAGS_X, FLAGS_Y + 6 * ROW_SPACE, right_loss, 1);
    ips200_show_int(FLAGS_X, FLAGS_Y + 7 * ROW_SPACE, destination_flag, 1);
}



//====================== �����㺯�� ======================//
void calculate_track_error() {
    const int control_point_num = 5 ; // ���Ƶ�����
    int valid_points = 0;           // ��Ч�����
    int sum_diff = 0;               // ��ֵ�ۼӺ�
    const int center_x = image_w / 2; // ͼ������x����

    // ȷ�������Ƶ�λ�ã�ͼ����1/3����
    const int y_center = image_h * 3 / 4 - 10 ; // �Ӷ�����ʼ�������1/3λ��
    
    // ����������Ƶ��y���꣨���¸������㣩
    int y_coords[5] = {
        y_center - 2,  // ���ص��Ϸ���2��
        y_center - 1,  // ���ص��Ϸ���1��
        y_center,       // ���ص�
        y_center + 1,  // ���ص��·���1��
        y_center + 2   // ���ص��·���2��
    };

    // �������п��Ƶ�
    for (int i = 0; i < control_point_num; i++) {
        int y = y_coords[i];
        
        // ���y�����Ƿ���Ч
        if (y < 0 || y >= image_h) continue;      
        // ����������x�Ĳ�ֵ���ۼ�
        sum_diff += (center_line[y].x - center_x);
        valid_points++;
    }

    // ����ƽ����������Ҫ1����Ч�㣩
    if (valid_points > 0 ) {
        track_error = sum_diff / valid_points;
    } else {
        track_error = 0; // ����Ч��ʱ������
    }
}


//�յ��� function
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
//// ʹ�ú궨����ֵ�Ͷ��д�С�������޸�
//#define ABS_THRESHOLD 45  // ����ֵ�ж���ֵ
//#define QUEUE_SIZE 10     // ���д�С

//int buffer_data(int data) {
//    // ��̬�������ڶ�ε��ü䱣��״̬
//    static int queue[QUEUE_SIZE]; // ��������
//    static int start = 0;         // ��ͷ����
//    static int count = 0;         // ��ǰ�����е�Ԫ������
//    static int initialized = 0;   // ��ʼ����־

//    // �״ε���ʱ��ʼ������
//    if (!initialized) {
//        for (int i = 0; i < QUEUE_SIZE; i++) {
//            queue[i] = 0;
//        }
//        initialized = 1;
//    }

//    if (count < QUEUE_SIZE) {
//        // ����δ������������ݵ���β
//        int index = (start + count) % QUEUE_SIZE;
//        queue[index] = data;
//        count++;
//        
//        // ���и���ʱ�������Ԫ��
//        if (count == QUEUE_SIZE) {
//            for (int i = 0; i < QUEUE_SIZE; i++) {
//                int idx = (start + i) % QUEUE_SIZE;
//                if (abs(queue[idx]) <= ABS_THRESHOLD) {
//                    return 0; // ��Ԫ�ز���������
//                }
//            }
//            return 1; // ����Ԫ�ؾ���ֵ��������ֵ
//        }
//        return 0; // ����δ��������0
//    } else {
//        // ����������������ɵ�����
//        queue[start] = data;
//        start = (start + 1) % QUEUE_SIZE; // ѭ������
//        
//        // ����������Ԫ��
//        for (int i = 0; i < QUEUE_SIZE; i++) {
//            int idx = (start + i) % QUEUE_SIZE;
//            if (abs(queue[idx]) <= ABS_THRESHOLD) {
//                return 0; // ��Ԫ�ز���������
//            }
//        }
//        return 1; // ����Ԫ�ض���������
//    }
//}


extern int moveflag;

// ��������
void ImageProcess() {
	if (mt9v03x_finish_flag)
	{
		mt9v03x_finish_flag = 0;
			/*------------------���������ֵ����ֵ��------------------*/ 
		 uint8 threshold = otsu_threshold();//otsu_threshold_new()
		 
		 image_binarization(threshold);
        //quad_otsu_binarization();
		//threshold();

		/*----------------------ͼ��ѧ����------------------------*/ 
		erosion(bin_image, morph_image);
		dilation(morph_image, bin_image);
		before_process();//Ԥ�����ֹ���߳���

		/*-----------------------��ʼ����-------------------------*/
		find_borders_and_centerline(); 
		/*-----------------------�յ���-------------------------*/
		detectDestination(40, 70);
		/*-----------------------ʮ��·�ڴ���-------------------------*/
		if( lock_cross == 0)
					{
							handle_crossroad2();
							handle_crossroad_process();
					}
		/*-----------------------��������-------------------------*/
//		//plan1
		handle_roundabout_change();//����״̬��
		handle_roundabout_process(); //�������߲���
		//plan2
//		handle_roundabout_change_new();//����״̬��
//		handle_roundabout_process_new(); //�������߲���
		//������������Ѱ��
		if(roundabout_entry_flag != 0 || crossroad_flag != 0){find_borders_and_centerline();}

		/*-----------------------�������-------------------------*/
		if(destination_flag)roundabout_entry_flag = 0;
		calculate_track_error();
//		if(buffer_data(track_error))moveflag = 2;
		check_end();
		/*-----------------------��ʾ����-------------------------*/   
		//    draw_enhanced_borders();       // ��ʾ�߽�
		//    draw_centerline();       // ��ʾ����(��ɫ)
		//	display_status_flags(); // ��ʾ״̬��־
		//	ips200_displayimage03x((uint8 *)bin_image, image_w, image_h);
		/*-----------------------���߲���-------------------------*/
		//draw_thick_line(0, 119,52,15);//�������
		//draw_thick_line(179, 119,128,15);//���ұ���
		//draw_thick_line(179, 119,52,15);//�������
		//draw_thick_line(0, 119,128,15);//�������
	}

}