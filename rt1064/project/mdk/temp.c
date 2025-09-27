#include "uart.h"
uint8 uart_get_data[64] = {0};                                                        // 串口接收数据缓冲区
uint8 fifo_get_data[64] = {0};                                                        // fifo 输出读出缓冲区
uint8 get_data = 0;                                                             // 接收数据变量
uint32 fifo_data_count = 0;                                                     // fifo 数据个数
fifo_struct uart_data_fifo;
int box_x = 0;
int box_y = 0;

#define RX_BUFFER_SIZE 64
#define HEADER "a"
#define HEADER_LEN (sizeof(HEADER) - 1)
#define FOOTER "b"
#define FOOTER_LEN (sizeof(FOOTER) - 1)
// 全局变量
uint8_t rx_buffer[RX_BUFFER_SIZE];  // 接收缓冲区
uint8_t rx_index = 0;               // 当前写入位置
uint8_t header_found = 0;           // 头部匹配标志
uint8_t data_ready = 0;             // 数据就绪标志

int latest_x = 0;                   // 最新解析的x坐标
int latest_y = 0;                   // 最新解析的y坐标
uint8_t coords_valid = 0;           // 坐标有效性标志
	
void uart4_init()
{
	fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // 初始化 fifo 挂载缓冲区
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // 初始化编码器模块与引脚 正交解码编码器模式
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);                                   // 开启 UART_INDEX 的接收中断
    interrupt_set_priority(UART_PRIORITY, 0);                                   // 设置对应 UART_INDEX 的中断优先级为 0
}

// 解析函数：从缓冲区提取坐标
uint8_t parse_coordinates_from_buffer(void) {
    if (!data_ready) return 0;
    
    // 确保数据长度至少为 "a0,0b"（4字节）
    if (rx_index < 4) {
        data_ready = 0;
        return 0;
    }
    
    // 查找逗号位置（在头部和尾部之间）
    uint8_t comma_pos = 0;
    for (uint8_t i = HEADER_LEN; i < rx_index - FOOTER_LEN; i++) {
        if (rx_buffer[i] == ',') {
            comma_pos = i;
            break;
        }
    }
    
    if (comma_pos == 0) {  // 未找到逗号
        data_ready = 0;
        return 0;
    }
    
    // 临时存储解析结果
    int temp_x, temp_y;
    char *endptr;
    
    // 解析x坐标（跳过头部）
    temp_x = (int)strtol((const char*)&rx_buffer[HEADER_LEN], &endptr, 10);
    if (*endptr != ',') {
        data_ready = 0;
        return 0;
    }
    
    // 解析y坐标（从逗号后开始，到尾部前结束）
    temp_y = (int)strtol((const char*)&rx_buffer[comma_pos + 1], &endptr, 10);
    if (endptr - (const char*)&rx_buffer[0] != rx_index - FOOTER_LEN) {
        data_ready = 0;
        return 0;
    }
    
    // 解析成功，更新坐标
    latest_x = temp_x;
    latest_y = temp_y;
    coords_valid = 1;
    data_ready = 0;  // 清除就绪标志
    return 1;
}

void uart4_rx_interrupt_handler (void)
{
	//get_data = uart_read_byte(UART_INDEX);	
    if(uart_query_byte(UART_INDEX, &get_data))           // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
	{
		uint8_t byte = get_data;  
        // 缓冲区溢出处理
        if (rx_index >= RX_BUFFER_SIZE) {
            rx_index = 0;
            header_found = 0;
        }
        
        // 头部匹配逻辑
        if (!header_found) {
            if (byte == HEADER[0]) {
                header_found = 1;
                rx_index = 0;  // 开始存储数据
            }
            return;  // 未找到头部，丢弃数据
        }
        
        // 存储数据（已找到头部）
        rx_buffer[rx_index++] = byte;
        
        // 尾部匹配逻辑
        if (byte == FOOTER[0]) {
            // 确保数据长度至少为 "a0,0b"（4字节）
            if (rx_index >= 4) {
                data_ready = 1;  // 完整匹配到尾部，数据就绪
            }
            header_found = 0;  // 重置头部标志，准备下一包
        }
		if (data_ready) {
            if (parse_coordinates_from_buffer()) {
                // 使用解析后的坐标
				box_x = latest_x;
				box_y = latest_y;
				printf("success\n");
            } else {
                printf("fail\n");
            }}
	printf("%c ",get_data);
	printf("_");
	}
	
}

