#include "uart.h"
uint8 uart_get_data[64];                            // 串口接收数据缓冲区
uint8 fifo_get_data[64];                            // fifo 输出读出缓冲区                   
uint32 fifo_data_count = 0;                         // fifo 数据个数
fifo_struct uart_data_fifo;
int update_data = 0;
od_result_t od_result[10];

void uart4_init()
{
	fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // 初始化 fifo 挂载缓冲区
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // 初始化串口
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);                                   // 开启 UART_INDEX 的接收中断
    interrupt_set_priority(UART_PRIORITY, 0);                                   // 设置对应 UART_INDEX 的中断优先级为 0
}

void uart_rx_interrupt_handler ()
{ 
    uint8 get_data = 0;                                                             // 接收数据变量
    uint32 temp_length = 0;
    uint8 od_num = 0;
    uart_query_byte(UART_INDEX, &get_data);  
    {
        fifo_write_buffer(&uart_data_fifo, &get_data, 1);   
    }
    
    if(0xFF == get_data)
    {
        // 读取第1个数据，用于判断帧头，使用完清除此数据
        temp_length = 1;
        fifo_read_buffer(&uart_data_fifo, fifo_get_data, &temp_length, FIFO_READ_AND_CLEAN);
        if(0xAA == fifo_get_data[0])
        {
            // 读取第1个数据，用于获取目标序号，使用完清除此数据
            temp_length = 1;
            fifo_read_buffer(&uart_data_fifo, fifo_get_data, &temp_length, FIFO_READ_AND_CLEAN);
            od_num = fifo_get_data[0];
            // 读取8个数据，用于获取目标数据，然后转移到结构体数组中
            temp_length = 8;
            fifo_read_buffer(&uart_data_fifo, fifo_get_data, &temp_length, FIFO_READ_AND_CLEAN);
			update_data = 0;
			//printf("\nx\n");
            memcpy((uint8*)(&od_result[od_num]), fifo_get_data, 8);
			update_data = 1;
        }
        fifo_clear(&uart_data_fifo);
    }
}