#include "uart.h"
uint8 uart_get_data[64];                            // ���ڽ������ݻ�����
uint8 fifo_get_data[64];                            // fifo �������������                   
uint32 fifo_data_count = 0;                         // fifo ���ݸ���
fifo_struct uart_data_fifo;
int update_data = 0;
od_result_t od_result[10];

void uart4_init()
{
	fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // ��ʼ�� fifo ���ػ�����
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // ��ʼ������
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);                                   // ���� UART_INDEX �Ľ����ж�
    interrupt_set_priority(UART_PRIORITY, 0);                                   // ���ö�Ӧ UART_INDEX ���ж����ȼ�Ϊ 0
}

void uart_rx_interrupt_handler ()
{ 
    uint8 get_data = 0;                                                             // �������ݱ���
    uint32 temp_length = 0;
    uint8 od_num = 0;
    uart_query_byte(UART_INDEX, &get_data);  
    {
        fifo_write_buffer(&uart_data_fifo, &get_data, 1);   
    }
    
    if(0xFF == get_data)
    {
        // ��ȡ��1�����ݣ������ж�֡ͷ��ʹ�������������
        temp_length = 1;
        fifo_read_buffer(&uart_data_fifo, fifo_get_data, &temp_length, FIFO_READ_AND_CLEAN);
        if(0xAA == fifo_get_data[0])
        {
            // ��ȡ��1�����ݣ����ڻ�ȡĿ����ţ�ʹ�������������
            temp_length = 1;
            fifo_read_buffer(&uart_data_fifo, fifo_get_data, &temp_length, FIFO_READ_AND_CLEAN);
            od_num = fifo_get_data[0];
            // ��ȡ8�����ݣ����ڻ�ȡĿ�����ݣ�Ȼ��ת�Ƶ��ṹ��������
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