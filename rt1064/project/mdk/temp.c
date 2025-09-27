#include "uart.h"
uint8 uart_get_data[64] = {0};                                                        // ���ڽ������ݻ�����
uint8 fifo_get_data[64] = {0};                                                        // fifo �������������
uint8 get_data = 0;                                                             // �������ݱ���
uint32 fifo_data_count = 0;                                                     // fifo ���ݸ���
fifo_struct uart_data_fifo;
int box_x = 0;
int box_y = 0;

#define RX_BUFFER_SIZE 64
#define HEADER "a"
#define HEADER_LEN (sizeof(HEADER) - 1)
#define FOOTER "b"
#define FOOTER_LEN (sizeof(FOOTER) - 1)
// ȫ�ֱ���
uint8_t rx_buffer[RX_BUFFER_SIZE];  // ���ջ�����
uint8_t rx_index = 0;               // ��ǰд��λ��
uint8_t header_found = 0;           // ͷ��ƥ���־
uint8_t data_ready = 0;             // ���ݾ�����־

int latest_x = 0;                   // ���½�����x����
int latest_y = 0;                   // ���½�����y����
uint8_t coords_valid = 0;           // ������Ч�Ա�־
	
void uart4_init()
{
	fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // ��ʼ�� fifo ���ػ�����
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // ��ʼ��������ģ�������� �������������ģʽ
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);                                   // ���� UART_INDEX �Ľ����ж�
    interrupt_set_priority(UART_PRIORITY, 0);                                   // ���ö�Ӧ UART_INDEX ���ж����ȼ�Ϊ 0
}

// �����������ӻ�������ȡ����
uint8_t parse_coordinates_from_buffer(void) {
    if (!data_ready) return 0;
    
    // ȷ�����ݳ�������Ϊ "a0,0b"��4�ֽڣ�
    if (rx_index < 4) {
        data_ready = 0;
        return 0;
    }
    
    // ���Ҷ���λ�ã���ͷ����β��֮�䣩
    uint8_t comma_pos = 0;
    for (uint8_t i = HEADER_LEN; i < rx_index - FOOTER_LEN; i++) {
        if (rx_buffer[i] == ',') {
            comma_pos = i;
            break;
        }
    }
    
    if (comma_pos == 0) {  // δ�ҵ�����
        data_ready = 0;
        return 0;
    }
    
    // ��ʱ�洢�������
    int temp_x, temp_y;
    char *endptr;
    
    // ����x���꣨����ͷ����
    temp_x = (int)strtol((const char*)&rx_buffer[HEADER_LEN], &endptr, 10);
    if (*endptr != ',') {
        data_ready = 0;
        return 0;
    }
    
    // ����y���꣨�Ӷ��ź�ʼ����β��ǰ������
    temp_y = (int)strtol((const char*)&rx_buffer[comma_pos + 1], &endptr, 10);
    if (endptr - (const char*)&rx_buffer[0] != rx_index - FOOTER_LEN) {
        data_ready = 0;
        return 0;
    }
    
    // �����ɹ�����������
    latest_x = temp_x;
    latest_y = temp_y;
    coords_valid = 1;
    data_ready = 0;  // ���������־
    return 1;
}

void uart4_rx_interrupt_handler (void)
{
	//get_data = uart_read_byte(UART_INDEX);	
    if(uart_query_byte(UART_INDEX, &get_data))           // �������� ��ѯʽ �����ݻ᷵�� TRUE û�����ݻ᷵�� FALSE
	{
		uint8_t byte = get_data;  
        // �������������
        if (rx_index >= RX_BUFFER_SIZE) {
            rx_index = 0;
            header_found = 0;
        }
        
        // ͷ��ƥ���߼�
        if (!header_found) {
            if (byte == HEADER[0]) {
                header_found = 1;
                rx_index = 0;  // ��ʼ�洢����
            }
            return;  // δ�ҵ�ͷ������������
        }
        
        // �洢���ݣ����ҵ�ͷ����
        rx_buffer[rx_index++] = byte;
        
        // β��ƥ���߼�
        if (byte == FOOTER[0]) {
            // ȷ�����ݳ�������Ϊ "a0,0b"��4�ֽڣ�
            if (rx_index >= 4) {
                data_ready = 1;  // ����ƥ�䵽β�������ݾ���
            }
            header_found = 0;  // ����ͷ����־��׼����һ��
        }
		if (data_ready) {
            if (parse_coordinates_from_buffer()) {
                // ʹ�ý����������
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

