#ifndef __UART_H
#define __UART_H
#include "zf_common_headfile.h"
#define UART_INDEX              (UART_4)                          // Ĭ�� UART_1
#define UART_BAUDRATE           (115200)                           // Ĭ�� 115200
#define UART_TX_PIN             (UART4_TX_C16)                           // Ĭ�� UART1_TX_A9
#define UART_RX_PIN             (UART4_RX_C17)                           // Ĭ�� UART1_RX_A10
#define UART_PRIORITY           (LPUART4_IRQn)                                  // ��Ӧ�����жϵ��жϱ�� �� MIMXRT1064.h ͷ�ļ��в鿴 IRQn_Type ö����
void uart4_init();
typedef struct
{
    uint16 res_x1;
    uint16 res_y1;
    uint16 res_x2;
    uint16 res_y2;
}od_result_t;
#endif