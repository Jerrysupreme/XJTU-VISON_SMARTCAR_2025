#ifndef __UART_H
#define __UART_H
#include "zf_common_headfile.h"
#define UART_INDEX              (UART_4)                          // 默认 UART_1
#define UART_BAUDRATE           (115200)                           // 默认 115200
#define UART_TX_PIN             (UART4_TX_C16)                           // 默认 UART1_TX_A9
#define UART_RX_PIN             (UART4_RX_C17)                           // 默认 UART1_RX_A10
#define UART_PRIORITY           (LPUART4_IRQn)                                  // 对应串口中断的中断编号 在 MIMXRT1064.h 头文件中查看 IRQn_Type 枚举体
void uart4_init();
typedef struct
{
    uint16 res_x1;
    uint16 res_y1;
    uint16 res_x2;
    uint16 res_y2;
}od_result_t;
#endif