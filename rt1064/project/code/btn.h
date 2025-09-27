#ifndef __BTN_H
#define __BTN_H
#include "zf_common_headfile.h"
#define SWITCH1             (C27)
#define SWITCH2             (C26)
#define LED1                (B9)
#define BEEP                (B11)                              // ��Ӧ�����жϵ��жϱ�� �� MIMXRT1064.h ͷ�ļ��в鿴 IRQn_Type ö����
void btn_init();
void key_process();
#endif