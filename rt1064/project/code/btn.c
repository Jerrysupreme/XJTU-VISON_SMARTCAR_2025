#include"btn.h"
extern int moveflag;
void btn_init()
{
    key_init(10);
    // gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP);
    // gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_PULL_UP);
    // gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    // gpio_init(BEEP, GPO, GPIO_LOW, GPO_PUSH_PULL); 
}
void key_process()
{
    key_scanner();

    // 检测 KEY_1
    if (KEY_SHORT_PRESS == key_get_state(KEY_1)) // KEY_1 短按
    {
        // 对 KEY_1 短按执行操作
		moveflag = 4;
    }
    else if (KEY_LONG_PRESS == key_get_state(KEY_1)) // KEY_1 长按
    {
        // 对 KEY_1 长按执行操作
    }

    // 检测 KEY_2
    if (KEY_SHORT_PRESS == key_get_state(KEY_2)) // KEY_2 短按
    {
		// 对 KEY_2 短按执行操作
		moveflag = 2;

    }
    else if (KEY_LONG_PRESS == key_get_state(KEY_2)) // KEY_2 长按
    {
        // 对 KEY_2 长按执行操作
    }

    // 检测 KEY_3
    if (KEY_SHORT_PRESS == key_get_state(KEY_3)) // KEY_3 短按
    {	
        // 对 KEY_3 短按执行操作
		moveflag = 3;
    }
    else if (KEY_LONG_PRESS == key_get_state(KEY_3)) // KEY_3 长按
    {
        // 对 KEY_3 长按执行操作
    }

    // 检测 KEY_4
    if (KEY_SHORT_PRESS == key_get_state(KEY_4)) // KEY_4 短按
    {

        // 对 KEY_4 短按执行操作
		moveflag = 1;
    }
    else if (KEY_LONG_PRESS == key_get_state(KEY_4)) // KEY_4 长按
    {
        // 对 KEY_4 长按执行操作
    }

    // 统一清除所有按键状态
    key_clear_all_state();
}