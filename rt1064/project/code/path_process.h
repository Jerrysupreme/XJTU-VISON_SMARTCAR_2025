/*
该文件用于车模赛道循迹控制

API：
****用户****
赛道循迹控制初始化
************

****底层****
路径线扫描
************
*/

#ifndef _PATH_PROCESS_H_
#define _PATH_PROCESS_H_

#include "zf_common_headfile.h"

/* 赛道循迹控制初始化 */
void path_process_init(void);

/* 路径线扫描 */
void path_search(void);
void get_err(void);
#endif