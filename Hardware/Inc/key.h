#pragma once

#include "sys.h"

#define KEY0 PAin(0)

#define KEY_ON	0
#define KEY_OFF	1

// 按键初始化, io口为A0
void key_init();

// 按键扫描
uint8_t key_scan();
