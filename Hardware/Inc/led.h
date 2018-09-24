#pragma once

#include "sys.h"

#define LED0 PCout(13)

#define LED_ON	0
#define LED_OFF	1

/**
 * LED初始化, io口为C13
 */
void led_init();
