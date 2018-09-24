#include "key.h"

void key_init() {
	LL_GPIO_InitTypeDef gpio_init_struct;
	// 使能gpioc时钟
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

	// 初始化结构体
	gpio_init_struct.Pin = LL_GPIO_PIN_0;
	gpio_init_struct.Mode = LL_GPIO_MODE_INPUT;
	gpio_init_struct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
	gpio_init_struct.Pull = LL_GPIO_PULL_UP;

	// 初始化io口
	LL_GPIO_Init(GPIOA, &gpio_init_struct);
}

uint8_t key_scan() {
	if (KEY0 == KEY_ON) {
		LL_mDelay(50);
		if (KEY0 == KEY_ON) {
			while (KEY0 == KEY_ON)
				return KEY_ON;
		}
	}
	return KEY_OFF;
}
