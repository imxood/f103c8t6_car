#include "led.h"

void led_init()
{
	LL_GPIO_InitTypeDef gpio_init_struct;

	// 使能gpioc时钟
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);

	// 初始化结构体
	gpio_init_struct.Pin = LL_GPIO_PIN_13;
	// 输出模式
	gpio_init_struct.Mode = LL_GPIO_MODE_OUTPUT;
	gpio_init_struct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	// 推挽输出
	gpio_init_struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

	// 初始化io口
	LL_GPIO_Init(GPIOC, &gpio_init_struct);

//	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
	// 初始状态, 设置LED为不亮
	LED0 = LED_OFF;

}
