#include "main.h"

extern volatile uint8_t LEDbuffer[LED_BUFFER_SIZE];

/*************************************************************
 * 函 数 名 : WS2812_update
 * 函数描述 : DMA发送数据
 * 输入参数 : none
 * 输出参数 : none
 * 返 回 值 : none
 *************************************************************/

void WS2812_update(uint8_t group) {

	LL_TIM_EnableCounter(TIM4);

	if (group == GROUP_A)									// GROUP_A只有1盏LED灯
	{

		LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, LED_BUFFER_SIZE);

		LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	} else {

		LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, LED_BUFFER_SIZE);

		LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);

	}

	LL_TIM_EnableDMAReq_UPDATE(TIM4);					// 使能TIM4 DMA请求
}

/*************************************************************
 * 函 数 名 : SetLedColor
 * 函数描述 : 设置某一盏LED颜色
 * 输入参数 : index  需要点亮的LED序号（第一盏、第二盏.....）
 * 输出参数 : none
 * 返 回 值 : none
 *************************************************************/

void SetLedColor(uint8_t index, uint8_t RED, uint8_t GREEN, uint8_t BLUE) {
	uint8_t tempBuffer[24] = { 0 };
	uint8_t i;

	for (i = 0; i < 8; i++) // GREEN data
		tempBuffer[i] = ((GREEN << i) & 0x80) ? WS2812_1 : WS2812_0;

	for (i = 0; i < 8; i++) // RED
		tempBuffer[8 + i] = ((RED << i) & 0x80) ? WS2812_1 : WS2812_0;

	for (i = 0; i < 8; i++) // BLUE
		tempBuffer[16 + i] = ((BLUE << i) & 0x80) ? WS2812_1 : WS2812_0;

	for (i = 0; i < 24; i++)
		LEDbuffer[RESET_SLOTS_BEGIN + index * 24 + i] = tempBuffer[i];

}

void SetWholeColor(uint8_t num, uint8_t red, uint8_t green, uint8_t blue) {

	for (uint32_t index = 0; index < num; index++)
		SetLedColor(index, red, green, blue);

}
