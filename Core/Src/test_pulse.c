#include "test_pulse.h"
#include "main.h"

volatile uint8_t pulseBuf[PluseLen];

// 脉冲绘制
void pulse_calc(){
	for(int i = 0; i< PluseLen; i++){
		pulseBuf[i] = i % 90;
	}
}

// 脉冲更新
void pulse_update(){
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, PluseLen);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}
