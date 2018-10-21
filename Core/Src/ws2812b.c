#include "main.h"
#include "cmsis_os.h"

volatile uint8_t LEDbuffer[LED_BUFFER_SIZE];

void SetPixelColor(uint16_t n, uint32_t c) {

	uint8_t r = (uint8_t) (c >> 16);
	uint8_t g = (uint8_t) (c >> 8);
	uint8_t b = (uint8_t) c;

	uint8_t tempBuffer[24] = { 0 };
	uint8_t i;

	for (i = 0; i < 8; i++) // GREEN
		tempBuffer[i] = ((g << i) & 0x80) ? WS2812_1 : WS2812_0;

	for (i = 0; i < 8; i++) // RED
		tempBuffer[8 + i] = ((r << i) & 0x80) ? WS2812_1 : WS2812_0;

	for (i = 0; i < 8; i++) // BLUE
		tempBuffer[16 + i] = ((b << i) & 0x80) ? WS2812_1 : WS2812_0;

	for (i = 0; i < 24; i++)
		LEDbuffer[RESET_SLOTS_BEGIN + n * 24 + i] = tempBuffer[i];

}

void PixelUpdate() {
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, LED_BUFFER_SIZE);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
	uint16_t i = 0;
	for (i = 0; i < LED_NUMBER; i++) {
		SetPixelColor(i, c);
		PixelUpdate();
		osDelay(wait);
	}
}

uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
	return ((uint32_t) r << 16) | ((uint32_t) g << 8) | b;
}

uint32_t Wheel(uint8_t WheelPos) {
	WheelPos = 255 - WheelPos;
	if (WheelPos < 85) {
		return Color(255 - WheelPos * 3, 0, WheelPos * 3);
	}
	if (WheelPos < 170) {
		WheelPos -= 85;
		return Color(0, WheelPos * 3, 255 - WheelPos * 3);
	}
	WheelPos -= 170;
	return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

//彩虹
void rainbow(uint8_t wait) {
	uint16_t i, j;

	for (j = 0; j < 256; j++) {
		for (i = 0; i < LED_NUMBER; i++) {
			SetPixelColor(i, Wheel((i + j) & 255));
			PixelUpdate();
			osDelay(wait);
		}
	}
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
	uint16_t i, j;

	for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
		for (i = 0; i < LED_NUMBER; i++) {
			SetPixelColor(i, Wheel(((i * 256 / LED_NUMBER) + j) & 255));
		}
		PixelUpdate();
		osDelay(wait);
	}
}
//Theatre-style crawling lights.呼吸灯
void theaterChase(uint32_t c, uint8_t wait) {
	for (int j = 0; j < 10; j++) { //do 10 cycles of chasing
		for (int q = 0; q < 3; q++) {
			for (uint16_t i = 0; i < LED_NUMBER; i = i + 3) {
				SetPixelColor(i + q, c); //turn every third pixel on
			}
			PixelUpdate();
			osDelay(wait);

			for (uint16_t i = 0; i < LED_NUMBER; i = i + 3) {
				SetPixelColor(i + q, 0); //turn every third pixel off
			}
			PixelUpdate();
			osDelay(wait);
		}
	}
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
	for (int j = 0; j < 256; j++) { // cycle all 256 colors in the wheel
		for (int q = 0; q < 3; q++) {
			for (uint16_t i = 0; i < LED_NUMBER; i = i + 3) {
				SetPixelColor(i + q, Wheel((i + j) % 255)); //turn every third pixel on
				PixelUpdate();
				osDelay(wait);
			}

			for (uint16_t i = 0; i < LED_NUMBER; i = i + 3) {
				SetPixelColor(i + q, 0); //turn every third pixel off
				PixelUpdate();
				osDelay(wait);
			}
		}
	}
}
