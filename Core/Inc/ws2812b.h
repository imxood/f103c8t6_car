#ifndef __WS2812B_H
#define	__WS2812B_H

// 有6盏灯
#define numLEDs 6

void Din_0(void);

void Din_1(void);

void Send_8bits(uint8_t dat);

//G--R--B
//MSB first
void Send_2811_24bits(uint8_t GData, uint8_t RData, uint8_t BData);

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait);

void SetPixelColor(uint16_t n, uint32_t c);

void PixelUpdate();

void rst();

uint32_t Color(uint8_t r, uint8_t g, uint8_t b);

uint32_t Wheel(uint8_t WheelPos);

//彩虹
void rainbow(uint8_t wait);

//Theatre-style crawling lights.呼吸灯
void theaterChase(uint32_t c, uint8_t wait);

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait);

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait);

#endif /* __WS2812B_H */
