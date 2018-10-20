#include "main.h"
#include "cmsis_os.h"

typedef uint8_t echoDeng_u8Type;

echoDeng_u8Type rBuffer[numLEDs] = {0};
echoDeng_u8Type gBuffer[numLEDs] = {0};
echoDeng_u8Type bBuffer[numLEDs] = {0};

void Din_0(void)
{
	WRITE_REG(TIM4->CCR2, 13);
}

void Din_1(void)
{
	WRITE_REG(TIM4->CCR2, 26);
}

void Send_8bits(uint8_t dat)
{
	uint8_t i;
	Din_0();
	for (i = 0; i < 8; i++)
	{
		if (dat & 0x80) //1,for "1",H:0.8us,L:0.45us;
		{
			Din_1();
		}
		else //0 ,for "0",H:0.4us,L:	0.85us
		{
			Din_0();
		}
		dat = dat << 1;
	}
}

//G--R--B
//MSB first
void Send_2811_24bits(uint8_t GData, uint8_t RData, uint8_t BData)
{
	Send_8bits(GData);
	Send_8bits(RData);
	Send_8bits(BData);
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait)
{
	uint16_t i = 0;
	for (i = 0; i < numLEDs; i++)
	{
		SetPixelColor(i, c);
		PixelUpdate();
		osDelay(wait);
	}
}

void SetPixelColor(uint16_t n, uint32_t c)
{
	echoDeng_u8Type i = 0;
	rBuffer[n] = (uint8_t)(c >> 16);
	gBuffer[n] = (uint8_t)(c >> 8);
	bBuffer[n] = (uint8_t)c;
	for (i = 0; i < numLEDs; i++)
	{
		Send_2811_24bits(rBuffer[i], gBuffer[i], bBuffer[i]);
	}
}

void PixelUpdate()
{
	rst();
}

void rst()
{
	WRITE_REG(TIM4->CCR2, 0);
	osDelay(1);
}

uint32_t Color(uint8_t r, uint8_t g, uint8_t b)
{
	return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

uint32_t Wheel(uint8_t WheelPos)
{
	WheelPos = 255 - WheelPos;
	if (WheelPos < 85)
	{
		return Color(255 - WheelPos * 3, 0, WheelPos * 3);
	}
	if (WheelPos < 170)
	{
		WheelPos -= 85;
		return Color(0, WheelPos * 3, 255 - WheelPos * 3);
	}
	WheelPos -= 170;
	return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

//彩虹
void rainbow(uint8_t wait)
{
	uint16_t i, j;

	for (j = 0; j < 256; j++)
	{
		for (i = 0; i < numLEDs; i++)
		{
			SetPixelColor(i, Wheel((i + j) & 255));
		}
		PixelUpdate();
		osDelay(wait);
	}
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait)
{
	uint16_t i, j;

	for (j = 0; j < 256 * 5; j++)
	{ // 5 cycles of all colors on wheel
		for (i = 0; i < numLEDs; i++)
		{
			SetPixelColor(i, Wheel(((i * 256 / numLEDs) + j) & 255));
		}
		PixelUpdate();
		osDelay(wait);
	}
}
//Theatre-style crawling lights.呼吸灯
void theaterChase(uint32_t c, uint8_t wait)
{
	for (int j = 0; j < 10; j++)
	{ //do 10 cycles of chasing
		for (int q = 0; q < 3; q++)
		{
			for (uint16_t i = 0; i < numLEDs; i = i + 3)
			{
				SetPixelColor(i + q, c); //turn every third pixel on
			}
			PixelUpdate();
			osDelay(wait);

			for (uint16_t i = 0; i < numLEDs; i = i + 3)
			{
				SetPixelColor(i + q, 0); //turn every third pixel off
			}
			PixelUpdate();
		}
	}
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait)
{
	for (int j = 0; j < 256; j++)
	{ // cycle all 256 colors in the wheel
		for (int q = 0; q < 3; q++)
		{
			for (uint16_t i = 0; i < numLEDs; i = i + 3)
			{
				SetPixelColor(i + q, Wheel((i + j) % 255)); //turn every third pixel on
			}
			PixelUpdate();

			osDelay(wait);

			for (uint16_t i = 0; i < numLEDs; i = i + 3)
			{
				SetPixelColor(i + q, 0); //turn every third pixel off
			}
		}
	}
}
