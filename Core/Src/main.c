
#include "sys.h"
#include "led.h"
#include "key.h"

int main(void) {
	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	LL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	key_init();

	led_init();

	while (1) {
		if(key_scan() == KEY_ON){
			LED0 = LED_ON;
		}else
			LED0 = LED_OFF;
	}

}
