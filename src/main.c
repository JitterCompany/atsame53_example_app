#include <atmel_start.h>

int main(void)
{
	init_mcu();
	/* Initializes MCU, drivers and middleware */
	// atmel_start_init();


	// LED on SAM E54 Xplained Pro board is PC18
	gpio_set_pin_direction(PC18, GPIO_DIRECTION_OUT);

	gpio_set_pin_level(PC18, false);

	/* Replace with your application code */
	while (1) {


		gpio_set_pin_level(PC18, false);

		for (int i=0;i<100000;i++) {}

		gpio_set_pin_level(PC18, true);

		for (int i=0;i<100000;i++) {}
	}
}
