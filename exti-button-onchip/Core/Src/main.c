#include "main.h"
#include <stdint.h>

#define LED_4 12
#define LED_3 13
#define LED_5 14
#define LED_6 15

void custom_delay(uint32_t time);
void gpio_led_init();
void gpio_led_write(uint8_t LED_x, uint8_t value);
void exti0_init();

uint8_t flag = 0;	/*this flag is status of on-board button*/
uint8_t cnt = 0;	/*this flag is to debug*/

int
main(void) {
	HAL_Init();
	gpio_led_init();
	exti0_init();

	while (1) {
		 if(flag == 1) {
			gpio_led_write(LED_6, 1);
		 }
		 else {
			gpio_led_write(LED_6, 0);
		 }
	}

	return 0;
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
custom_delay(uint32_t time) {
	for (uint32_t i = 0; i < time; i++) {
		__asm("NOP");
	}
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
gpio_led_init() {
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	uint32_t *GPIOD_MODER = (uint32_t *)(0x40020c00 + 0x00);
	*GPIOD_MODER &= ~(0b11111111<<24);
	*GPIOD_MODER |= (0b01010101<<24);
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
gpio_led_write(uint8_t LED_x, uint8_t state) {
	uint32_t *GPIOD_ODR = (uint32_t *)(0x40020c00 + 0x14);
	if (state == 1) {
	  *GPIOD_ODR |= (1 << LED_x);
	}
	else {
	  *GPIOD_ODR &= ~(1 << LED_x);
	}
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
exti0_init() {
	/*Configure External Interrupt pin   */
	uint32_t *EXTI_IMR = (uint32_t *)(0x40013c00 + 0x00); //enable mask interrupt
	*EXTI_IMR |= (1<<0);

	uint32_t *EXTI_RTSR = (uint32_t *)(0x40013c00 + 0x08); //rising-mode
	*EXTI_RTSR |= (1<<0);

	uint32_t *NVIC_ISER0 = (uint32_t *)(0xe000e100 + 0x00); //enable vector interrupt position 6
	*NVIC_ISER0 |= (1<<6);
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
EXTI0_IRQHandler() {
	custom_delay(500000);
	flag = 1 - flag;
	cnt++;

	/* clear interrupt flag */
	uint32_t *EXTI_PR = (uint32_t *)(0x40013c00 + 0x14);
	*EXTI_PR |= (1<<0);
}
