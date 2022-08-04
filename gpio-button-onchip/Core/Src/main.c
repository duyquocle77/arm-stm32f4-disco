#include "main.h"
#include <stdint.h>

#define LED_4 12
#define LED_3 13
#define LED_5 14
#define LED_6 15

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
typedef enum
{
	OFF,
	ON
} eLED_STATE;

void gpio_button_init();
uint8_t gpio_button_read();
void gpio_led_init();
void gpio_led_write(uint8_t LED_x, uint8_t value);

uint8_t flag = 0;

int
main(void) {
	HAL_Init();
	gpio_button_init();
	gpio_led_init();

	while (1) {
		 if (gpio_button_read() == 1) {
			 HAL_Delay(200);	// delay chong nhieu nut nhan
			 flag = 1 - flag;
		 }

		 if (flag == 1) {
			 gpio_led_write(LED_4, 1);
		 }
		 else {
			 gpio_led_write(LED_4, 0);
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
gpio_button_init()
{
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Input Level */
	uint32_t *GPIOA_MODER = (uint32_t *)(0x40020000);
	*GPIOA_MODER &= ~(0b11<<0);	// Set PA0 pin Input

	uint32_t *GPIOA_PUPDR = (uint32_t *)(0x4002000c);
	*GPIOA_PUPDR &= ~(0b11<<0);	// Set PA0 pin Floating
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
uint8_t
gpio_button_read()
{
	uint32_t *GPIOA_IDR = (uint32_t *)(0x40020010);
	if (((*GPIOA_IDR >> 1) & 1) == 1) {
		return 1;
	}
	else {
		return 0;
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
gpio_led_write(uint8_t LED_x, eLED_STATE state) {
	uint32_t *GPIOD_ODR = (uint32_t *)(0x40020c00 + 0x14);
	if (state == ON) {
		*GPIOD_ODR |= (1 << LED_x);
	}
	else {
		*GPIOD_ODR &= ~(1 << LED_x);
	}
}
