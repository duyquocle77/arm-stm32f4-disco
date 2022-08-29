#include "main.h"
#include <stdint.h>

#define LED_4 12
#define LED_3 13
#define LED_5 14
#define LED_6 15

void gpio_led_init();
void gpio_led_write(uint8_t LED_x, uint8_t value);

int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	gpio_led_init();

	while (1)
	{
		gpio_led_write(LED_6, 1);
		HAL_Delay(1000);
		gpio_led_write(LED_6, 0);
		HAL_Delay(1000);
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
gpio_led_init() {
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  uint32_t volatile *const GPIOD_MODER = (uint32_t *)(0x40020c00 + 0x00);
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
	uint32_t volatile *const GPIOD_ODR = (uint32_t *)(0x40020c00 + 0x14);
  if (state == 1) {
	  *GPIOD_ODR |= (1 << LED_x);
  }
  else {
	  *GPIOD_ODR &= ~(1 << LED_x);
  }
}
