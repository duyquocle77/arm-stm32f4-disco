#include "main.h"
#include <stdint.h>

#define LED_4 12
#define LED_3 13
#define LED_5 14
#define LED_6 15

void gpio_led_init();
void gpio_led_write(uint8_t LED_x, uint8_t value);
void gpio_led_effect_1(uint8_t n);
void gpio_led_effect_2(uint8_t n);
void gpio_led_effect_3();
void gpio_led_effect_4(uint8_t n);
void gpio_led_effect_5();
void gpio_led_effect_6();

int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	gpio_led_init();

	while (1)
	{
		gpio_led_effect_6();
		gpio_led_effect_4(3);
		gpio_led_effect_1(1);
		gpio_led_effect_2(1);
		gpio_led_effect_4(3);
		gpio_led_effect_3();
	}

  return 0;
}

/*
 * @brief	: initialize D.12, D.13, D.14, D.15 as leds on-board
 * @param	: None
 * @retval	: None
 */
void gpio_led_init()
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  uint32_t *GPIOD_MODER = (uint32_t *)(0x40020c00 + 0x00);
  *GPIOD_MODER &= ~(0b11111111<<24);
  *GPIOD_MODER |= (0b01010101<<24);
}

/*
 * @brief	: ON or OFF led
 * @param	: LED_x
 * 		x can be (x: 3...6)
 * @param	: state
 * 		@arg 1 : ON
 * 		@arg 0 : OFF
 * @retval	: None
 */
void gpio_led_write(uint8_t LED_x, uint8_t state)
{
  uint32_t *GPIOD_ODR = (uint32_t *)(0x40020c00 + 0x14);
  if (state == 1)
	  *GPIOD_ODR |= (1 << LED_x);
  else
	  *GPIOD_ODR &= ~(1 << LED_x);
}

void gpio_led_effect_1(uint8_t n)
{
  uint32_t *GPIOD_ODR = (uint32_t *)(0x40020c00 + 0x14);
  uint8_t i, j;
  for(i = 0; i < n; i++)
  {
	  for(j = 12; j < 16; j++)
	  {
		  *GPIOD_ODR |= (1 << j);
		  HAL_Delay(200);
	  }
	  *GPIOD_ODR = 0x00;
	  HAL_Delay(200);
  }
}

void gpio_led_effect_2(uint8_t n)
{
	  uint32_t *GPIOD_ODR = (uint32_t *)(0x40020c00 + 0x14);
	  uint8_t i, j;
	  for(i = 0; i < n; i++)
	  {
		  for(j = 15; j > 11; j--)
		  {
			  *GPIOD_ODR |= (1 << j);
			  HAL_Delay(200);
		  }
		  *GPIOD_ODR = 0x00;
		  HAL_Delay(200);
	  }
}

void gpio_led_effect_3()
{
	uint8_t i = 200;
	while (i > 0)
	{
		for(uint8_t j = 12; j < 16; j++)
		{
			gpio_led_write(j, 1);
			HAL_Delay(i);
			gpio_led_write(j, 0);
			HAL_Delay(i);
		}

		i -= 20;
	}
}

void gpio_led_effect_4(uint8_t n)
{
	  uint32_t *GPIOD_ODR = (uint32_t *)(0x40020c00 + 0x14);
	  for(uint8_t i = 0; i <= n; i++)
	  {
		  *GPIOD_ODR |= (0b1111<<12);
		  HAL_Delay(50);
		  *GPIOD_ODR = 0x00;
		  HAL_Delay(50);
	  }
}

void gpio_led_effect_5()
{
	  uint32_t *GPIOD_ODR = (uint32_t *)(0x40020c00 + 0x14);
	  for(uint8_t i = 8; i < 13; i++)
	  {
		  *GPIOD_ODR |= (0b1111<<i);
		  HAL_Delay(200);
	  }

	  for(uint8_t i = 15; i > 11; i--)
	  {
		  *GPIOD_ODR &= ~(0b1111<<i);
		  HAL_Delay(200);
	  }
}

void gpio_led_effect_6()
{
	uint32_t *GPIOD_ODR = (uint32_t *)(0x40020c00 + 0x14);
	uint8_t i = 200;
	while (i > 0)
	{
		for(uint8_t j = 12; j < 16; j++)
		{
			*GPIOD_ODR |= (0x5<<12);
			HAL_Delay(i);
			*GPIOD_ODR = 0x00;
			HAL_Delay(i);
			*GPIOD_ODR |= (0xA<<12);
			HAL_Delay(i);
			*GPIOD_ODR = 0x00;
			HAL_Delay(i);
		}

		i -= 20;
	}
}
