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
void exti1_init();

uint8_t flag_1 = 0;	/*this flag is status of on-board button*/
uint8_t flag_2 = 0;	/*this flag is status of external button*/
uint8_t cnt_1 = 0;	/*this flag is to debug*/
uint8_t cnt_2 = 0;	/*this flag is to debug*/

int main(void)
{
 	HAL_Init();

	gpio_led_init();
	exti0_init();
	exti1_init();

	while (1)
	{
		 if(flag_1 == 1)
			gpio_led_write(LED_6, 1);
		 else
			gpio_led_write(LED_6, 0);

		 if(flag_2 == 1)
			gpio_led_write(LED_3, 1);
		 else
			gpio_led_write(LED_3, 0);
	}

	return 0;
}

/*
 * @brief	: creat delay time
 * @param	: time
 * 		@arg time can be in range (0...2^32)
 *
 * @retval	: None
 */
void custom_delay(uint32_t time)
{
	for(uint32_t i = 0; i < time; i++)
	{
		__asm("NOP");
	}
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
 * @brief	: set or reset led
 * @param	: LED_x
 * 		x can be (x: 3...6)
 * @param	: state
 * 		@arg 1 : SET
 * 		@arg 0 : RESET
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

void gpio_button_init()
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
 * @brief read button state
 * @param None
 * @retval uint8_t:
 * 		@arg 1 button in active
 * 		@arg 2 button in in-active
 */
uint8_t gpio_button_read()
{
	uint32_t *GPIOA_IDR = (uint32_t *)(0x40020010);
	if(((*GPIOA_IDR >> 1) & 1) == 1)
		return 1;
	else
		return 0;
}

/*
 * @brief	: initialize external interrupt EXTI0
 * @param	: None
 * @retval	: None
 */
void exti0_init()
{
	/*Configure External Interrupt pin   */
	uint32_t *EXTI_IMR = (uint32_t *)(0x40013c00 + 0x00); //enable mask interrupt
	*EXTI_IMR |= (1<<0);

	uint32_t *EXTI_RTSR = (uint32_t *)(0x40013c00 + 0x08); //rising-mode
	*EXTI_RTSR |= (1<<0);

	uint32_t *NVIC_ISER0 = (uint32_t *)(0xe000e100 + 0x00); //enable vector interrupt position 6
	*NVIC_ISER0 |= (1<<6);
}

/*
 * @brief	: handle vector interrupt EXTI0
 * @param	: None
 * @retval	: None
 */
void EXTI0_IRQHandler()
{
	//custom_delay(500000);
	flag_1 = 1 - flag_1;
	cnt_1++;

	/* clear interrupt flag */
	uint32_t *EXTI_PR = (uint32_t *)(0x40013c00 + 0x14);
	*EXTI_PR |= (1<<0);
}

/*
 * @brief	: initialize external interrupt EXTI1 on A.1 pin
 * @param	: None
 * @retval	: None
 */
void exti1_init()
{
	/*Configure External Interrupt pin   */
	uint32_t *EXTI_IMR = (uint32_t *)(0x40013c00 + 0x00); //enable mask interrupt on line 1
	*EXTI_IMR |= (1<<1);

	uint32_t *EXTI_RTSR = (uint32_t *)(0x40013c00 + 0x08); //rising-mode in line 1
	*EXTI_RTSR |= (1<<1);

	uint32_t *NVIC_ISER0 = (uint32_t *)(0xe000e100 + 0x00); //enable vector interrupt position 7
	*NVIC_ISER0 |= (1<<7);
}

void EXTI1_IRQHandler()
{
	//custom_delay(500000);
	flag_2 = 1 - flag_2;
	cnt_2++;

	/* clear interrupt flag */
	uint32_t *EXTI_PR = (uint32_t *)(0x40013c00 + 0x14);
	*EXTI_PR |= (1<<1);
}
