#include "main.h"
#include <stdint.h>
#include <string.h>

#define LED_4 12
#define LED_3 13
#define LED_5 14
#define LED_6 15

typedef enum
{
	OFF,
	ON
} eLED_STATE;

void custom_delay(uint32_t time);

void gpio_led_init();
void gpio_led_write(uint8_t LED_x, eLED_STATE state);

void vectortable_move();

void tim_systick_init();
void tim_systick_handler();
void tim_systick_delay_ms(uint32_t time_milisec);
void systick_handler();
void delay_ms(uint32_t time_milisec);

uint8_t flag = 0;	/*this flag is status of button*/
uint16_t cnt = 0;	/*this flag is to debug*/

int main(void)
{
	HAL_Init();

	gpio_led_init();
	vectortable_move();
	tim_systick_init();

	while (1)
	{

			gpio_led_write(LED_6, ON);
			delay_ms(1000);
			gpio_led_write(LED_6, OFF);
			delay_ms(1000);
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
void gpio_led_write(uint8_t LED_x, eLED_STATE state)
{
  uint32_t *GPIOD_ODR = (uint32_t *)(0x40020c00 + 0x14);
  if (state == ON)
	  *GPIOD_ODR |= (1 << LED_x);
  else
	  *GPIOD_ODR &= ~(1 << LED_x);
}

/*
 * @brief	: move vector table from FLASH to RAM
 * @param	: None
 * @retval	: None
 */
void vectortable_move()
{
	/*
	 * size(vector_table) = 0x194 + 0x4 - 0x00 = 0x198
	 * */
	/* move vector table from flash to ram */
	void *volatile dst = (void *volatile)0x20000000;	// RAM_address
	void *volatile src = (void *volatile)0x08000000;	// FLASH_address
	memcpy(dst, src, 0x198);

	/**/
	uint32_t *VTOR = (uint32_t *)(0xE000ED08);
	*VTOR = 0x20000000;
}

void tim_systick_init()
{
	uint32_t *CSR = (uint32_t *)(0xe000e010 + 0x00);
	/*enable clock*/
	*CSR |= (1<<2);
	/*enable the counter*/
	*CSR |= (1<<0);
	*CSR |= (1<<1);


	/*
	 * F = 16 000 000hz
	 * prescaler = 1 -> f = 16000000 hz -> t = 16*10^-6 s
	 * T = 0.01s
	 * T = N*t -> N = 160 000;
	 * */
	uint32_t *RVR = (uint32_t *)(0xe000e010 + 0x00);
	*RVR = 160000 - 1;
	*((uint32_t *)(0x20000000 + 0x3c)) = ((uint32_t)systick_handler | 1);
}

void systick_handler()
{
	cnt++;
}

void delay_ms(uint32_t time_milisec)
{
	cnt = 0;
	while(cnt < time_milisec);
}

void tim_systick_delay_ms(uint32_t time_milisec)
{
	uint32_t *CSR = (uint32_t *)(0xe000e010 + 0x00);
	for(uint32_t i = 0; i <= time_milisec; i++)
		while(((*CSR>>16)&1)==0);
}
