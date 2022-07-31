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

void tim1_init();
void tim1_handler();
void tim1_delay_ms(uint32_t time_milisec);

uint8_t flag = 0;	/*this flag is status of button*/
uint16_t cnt = 0;	/*this flag is to debug*/

int main(void)
{
	HAL_Init();

	gpio_led_init();
	vectortable_move();
	tim1_init();

	while (1)
	{

			gpio_led_write(LED_6, ON);
			tim1_delay_ms(100);
			gpio_led_write(LED_6, OFF);
			tim1_delay_ms(100);
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

void tim1_init()
{
	__HAL_RCC_TIM1_CLK_ENABLE();

	/*
	 * T = 1ms = 10x 0.1ms
	 * N = 10 < 65535
	 * t = 0.1ms -> f = 10000hz
	 * F = 16 000 000hz -> pre-scaler = F/f = 16 00
	 * */
	uint32_t *ARR = (uint32_t *)(0x40010000 + 0x2c);
	*ARR = 10 - 1;
	uint32_t *PCR = (uint32_t *)(0x40010000 + 0x28);
	*PCR = 1600 - 1;			//prescaler  = f/(PCR + 1)

	/*enable counter CEN bit*/
	uint32_t *CR1 = (uint32_t *)(0x40010000 + 0x00);
	*CR1 |= (1 << 0);

	/*enable interrupt event*/
	uint32_t *DIER = (uint32_t *)(0x40010000 + 0x0c);
	*DIER |= (1<<0);

	uint32_t *ISER0 = (uint32_t *)(0xe000e100 + 0x00); //enable vector interrupt position 25
	*ISER0 |= (1<<25);

	/*
	 * 0xa4 : address of EXTI0 vector
	 * theo quy định của tập lệnh thumb, ta phải |1 với địa chỉ hàm handler
	 *
	 * hàm [void exti0_handler()] có con trỏ hàm là [void(*)()] chứa địa chỉ của hàm
	 * gọi địa chỉ hàm bằng cách gọi tên hàm exti0_handler
	 * */
	/* assign address of user's handler*/
	*((uint32_t *)(0x20000000 + 0xa4)) = ((uint32_t)tim1_handler | 1);
}

void TIM1_UP_TIM10_IRQHandler()
{
	/* clear interrupt flag */
	uint32_t *SR = (uint32_t *)(0x40010000 + 0x10);
	*SR &= ~(1<<0);
}

void tim1_handler()
{

	flag = 1 - flag;
	cnt++;

	/* clear interrupt flag */
	uint32_t *SR = (uint32_t *)(0x40010000 + 0x10);
	*SR &= ~(1<<0);
}

void tim1_delay_ms(uint32_t time_milisec)
{
	cnt = 0;
	while(cnt < time_milisec);
}
