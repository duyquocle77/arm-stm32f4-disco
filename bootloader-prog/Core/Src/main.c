#include "main.h"
#include <stdio.h>
#include <stdint.h>

#define 	SECTOR_0_BASE_ADDR		0x08000000
#define 	SECTOR_1_BASE_ADDR		0x08004000
#define 	SECTOR_2_BASE_ADDR		0x08008000

#define 	BOOTLOADER_BASE_ADDR		SECTOR_0_BASE_ADDR
#define 	APPLICATION_1_BASE_ADDR		SECTOR_1_BASE_ADDR
#define 	APPLICATION_2_BASE_ADDR		SECTOR_2_BASE_ADDR

#define 	RESET_HANDLER_OFFSET			0x00000004
#define 	RESET_HANDLER_APPLICATION_1		(APPLICATION_1_BASE_ADDR + RESET_HANDLER_OFFSET)
#define 	RESET_HANDLER_APPLICATION_2		(APPLICATION_2_BASE_ADDR + RESET_HANDLER_OFFSET)

void gpio_button_init();
uint8_t gpio_button_read();

uint32_t val = 0;

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
int
main() {
	void (*boot_jump)(void);


	if (gpio_button_read() == 1) {
		val = *((uint32_t*)RESET_HANDLER_APPLICATION_1);
	}
	else {
		val = *((uint32_t*)RESET_HANDLER_APPLICATION_2);
	}

	/*if (gpio_button_read() == 1) {
		boot_jump = (void (*)(void))*((uint32_t*)RESET_HANDLER_APPLICATION_1);
	}
	else {
		boot_jump = (void (*)(void))*((uint32_t*)RESET_HANDLER_APPLICATION_2);
	}*/

	boot_jump = val;

	boot_jump();

	while (1) {

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
