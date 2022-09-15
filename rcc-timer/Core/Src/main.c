#include "main.h"
#include <stdint.h>
#include <string.h>

void vectortable_move();

void tim4_pwm_ch1_start(uint16_t prescaler, uint16_t count, uint8_t duty_cycle);
void tim4_pwm_ch1_stop();

void sys_delay_ms(uint32_t time_milisec);


int main(void)
{
	vectortable_move();

	while (1)
	{
		tim4_pwm_ch1_start(1600, 1000, 50);
	}

	return 0;
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
	uint32_t volatile *const VTOR = (uint32_t *)(0xE000ED08);
	*VTOR = 0x20000000;
}

void sys_delay_ms(uint32_t time_milisec)
{
	volatile uint32_t *const SYS_CSR = (volatile uint32_t *const)(0xe000e010 + 0x00);
	volatile uint32_t *const SYS_RVR = (volatile uint32_t *const)(0xe000e010 + 0x04);

	/*enable clock*/
	*SYS_CSR |= (1<<2);

	/*enable the counter*/
	*SYS_CSR |= (1<<0);

	/*set count*/
	*SYS_RVR = 160000 - 1;
	// F = 16 Mhz -> 1 count = 16 us

	/*delay*/
	for(uint32_t i = 0; i <= time_milisec; i++)
		while(0 == ((*SYS_CSR >> 16) & 1));		// check COUNTFLAG, every 1 ms -> COUNTFLAG = 1

	/*disable the counter*/
	*SYS_CSR &= ~(1 << 0);
}

void tim4_pwm_ch1_start(uint16_t prescaler, uint16_t count, uint8_t duty_cycle)
{
	volatile uint32_t *const RCC_CR 	 = (uint32_t *)(0x40023800 + 0x00);
	volatile uint32_t *const RCC_PLLCFGR = (uint32_t *)(0x40023800 + 0x04);
	volatile uint32_t *const RCC_CFGR	 = (uint32_t *)(0x40023800 + 0x08);
	volatile uint32_t *const RCC_AHB1LPENR = (uint32_t *)(0x40023800 + 0x30);
	volatile uint32_t *const RCC_APB1LPENR = (uint32_t *)(0x40023800 + 0x40);

	*RCC_CR |= (1 << 16) | (1 << 24);
	while(((*RCC_CR >> 17) & 1) == 0);
	while(((*RCC_CR >> 25) & 1) == 0);
	*RCC_PLLCFGR |= (1 << 22);
	*RCC_PLLCFGR |= (9 << 0);
	*RCC_CFGR |= (1 << 1);
	*RCC_AHB1LPENR |= (1 << 3);
	*RCC_APB1LPENR |= (1 << 2);

	volatile uint32_t *const GPIOD_MODER = (uint32_t *)(0x40020c00 + 0x00);
	volatile uint32_t *const GPIOD_AFRH  = (uint32_t *)(0x40020c00 + 0x24);
	volatile uint32_t *const TIM4_CR1    = (uint32_t *)(0x40000800 + 0x00);
	volatile uint32_t *const TIM4_CCMR1  = (uint32_t *)(0x40000800 + 0x18);
	volatile uint32_t *const TIM4_CCER   = (uint32_t *)(0x40000800 + 0x20);
	volatile uint32_t *const TIM4_PCR    = (uint32_t *)(0x40000800 + 0x28);
	volatile uint32_t *const TIM4_ARR    = (uint32_t *)(0x40000800 + 0x2c);
	volatile uint32_t *const TIM4_CCR1   = (uint32_t *)(0x40000800 + 0x34);

	/*alternate mode*/
	*GPIOD_MODER &= ~(0b11 << (2 * 12));
	*GPIOD_MODER |=  (0b10 << (2 * 12));

	/*alternate function 2*/
	*GPIOD_AFRH &= ~(0b1111 << (4 * 4));
	*GPIOD_AFRH |=  (2 << (4 * 4));

	*TIM4_ARR = count - 1;							/*set count*/
	*TIM4_PCR = prescaler - 1;						/*set prescaler*/

	*TIM4_CCR1 = (duty_cycle * (*TIM4_ARR + 1)) / 100;	/*set duty cycle*/

	/*up-counter mode*/
	*TIM4_CR1 &= ~(1 << 4);

	/*pwm mode 1: CNT < CCR1 -> active*/
	*TIM4_CCMR1 &= ~(0b111<<4);
	*TIM4_CCMR1 |= (0b11 << 5);

	*TIM4_CCER |= (1 << 0);						/*enable OC1 chanel 1*/
	*TIM4_CR1  |= (1 << 0);						/*enable counter */
}

void tim4_pwm_ch1_stop()
{
	volatile uint32_t *const TIM4_CR1   = (volatile uint32_t *const)(0x40000800 + 0x00);
	volatile uint32_t *const TIM4_CCER  = (volatile uint32_t *const)(0x40000800 + 0x20);
	volatile uint32_t *const TIM4_CCR1  = (volatile uint32_t *const)(0x40000800 + 0x34);

	*TIM4_CCR1 = 0;								/*set duty cycle*/

	*TIM4_CCER &= ~(1 << 0);						/*disable OC1 chanel 1*/
	*TIM4_CR1  &= ~(1 << 0);						/*disable counter */

	__HAL_RCC_TIM4_CLK_DISABLE();
}
