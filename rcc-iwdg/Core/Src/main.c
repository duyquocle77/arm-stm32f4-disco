#include "main.h"

typedef enum {
	OFF,
	ON
} Led_state_t;

void SysTick_Init();
void SysTick_Handler();
void Delay_ms(uint32_t time_ms);
void LED_Init();
void LED_Control(Led_state_t state);
void watchdog_start();
void watchdog_stop();

static uint32_t sys_cnt = 0;

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
int
main() {
	SysTick_Init();
	LED_Init();

	watchdog_start();

	/*program need to monitor*/
	LED_Control(ON);
	Delay_ms(100);
	LED_Control(OFF);
	Delay_ms(100);
	LED_Control(ON);
	Delay_ms(100);
	LED_Control(OFF);
	Delay_ms(100);

	while (1);

	watchdog_stop();

	while (1) {

	}

	return 0;
}

void SysTick_Init() {
	volatile uint32_t* const SYST_CSR = (uint32_t*)(0xe000e010 + 0x00);
	volatile uint32_t* const SYST_RVR = (uint32_t*)(0xe000e010 + 0x04);

	*SYST_RVR = 16000 - 1;
	*SYST_CSR |= (1 << 0) | (1 << 1) | (1 << 2);
}

void SysTick_Handler() {
	sys_cnt++;
}

void Delay_ms(uint32_t time_ms) {
	sys_cnt = 0;
	while (sys_cnt < time_ms);
}

void LED_Init() {
	volatile uint32_t* const RCC_AHB1ENR   = (uint32_t*)(0x40023800 + 0x30);
	*RCC_AHB1ENR |= (1 << 3);

	volatile uint32_t * const GPIOD_MODER = (uint32_t*)(0x40020c00 + 0x00);
	*GPIOD_MODER &= ~(0b11111111 << 24);
	*GPIOD_MODER |= (0b01010101 << 24);
}

void LED_Control(Led_state_t state) {
	uint32_t volatile* const GPIOD_ODR = (uint32_t*)(0x40020c00 + 0x14);
	if (state == ON) {
		*GPIOD_ODR |= (1 << 12);
	}
	else {
		*GPIOD_ODR &= ~(1 << 12);
	}
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
watchdog_start() {
	volatile uint32_t *const RCC_CSR   =(uint32_t *)(0x40023800 + 0x74);
	*RCC_CSR |= (1 << 0);			/* enable LSI_RC = 32 kHz */

	uint32_t volatile* const IWDG_KR   = (uint32_t*)(0x40003000 + 0x00);
	uint32_t volatile* const IWDG_PR   = (uint32_t*)(0x40003000 + 0x04);
	uint32_t volatile* const IWDG_RLR  = (uint32_t*)(0x40003000 + 0x08);


	*IWDG_KR = 0x5555;				/* enable access IWDG*/
	*IWDG_PR |= (0b011 << 0);		/* set pre-scaler: F_iwdg = 32 / 32 = 1 kHz -> T = 1 ms */
	/* watchdog reset after 1s */
	*IWDG_RLR = 500;				/* set reload value: t = 1000 * 1 = 1000 ms = 1s */
	*IWDG_KR = 0xCCCC;				/* start IWDG */

}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
watchdog_stop() {
	uint32_t volatile* const IWDG_KR   = (uint32_t*)(0x40003000 + 0x00);
	/* value reloaded in the counter and the watchdog reset is prevented */
	*IWDG_KR = 0xAAAA;
}
