#include "main.h"
#include <stdint.h>

/*leds on-board*/
#define LED_4 12
#define LED_3 13
#define LED_5 14
#define LED_6 15

/*
 *\brief
 *\param[in]
 *\param[out]
 * \retval
 */
typedef enum
{
	NO_CLICK = 0,
	SINGLE_CLICK,
	LONG_CLICK,
	DOUBLE_CLICK,
	TRIPLE_CLICK,
} eButton_state;


uint8_t flag = 0;
uint8_t flag_2 = 0;
uint16_t time = 0;
uint16_t time_2 = 0;
uint16_t cnt  = 0;
uint16_t cnt_2  = 0;

void gpio_button_init();
uint8_t gpio_button_read();
eButton_state gpio_button_3state();
eButton_state tim_button_4state();
void gpio_led_init();
void gpio_led_write(uint8_t LED_x, uint8_t value);
void gpio_led_toggle(uint8_t LED_x);

int
main(void) {
	HAL_Init();
	gpio_button_init();
	gpio_led_init();


	while (1) {
		eButton_state button = tim_button_4state();
		switch (button)
		{
			case NO_CLICK:
				break;
			case SINGLE_CLICK:
				gpio_led_toggle(LED_4);
				break;
			case LONG_CLICK:
				gpio_led_toggle(LED_3);
				break;
			case DOUBLE_CLICK:
				gpio_led_toggle(LED_5);
				break;
			case TRIPLE_CLICK:
				gpio_led_toggle(LED_6);
				break;
		}
	}
	return 0;
}

/*
 *\brief
 *\param[in]
 *\param[out]
 * \retval
 */
void
gpio_button_init() {
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Input  */
	uint32_t *GPIOA_MODER = (uint32_t *)(0x40020000 + 0x00);
	*GPIOA_MODER &= ~(0b11<<0);	// Set PA0 pin Input

	uint32_t *GPIOA_PUPDR = (uint32_t *)(0x40020000 + 0x0c);
	*GPIOA_PUPDR &= ~(0b11<<0);	// Set PA0 pin Floating-level
}

/*
 *\brief
 *\param[in]
 *\param[out]
 * \retval
 */
uint8_t
gpio_button_read() {
	uint32_t *GPIOA_IDR = (uint32_t *)(0x40020000 + 0x10);

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
 * \retval
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
 * \retval
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
 * \retval
 */
void
gpio_led_toggle(uint8_t LED_x)
{
  uint32_t *GPIOD_ODR = (uint32_t *)(0x40020c00 + 0x14);
  uint32_t *GPIOD_IDR = (uint32_t *)(0x40020c00 + 0x10);

  if (((*GPIOD_IDR >> LED_x) & 1) == 0) {
	  *GPIOD_ODR |= (1<<LED_x);
  }
  else {
	  *GPIOD_ODR &= ~(1<<LED_x);
  }
}

/*
 *\brief
 *\param[in]
 *\param[out]
 * \retval
 */
eButton_state
gpio_button_3state() {
	while (gpio_button_read() == 1) {
		 HAL_Delay(200);	// delay chong nhieu nut nhan
		 flag = 1;
		 cnt++;
	 }

	if (flag == 1) {
		while (gpio_button_read() == 0) {
			time++;
			HAL_Delay(1);

			if (time > 200) {
				if(cnt <= 2) {
					time = 0;
					flag = 0;
					cnt = 0;
					return SINGLE_CLICK;
				}
				else {
					time = 0;
					flag = 0;
					cnt = 0;
					return LONG_CLICK;
				}
			}
		}

		while (gpio_button_read() == 1) {
			HAL_Delay(200);	// delay chong nhieu nut nhan
			flag = 0;
			time = 0;
			cnt = 0;
			return DOUBLE_CLICK;
		}
	}
	return NO_CLICK;
}

/*
 *\brief
 *\param[in]
 *\param[out]
 * \retval
 */
eButton_state
tim_button_4state() {
	while (gpio_button_read() == 1) {
		 HAL_Delay(200);	// delay chong nhieu nut nhan
		 flag = 1;
		 cnt++;
	 }

	if (flag == 1) {
		while (gpio_button_read() == 0) {
			time++;
			HAL_Delay(1);

			if (time > 200) {
				if(cnt <= 2) {
					time = 0;
					flag = 0;
					cnt = 0;
					return SINGLE_CLICK;
				}
				else {
					time = 0;
					flag = 0;
					cnt = 0;
					return LONG_CLICK;
				}
			}
		}

		while (gpio_button_read() == 1) {
			HAL_Delay(200);	// delay chong nhieu nut nhan
			flag = 0;
			time = 0;
			cnt = 0;
			flag_2 = 1;
			cnt_2++;
		}

		if (flag_2 == 1) {
			while (gpio_button_read() == 0) {
				time_2++;
				HAL_Delay(1);

				if(time_2 > 200) {
					if(cnt_2 <= 2) {
						time = 0;
						flag = 0;
						cnt = 0;
						time_2 = 0;
						flag_2 = 0;
						cnt_2 = 0;
						return DOUBLE_CLICK;
					}
				}
			}
			while (gpio_button_read() == 1) {
				HAL_Delay(200);	// delay chong nhieu nut nhan
				time = 0;
				flag = 0;
				cnt = 0;
				time_2 = 0;
				flag_2 = 0;
				cnt_2 = 0;
				return TRIPLE_CLICK;
			}
		}
	}
	return NO_CLICK;
}
