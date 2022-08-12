#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define ACC_ADDRESS_7BIT	0x19U
#define MAG_ADDRESS_7BIT	0x1EU

void vectortable_move();
void tim_systick_init();
void sys_delay_ms(uint32_t time_milisec);
void i2c_init();
uint8_t i2c_read_data(uint8_t SAD, uint8_t SUB);
void i2c_write_data(uint8_t SAD, uint8_t SUB, uint8_t data);

uint8_t rx_dma_buffer[10];
uint8_t rx_int_buffer[128];
uint32_t rx_index;

uint8_t x_h, x_l, y_h, y_l, z_h, z_l;
int16_t x, y, z;

uint8_t acc_id, mag_id;
char* volatile data_buffer;

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
int
main() {
	vectortable_move();
	tim_systick_init();
	i2c_init();

	while (1) {
		//acc_id = i2c_read_data(ACC_ADDRESS_7BIT, 0x0F);
		//mag_id = i2c_read_data(MAG_ADDRESS_7BIT, 0x4F);

		i2c_write_data(ACC_ADDRESS_7BIT, 0x20, 0x57);
		i2c_read_data(ACC_ADDRESS_7BIT, 0x20);

		x_l = i2c_read_data(ACC_ADDRESS_7BIT, 0x28);
		x_h = i2c_read_data(ACC_ADDRESS_7BIT, 0x29);
		y_l = i2c_read_data(ACC_ADDRESS_7BIT, 0x2A);
		y_h = i2c_read_data(ACC_ADDRESS_7BIT, 0x2B);
		z_l = i2c_read_data(ACC_ADDRESS_7BIT, 0x2C);
		z_h = i2c_read_data(ACC_ADDRESS_7BIT, 0x2D);

		x = (x_h << 8) | x_l;
		y = (y_h << 8) | y_l;
		z = (z_h << 8) | z_l;

		sprintf(data_buffer, "x = %d\t\ty = %d\t\tz = %d\t\t", x, y, z);
		printf(data_buffer);
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
vectortable_move()
{
	/* size(vector_table) = 0x194 + 0x4 - 0x00 = 0x198 */
	/* move vector table from flash to ram */
	void *volatile ram   = (void *volatile)0x20000000;
	void *volatile flash = (void *volatile)0x08000000;
	memcpy(ram, flash, 0x198);

	uint32_t *VTOR = (uint32_t *)(0xE000ED08);
	*VTOR = 0x20000000;
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
tim_systick_init()
{
	uint32_t *SYS_CSR = (uint32_t *)(0xe000e010 + 0x00);
	uint32_t *SYS_RVR = (uint32_t *)(0xe000e010 + 0x00);

	/*clock source: processor clock*/
	*SYS_CSR |= (1 << 2);	// bit CLKSOURCE

	/*set count*/
	*SYS_RVR = 160000 - 1;

	/*enable the counter*/
	*SYS_CSR |= (1 << 0);	// bit ENABLE
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
sys_delay_ms(uint32_t time_milisec)
{
	uint32_t *SYS_CSR = (uint32_t *)(0xe000e010 + 0x00);

	for(uint32_t i = 0; i <= time_milisec; i++)
		/*wait bit COUNTFLAG set 1*/
		while(((*SYS_CSR >> 16) & 1) == 0);
}


/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
i2c_init() {
	/*enable peripherals clock*/
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();

	uint32_t volatile* const GPIOB_MODER  = (uint32_t*)(0x40020400 + 0x00);
	uint32_t volatile* const GPIOB_PUPDR  = (uint32_t*)(0x40020400 + 0x0c);
	uint32_t volatile* const GPIOB_AFLR	  = (uint32_t*)(0x40020400 + 0x20);
	uint32_t volatile* const GPIOB_AFHR	  = (uint32_t*)(0x40020400 + 0x24);
	uint32_t volatile* const I2C1_CR1	  = (uint32_t*)(0x40005400 + 0x00);
	uint32_t volatile* const I2C1_CR2	  = (uint32_t*)(0x40005400 + 0x04);
	uint32_t volatile* const I2C1_CCR	  = (uint32_t*)(0x40005400 + 0x1c);

	/*PB6 PB9 as alternate function mode*/
	*GPIOB_MODER &= ~((0b11 << (2 * 6)) | (0b11 << (2 * 9)));
	*GPIOB_MODER |=   (0b10 << (2 * 6)) | (0b10 << (2 * 9));

	*GPIOB_PUPDR |=   (0b01 << (2 * 6)) | (0b01 << (2 * 9));

	/*alternate function 4*/
	*GPIOB_AFLR  |= (4 << (4 * 6));
	*GPIOB_AFHR  |= (4 << (4 * (9 - 8)));

	/*disable i2c*/
	*I2C1_CR1 &= ~(1 << 0);

	/*peripheral clock : 16 MHz*/
	*I2C1_CR2 |= (16 << 0);

	/*prescale : 100 kHz*/
	*I2C1_CCR = 160;

	/*enable i2c*/
	*I2C1_CR1 |= (1 << 0);
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
uint8_t
i2c_read_data(uint8_t SAD_7bit, uint8_t SUB) {
	uint32_t volatile* const I2C1_CR1   = (uint32_t*)(0x40005400 + 0x00);
	uint32_t volatile* const I2C1_DR    = (uint32_t*)(0x40005400 + 0x10);
	uint32_t volatile* const I2C1_SR1   = (uint32_t*)(0x40005400 + 0x14);
	uint32_t volatile* const I2C1_SR2   = (uint32_t*)(0x40005400 + 0x18);
	uint8_t data;

	SAD_7bit = SAD_7bit << 1;

	/*wait BUSY flag*/
	while (((*I2C1_SR2 >> 1) & 1) == 1);

	/*START condition*/
	*I2C1_CR1 |= (1 << 8);

	/*wait start bit generate*/
	while (((*I2C1_SR1 >> 0) & 1) == 0);
	/*send slave address - write mode*/
	*I2C1_DR = SAD_7bit | 0;
	/*wait ADĐR bit*/
	while (((*I2C1_SR1 >> 1) & 1) == 0);
	/*clear ADDR flag*/
	(void)*(I2C1_SR2);

	/*send register address*/
	*I2C1_DR = SUB;
	/*wait tranfer finish*/
	while (((*I2C1_SR1 >> 2) & 1) == 0);
	/*wait ACK bit*/
	while (((*I2C1_SR1 >> 10) & 1) == 1);

	/*Re-START condition*/
	*I2C1_CR1 |= (1 << 8);

	/*wait start bit generate*/
	while (((*I2C1_SR1 >> 0) & 1) == 0);
	/*send slave address - read mode*/
	*I2C1_DR = SAD_7bit | 1;
	/*wait ADĐR bit*/
	while (((*I2C1_SR1 >> 1) & 1) == 0);
	/*clear ADDR flag*/
	(void)*(I2C1_SR2);

	/*read data*/
	data = *I2C1_DR;

	/*STOP bit*/
	*I2C1_CR1 |= (1 << 9);

	return data;
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
i2c_write_data(uint8_t SAD_7bit, uint8_t SUB, uint8_t data) {
	uint32_t volatile* const I2C1_CR1   = (uint32_t*)(0x40005400 + 0x00);
	uint32_t volatile* const I2C1_DR    = (uint32_t*)(0x40005400 + 0x10);
	uint32_t volatile* const I2C1_SR1   = (uint32_t*)(0x40005400 + 0x14);
	uint32_t volatile* const I2C1_SR2   = (uint32_t*)(0x40005400 + 0x18);

	SAD_7bit = SAD_7bit << 1;

	/*wait BUSY flag*/
	while (((*I2C1_SR2 >> 1) & 1) == 1);

	/*START condition*/
	*I2C1_CR1 |= (1 << 8);

	/*wait start bit generate*/
	while (((*I2C1_SR1 >> 0) & 1) == 0);
	/*send slave address - write mode*/
	*I2C1_DR = SAD_7bit | 0;
	/*wait ADĐR bit*/
	while (((*I2C1_SR1 >> 1) & 1) == 0);
	/*clear ADDR flag*/
	(void)*(I2C1_SR2);

	/*send register address*/
	*I2C1_DR = SUB;
	/*wait tranfer finish*/
	while (((*I2C1_SR1 >> 2) & 1) == 0);
	/*wait ACK bit*/
	while (((*I2C1_SR1 >> 10) & 1) == 1);

	/*write data*/
	*I2C1_DR = data;
	/*wait tranfer finish*/
	while (((*I2C1_SR1 >> 2) & 1) == 0);
	/*wait ACK bit*/
	while (((*I2C1_SR1 >> 10) & 1) == 1);

	/*STOP bit*/
	*I2C1_CR1 |= (1 << 9);
}
