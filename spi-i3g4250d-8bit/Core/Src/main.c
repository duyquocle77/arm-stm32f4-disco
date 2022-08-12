#include "main.h"
#include <stdint.h>
#include <string.h>

void vectortable_move();
void tim_systick_init();
void sys_delay_ms(uint32_t time_milisec);
void interrupt_init();
void dma_init();
void spi_init();
void spi_ss_enable();
void spi_ss_disable();
void spi_send_data(uint8_t data);
void spi_send_string(char* str);
uint8_t spi_receive_data();
void spi_cfg_sensor(uint8_t address, uint8_t data);
void spi_read_sensor(uint8_t address);
void spi_receive_handler();
void dma_transfer_handler();

uint8_t rx_dma_buffer[10];
uint8_t rx_int_buffer[128];
uint32_t rx_index;

uint8_t x_h, x_l, y_h, y_l, z_h, z_l;
short x, y, z;

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
	spi_init();
	//dma_init();
	//interrupt_init();

	while (1) {
		spi_ss_enable();
		spi_send_data(0x20|0x40);
		spi_send_data(0x0F);
		//spi_send_data(0x00);
		spi_ss_disable();

		spi_ss_enable();
		spi_send_data(0x20|0x80);
		spi_receive_data();
		spi_ss_disable();

		spi_ss_enable();
		spi_send_data(0x28|0x80|0x40);
		x_l = spi_receive_data();
		x_h = spi_receive_data();
		y_l = spi_receive_data();
		y_h = spi_receive_data();
		z_l = spi_receive_data();
		z_h = spi_receive_data();
		spi_ss_disable();

		x = (x_h << 8)|x_l;
		y = (y_h << 8)|y_l;
		z = (z_h << 8)|z_l;

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
 * \brief
 * \param[in]
 * \param[out]
 * \retval
 */
void
interrupt_init() {
	/*-----------------------SPI Receive complete Interrupt-----------------------*/
	uint32_t volatile *const SPI1_CR2   = (uint32_t *)(0x40013000 + 0x04);
	uint32_t volatile *const NVIC_ISER1 = (uint32_t *)(0xe000e100 + 0x04);
	//enable vector interrupt position 35
	*NVIC_ISER1 |= (1 << (35 - 32));
	/*change uart-interrupt handler*/
	*((volatile uint32_t *const)(0x20000000 + 0xCC)) = ((uint32_t)spi_receive_handler | 1);
	/*enable interrupt Rx*/
	*SPI1_CR2 |= (1 << 6);	// bit RXNEIE


	/*-----------------------DMA Transfer complete Interrupt-----------------------*/
	uint32_t volatile *const DMA2_LIFCR   = (uint32_t *)(0x40026400 + 0x08);
	uint32_t volatile *const DMA2_S0CR    = (uint32_t *)(0x40026400 + 0x10 + (0x18 * 0));
	/*clear stream 0 transfer complete interrupt flag*/
	*DMA2_LIFCR |= (1 << 5);	// bit CTCIF0
	/*enable vector interrupt position 56*/
	*NVIC_ISER1 |= (1 << (56 - 32));
	/*change dma-interrupt handler*/
	*((volatile uint32_t *const)(0x20000000 + 0x120)) = ((uint32_t)dma_transfer_handler | 1);
	/*enable transfer complete enable*/
	*DMA2_S0CR |= (1 << 4);		// bit TCIE
}

/*
 * \brief
 * \param[in]
 * \param[out]
 * \retval
 */
void
dma_init() {
	__HAL_RCC_DMA2_CLK_ENABLE();

	/*-----------------------Rx SPI-----------------------*/
	uint32_t volatile* const SPI1_CR2	  = (uint32_t*)(0x40013000 + 0x04);
	uint32_t volatile* const SPI1_DR      = (uint32_t*)(0x40013000 + 0x0c);
	uint32_t volatile *const DMA2_S0CR    = (uint32_t *)(0x40026400 + 0x10 + (0x18 * 0));
	uint32_t volatile *const DMA2_S0NDTR  = (uint32_t *)(0x40026400 + 0x14 + (0x18 * 0));
	uint32_t volatile *const DMA2_S0PAR   = (uint32_t *)(0x40026400 + 0x18 + (0x18 * 0));
	uint32_t volatile *const DMA2_S0M0AR  = (uint32_t *)(0x40026400 + 0x1c + (0x18 * 0));
	/*Rx SPI enable*/
	*SPI1_CR2 |= (1 << 0);	//bit RXDMAEN
	/*channel 3*/
	*DMA2_S0CR |= (3 << 25);
	/*number of data*/
	*DMA2_S0NDTR = sizeof(rx_dma_buffer);
	/*peripheral address*/
	*DMA2_S0PAR = (uint32_t)SPI1_DR;
	/*memory address*/
	*DMA2_S0M0AR = (uint32_t)rx_dma_buffer;
	/*circular mode*/
	*DMA2_S0CR |= (1 << 8);
	/*memory increment mode*/
	*DMA2_S0CR |= (1 << 10);
	/*DMA stream enable*/
	*DMA2_S0CR |= (1 << 0);
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
spi_init() {
	/*enable peripherals clock*/
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_SPI1_CLK_ENABLE();


	uint32_t volatile* const GPIOA_MODER  = (uint32_t*)(0x40020000 + 0x00);
	uint32_t volatile* const GPIOA_AFLR	  = (uint32_t*)(0x40020000 + 0x20);
	uint32_t volatile* const GPIOE_MODER  = (uint32_t*)(0x40021000 + 0x00);
	uint32_t volatile* const GPIOE_OTYPER = (uint32_t*)(0x40021000 + 0x04);
	uint32_t volatile* const GPIOE_ODR	  = (uint32_t*)(0x40021000 + 0x14);
	uint32_t volatile* const SPI1_CR1	  = (uint32_t*)(0x40013000 + 0x00);

	/*PA5 PA6 PA7 as alternate function mode*/
	*GPIOA_MODER &= ~((0b10 << (2 * 5)) | (0b10 << (2 * 6)) | (0b10 << (2 * 7)));
	*GPIOA_MODER |=   (0b10 << (2 * 5)) | (0b10 << (2 * 6)) | (0b10 << (2 * 7));
	/*alternate function 5*/
	*GPIOA_AFLR  |= (5 << (4 * 5)) | (5 << (4 * 6)) | (5 << (4 * 7));

	/*PE3 as output mode*/
	*GPIOE_MODER  |= (0b01 << (2 * 3));
	*GPIOE_OTYPER &= ~(1 << 3);
	*GPIOE_ODR    |= (1 << 3);

	/*spi baudrate = Fpclk/16*/
	*SPI1_CR1 |= (0b011 << 3);

	/*clock pin mode 0*/
	*SPI1_CR1 |= (1 << 1);	//CPOL
	*SPI1_CR1 |= (1 << 0);	//CPHA

	/*spi master config*/
	*SPI1_CR1 |= (1 <<2 );

	/*data frame: 8bit*/
	*SPI1_CR1 &= ~(1 << 11);
	//*SPI1_CR1 |= (1 << 11);

	/*SS pin mode : software*/
	*SPI1_CR1 |= (1 << 8) |(1 << 9);


	//*SPI1_CR1 |= (1 << 7);

	/*enable spi*/
	*SPI1_CR1 |= (1 << 6);
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
spi_ss_enable() {
	uint32_t volatile* const GPIOE_ODR = (uint32_t*)(0x40021000 + 0x14);
	*GPIOE_ODR &= ~(1 << 3);
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
spi_ss_disable() {
	uint32_t volatile* const GPIOE_ODR = (uint32_t*)(0x40021000 + 0x14);
	*GPIOE_ODR |= (1 << 3);
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
spi_send_data(uint8_t data) {
	uint32_t volatile* const SPI1_SR   = (uint32_t*)(0x40013000 + 0x08);
	uint32_t volatile* const SPI1_DR   = (uint32_t*)(0x40013000 + 0x0c);
	uint8_t tmp;


	//check TXE
	while (0 == ((*SPI1_SR >> 1) & 1));
	//send data
	*SPI1_DR = (uint32_t)data;
	//*SPI1_DR = ((uint32_t)data << 8)|0xFF;
	//check RXNE
	while (0 == ((*SPI1_SR >> 0) & 1));
	//clear trash
	tmp = (uint8_t)*SPI1_DR;
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void spi_send_string(char* str) {
	while (*str != '\0') {
		spi_send_data(*str);
		str++;
	}
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
uint8_t
spi_receive_data() {
	uint32_t volatile* const SPI1_SR   = (uint32_t*)(0x40013000 + 0x08);
	uint32_t volatile* const SPI1_DR   = (uint32_t*)(0x40013000 + 0x0c);
	uint8_t data;

	//check TXE
	while (0 == ((*SPI1_SR >> 1) & 1));
	//send trash to create master clock
	*SPI1_DR = 0xFFFF;
	//check RXNE
	while (0 == ((*SPI1_SR >> 0) & 1));
	//read data from slave
	data = (uint8_t)*SPI1_DR;

	return data;
}

/*
 * \brief
 * \param[in]
 * \param[out]
 * \retval
 */
void
spi_read_sensor(uint8_t address) {
	address |= 0x80;
	spi_send_data(address);
	spi_send_data(0xFF);
}

/*
 * \brief
 * \param[in]
 * \param[out]
 * \retval
 */
void
spi_cfg_sensor(uint8_t address, uint8_t data) {
	spi_send_data(address);
	spi_send_data(data);
}

/*
 * \brief
 * \param[in]
 * \param[out]
 * \retval
 */
void
spi_receive_handler()
{
	uint32_t volatile *const SPI1_SR = (uint32_t *)(0x40013000 + 0x08);
	uint32_t volatile *const SPI1_DR = (uint32_t *)(0x40013000 + 0x0c);

	/*read data*/
	rx_int_buffer[rx_index] = *SPI1_DR;
	rx_index++;

	if (rx_index >= 128) {
		rx_index = 0;
	}

	/*clear RXNE*/
	*SPI1_SR &= ~(1 << 0);
}

/*
 * \brief
 * \param[in]
 * \param[out]
 * \retval
 */
void
dma_transfer_handler() {
	uint32_t volatile *const DMA2_LIFCR    = (uint32_t *)(0x40026400 + 0x08);

	/*handler*/
	__asm("NOP");

	/*clear stream 0 transfer complete interrupt flag*/
	*DMA2_LIFCR |= (1 << 5);	// bit CTCIF0
}
