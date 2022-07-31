#include "main.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

void vectortable_move();
void interrupt_init();
void dma_init();
void uart_init();
void uart_send_char(uint8_t charac);
void uart_send_string(char *string);
uint8_t uart_receive_char();
void uart_receive_string();
void uart_receive_handler();
void dma_transfer_handler();
void sys_delay_ms(uint32_t time_milisec);

uint8_t data;
uint8_t rx_dma_buffer[10];
uint8_t rx_int_buffer[128];
uint8_t rx_index;

int main(void)
{
	vectortable_move();

	uart_init();
	dma_init();
	interrupt_init();

	while (1)
	{
		//uart_send_char('x');
		//uart_send_string("Hello master \r\n");

		//data = uart_receive_char();
		//uart_receive_string();
		//sys_delay_ms(500);
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
	/* size(vector_table) = 0x194 + 0x4 - 0x00 = 0x198 */
	/* move vector table from flash to ram */
	void *volatile ram   = (void *)0x20000000;
	void *volatile flash = (void *)0x08000000;
	memcpy(ram, flash, 0x198);

	uint32_t volatile *const VTOR = (uint32_t *)(0xE000ED08);
	*VTOR = 0x20000000;
}

void interrupt_init()
{
	/*-----------------------UART Receive complete Interrupt-----------------------*/
	uint32_t volatile *const USART2_CR1 = (uint32_t *)(0x40004400 + 0x0c);
	uint32_t volatile *const NVIC_ISER1 = (uint32_t *)(0xe000e100 + 0x04);
	//enable vector interrupt position 38
	*NVIC_ISER1 |= (1 << (38 - 32));
	/*change uart-interrupt handler*/
	*((volatile uint32_t *const)(0x20000000 + 0xD8)) = ((uint32_t)uart_receive_handler | 1);
	/*enable interrupt Rx*/
	*USART2_CR1 |= (1 << 5);	// bit RXNEIE


	/*-----------------------DMA Transfer complete Interrupt-----------------------*/
	uint32_t volatile *const DMA_HIFCR    = (uint32_t *)(0x40026000 + 0x0C);
	uint32_t volatile *const DMA1_S7CR    = (uint32_t *)(0x40026000 + 0x10 + (0x18 * 7));
	/*clear stream 7 transfer complete interrupt flag*/
	*DMA_HIFCR |= (1 << 27);	// bit CTCIF7
	/*enable vector interrupt position 47*/
	*NVIC_ISER1 |= (1 << (47 - 32));
	/*change dma-interrupt handler*/
	*((volatile uint32_t *const)(0x20000000 + 0xFC)) = ((uint32_t)dma_transfer_handler | 1);
	/*enable transfer complete enable*/
	*DMA1_S7CR |= (1 << 4);		// bit TCIE
}

void dma_init()
{
	__HAL_RCC_DMA1_CLK_ENABLE();

	/*-----------------------Rx DMA-----------------------*/
	uint32_t volatile *const USART2_DR   = (uint32_t *)(0x40004400 + 0x04);
	uint32_t volatile *const USART2_CR3  = (uint32_t *)(0x40004400 + 0x14);
	uint32_t volatile *const DMA1_S7CR    = (uint32_t *)(0x40026000 + 0x10 + (0x18 * 7));
	uint32_t volatile *const DMA1_S7NDTR  = (uint32_t *)(0x40026000 + 0x14 + (0x18 * 7));
	uint32_t volatile *const DMA1_S7PAR   = (uint32_t *)(0x40026000 + 0x18 + (0x18 * 7));
	uint32_t volatile *const DMA1_S7M0AR  = (uint32_t *)(0x40026000 + 0x1c + (0x18 * 7));
	/*Rx DMA enable*/
	*USART2_CR3 |= (1 << 6);
	/*channel 6*/
	*DMA1_S7CR |= (6 << 25);
	/*number of data*/
	*DMA1_S7NDTR = sizeof(rx_dma_buffer);
	/*peripheral address*/
	*DMA1_S7PAR = (0x40004400 + 0x04);
	/*memory address*/
	*DMA1_S7M0AR = (uint32_t)rx_dma_buffer;
	/*circular mode*/
	*DMA1_S7CR |= (1 << 8);
	/*memory increment mode*/
	*DMA1_S7CR |= (1 << 10);
	/*DMA stream enable*/
	*DMA1_S7CR |= (1 << 0);
}

void sys_delay_ms(uint32_t time_milisec)
{
	uint32_t volatile *const SYS_CSR = (uint32_t *)(0xe000e010 + 0x00);
	uint32_t volatile *const SYS_RVR = (uint32_t *)(0xe000e010 + 0x04);

	/*enable clock*/
	*SYS_CSR |= (1 << 2);

	/*enable the counter*/
	*SYS_CSR |= (1 << 0);

	/*set count*/
	*SYS_RVR = 16000;
	// F = 16 Mhz ->     1 count = 16 us
	/*delay*/
	for(uint32_t i = 0; i <= time_milisec; i++)
		while(((*SYS_CSR >> 16) & 1) == 0);		// check COUNTFLAG, every 1 ms -> COUNTFLAG = 1

	/*disable the counter*/
	*SYS_CSR &= ~(1 << 0);
}

void uart_init()
{
	/*enable clock peripherals*/
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();

	uint32_t volatile *const GPIOA_MODER = (uint32_t *)(0x40020000 + 0x00);
	uint32_t volatile *const GPIOA_AFRL  = (uint32_t *)(0x40020000 + 0x20);
	uint16_t volatile *const USART2_BRR = (uint16_t *)(0x40004400 + 0x08);
	uint32_t volatile *const USART2_CR1 = (uint32_t *)(0x40004400 + 0x0c);

	/*set PA2 as TX, PA3 as RX*/
	/*alternate mode*/
	*GPIOA_MODER &= ~((0b11 << (2 * 3)) | (0b11 << (2 * 2)));
	*GPIOA_MODER |=   (0b10 << (2 * 3)) | (0b10 << (2 * 2));

	/*alternate function 7*/
	*GPIOA_AFRL &= ~((0b1111 << (4 * 3)) | (0b1111 << (4 * 2)));
	*GPIOA_AFRL |=   (0b0111 << (4 * 3)) | (0b0111 << (4 * 2));

	/*set baudrate*/
	//fuart = 16mhz, baud = 9600 -> USART2_BRR = 104.1875
	/*uint16_t DIV_Mantissa = 16000000 / (16 * baudrate);
	uint8_t  DIV_Fraction = round((16000000 % (16 * baudrate)) * 16);
	*USART2_BRR = (DIV_Mantissa << 4) | DIV_Fraction;*/
	*USART2_BRR = (104 << 4) | 3;

	/*set data frame*/
	/*8-bits*/
	*USART2_CR1 &= ~(1 << 12);	// bit M
	/*disable parity*/
	*USART2_CR1 &= ~(1 << 10);	// bit PCE

	/*enable Tx, Rx*/
	*USART2_CR1 |= (1 << 2) | (1 << 3);	// bit TE, RE

	/*enable UART*/
	*USART2_CR1 |= (1 << 13);	// bit UE
}

void uart_send_char(uint8_t charac)
{
	uint32_t volatile *const UART2_SR = (uint32_t *)(0x40004400 + 0x00);
	uint8_t  volatile *const UART2_DR = (uint8_t *)(0x40004400 + 0x04);

	/*wait data empty*/
	while(((*UART2_SR >> 7) & 1) == 0);

	/*transmiss data*/
	*UART2_DR = charac;

	/*wait transmission complete*/
	while(((*UART2_SR >> 6) & 1) == 0);

	/*clear TC bit*/
	*UART2_SR &= ~(1 << 6);
}

void uart_send_string(char *string)
{
	while(*string != '\0')
	{
		uart_send_char(*string);
		string++;
	}
}

uint8_t uart_receive_char()
{
	uint32_t volatile *const UART2_SR = (uint32_t *)(0x40004400 + 0x00);
	uint8_t  volatile *const UART2_DR = (uint8_t *)(0x40004400 + 0x04);

	/*wait data not empty*/
	while(((*UART2_SR >> 5) & 1) == 0);

	/*clear RXNE*/
	*UART2_SR &= ~(1 << 5);

	return *UART2_DR;
}

void uart_receive_string()
{
	uint32_t volatile *const UART2_SR = (uint32_t *)(0x40004400 + 0x00);
	uint8_t  volatile *const UART2_DR = (uint8_t *)(0x40004400 + 0x04);

	/*wait data not empty*/
	while(((*UART2_SR >> 5) & 1) == 0);

	/*receive data*/
	rx_int_buffer[rx_index] = *UART2_DR;
	rx_index++;

	if (rx_index >= 50) rx_index = 0;

	/*clear RXNE*/
	*UART2_SR &= ~(1 << 5);
}

void uart_receive_handler()
{
	uint32_t volatile *const USART2_SR = (uint32_t *)(0x40004400 + 0x00);
	uint32_t volatile *const UART2_DR  = (uint32_t *)(0x40004400 + 0x04);

	/*read data*/
	rx_int_buffer[rx_index] = *UART2_DR;
	rx_index++;

	if (rx_index >= 50) rx_index = 0;

	/*clear RXNE*/
	*USART2_SR &= ~(1 << 5);
}

void
dma_transfer_handler() {
	uint32_t volatile *const DMA1_HIFCR    = (uint32_t *)(0x40026000 + 0x0C);

	/*handler*/
	__asm("NOP");

	/*clear stream 7 transfer complete interrupt flag*/
	*DMA1_HIFCR |= (1 << 27);	// bit CTCIF7
}
