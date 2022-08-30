#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define 	SECTOR_0_BASE_ADDR		0x08000000
#define 	SECTOR_1_BASE_ADDR		0x08004000
#define 	SECTOR_2_BASE_ADDR		0x08008000
#define 	SECTOR_3_BASE_ADDR		0x0800C000
#define 	SECTOR_4_BASE_ADDR		0x08010000
#define 	SECTOR_5_BASE_ADDR		0x08020000
#define 	SECTOR_6_BASE_ADDR		0x08040000
#define 	SECTOR_7_BASE_ADDR		0x08060000

typedef enum {
	SECTOR_0 = 0,
	SECTOR_1,
	SECTOR_2,
	SECTOR_3,
	SECTOR_4,
	SECTOR_5,
	SECTOR_6,
	SECTOR_7,
} eSERTOR_t;

void vectortable_move();

void interrupt_init();
void dma_init();

void uart_init();
void dma_transfer_handler();

void flash_lock() __attribute__((section(".RamFunc")));
void flash_unlock() __attribute__((section(".RamFunc")));
void flash_erase_sector(eSERTOR_t sector) __attribute__((section(".RamFunc")));
void flash_program_byte(void* address, uint8_t* buffer, uint32_t size) __attribute__((section(".RamFunc")));
void reset_system() __attribute__((section(".RamFunc")));
void update_firmware() __attribute__((section(".RamFunc")));

uint8_t receive_done = 0;
static uint8_t rx_dma_buffer[5820] = {0};


/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
int
main() {
	vectortable_move();
	uart_init();
	dma_init();
	interrupt_init();

	while(!receive_done);
	update_firmware();

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
 * \brief
 * \param[in]
 * \param[out]
 * \retval
 */
void
interrupt_init() {
	/*-----------------------DMA Transfer complete Interrupt-----------------------*/
	uint32_t volatile *const DMA_HIFCR    = (uint32_t *)(0x40026000 + 0x0C);
	uint32_t volatile *const DMA1_S7CR    = (uint32_t *)(0x40026000 + 0x10 + (0x18 * 7));
	uint32_t volatile *const NVIC_ISER1 = (uint32_t *)(0xe000e100 + 0x04);
	/*clear stream 7 transfer complete interrupt flag*/
	*DMA_HIFCR |= (1 << 27);	// bit CTCIF7
	/*enable vector interrupt position 47*/
	*NVIC_ISER1 |= (1 << (47 - 32));
	/*change dma-interrupt handler*/
	*((volatile uint32_t *const)(0x20000000 + 0xFC)) = ((uint32_t)dma_transfer_handler | 1);
	/*enable transfer complete enable*/
	*DMA1_S7CR |= (1 << 4);		// bit TCIE
}

/*
 * \brief
 * \param[in]
 * \param[out]
 * \retval
 */
void
dma_init() {
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
	*DMA1_S7PAR = (uint32_t)USART2_DR;
	/*memory address*/
	*DMA1_S7M0AR = (uint32_t)rx_dma_buffer;
	/*circular mode*/
	*DMA1_S7CR |= (1 << 8);
	/*memory increment mode*/
	*DMA1_S7CR |= (1 << 10);
	/*DMA stream enable*/
	*DMA1_S7CR |= (1 << 0);
}

/*
 * \brief
 * \param[in]
 * \param[out]
 * \retval
 */
void
uart_init() {
	/*enable clock peripherals*/
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();

	uint32_t volatile *const GPIOA_MODER = (uint32_t *)(0x40020000 + 0x00);
	uint32_t volatile *const GPIOA_AFRL  = (uint32_t *)(0x40020000 + 0x20);
	uint16_t volatile *const USART2_BRR = (uint16_t *)(0x40004400 + 0x08);
	uint32_t volatile *const USART2_CR1 = (uint32_t *)(0x40004400 + 0x0c);
	uint32_t volatile *const USART2_CR2 = (uint32_t *)(0x40004400 + 0x10);

	/*set PA2 as TX, PA3 as RX*/
	/*alternate mode*/
	*GPIOA_MODER &= ~((0b11 << (2 * 3)) | (0b11 << (2 * 2)));
	*GPIOA_MODER |=   (0b10 << (2 * 3)) | (0b10 << (2 * 2));

	/*alternate function 7*/
	*GPIOA_AFRL &= ~((0b1111 << (4 * 3)) | (0b1111 << (4 * 2)));
	*GPIOA_AFRL |=   (0b0111 << (4 * 3)) | (0b0111 << (4 * 2));

	/*set data frame*/
	/*word length: 8 data bits*/
	*USART2_CR1 &= ~(1 << 12);	// bit M
	/* 1 stop bit*/
	*USART2_CR2 &= (1 << 13);
	*USART2_CR2 &= (1 << 12);
	/*disable parity bit*/
	*USART2_CR1 &= ~(1 << 10);	// bit PCE

	/*set baudrate*/
	//fuart = 16mhz, baud = 9600 -> USART2_BRR = 104.1875
	/*uint16_t DIV_Mantissa = 16000000 / (16 * baudrate);
	uint8_t  DIV_Fraction = round((16000000 % (16 * baudrate)) * 16);
	*USART2_BRR = (DIV_Mantissa << 4) | DIV_Fraction;*/
	*USART2_BRR = (104 << 4) | 3;


	/*enable Tx, Rx*/
	*USART2_CR1 |= (1 << 2) | (1 << 3);	// bit TE, RE

	/*enable UART*/
	*USART2_CR1 |= (1 << 13);	// bit UE
}


/*
 * \brief
 * \param[in]
 * \param[out]
 * \retval
 */
void
dma_transfer_handler() {
	uint32_t volatile *const DMA1_HIFCR    = (uint32_t *)(0x40026000 + 0x0C);

	/*handler*/
	receive_done = 1;

	/*clear stream 7 transfer complete interrupt flag*/
	*DMA1_HIFCR |= (1 << 27);	// bit CTCIF7
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
flash_lock() {
	uint32_t volatile* const FLASH_CR   = (uint32_t*)(0x40023c00 + 0x10);

	/*check LOCK bit*/
	if (((*FLASH_CR >> 31) & 1) == 0) {
		*FLASH_CR |= (1 << 31);
	}
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
flash_unlock() {
	uint32_t volatile* const FLASH_KEYR = (uint32_t*)(0x40023c00 + 0x04);
	uint32_t volatile* const FLASH_CR   = (uint32_t*)(0x40023c00 + 0x10);

	/*check LOCK bit*/
	if (((*FLASH_CR >> 31) & 1) == 1) {
		*FLASH_KEYR = 0x45670123;
		*FLASH_KEYR = 0xCDEF89AB;
	}
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
flash_erase_sector(eSERTOR_t sector) {
	uint32_t volatile* const FLASH_SR   = (uint32_t*)(0x40023c00 + 0x0C);
	uint32_t volatile* const FLASH_CR   = (uint32_t*)(0x40023c00 + 0x10);

	flash_unlock();

	/*check BUSY bit*/
	while (((*FLASH_SR >> 16) & 1) == 1) {}
	/*SET erase sector mode*/
	*FLASH_CR |= (1 << 1);
	/*select sector*/
	*FLASH_CR |= (sector << 3);
	/*start erase*/
	*FLASH_CR |= (1 << 16);
	/*check BUSY bit*/
	while (((*FLASH_SR >> 16) & 1) == 1) {}
	/*CLEAR erase sector mode*/
	*FLASH_CR &= ~(1 << 1);

	flash_lock();
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
flash_program_byte(void* address, uint8_t* buffer, uint32_t size) {
	uint32_t volatile* const FLASH_SR   = (uint32_t*)(0x40023c00 + 0x0C);
	uint32_t volatile* const FLASH_CR   = (uint32_t*)(0x40023c00 + 0x10);

	flash_unlock();

	/*check BUSY bit*/
	while (((*FLASH_SR >> 16) & 1) == 1) {}
	/*SET programming mode*/
	*FLASH_CR |= (1 << 0);
	/*write data*/
	uint8_t* flash = (uint8_t*)address;
	for(uint32_t i = 0; i < size; i++) {
	  *flash = *buffer;
	   flash++;
	   buffer++;
	}
	/*CLEAR programming mode*/
	*FLASH_CR &= ~(1 << 0);
	/*check BUSY bit*/
	while (((*FLASH_SR >> 16) & 1) == 1) {}

	flash_lock();
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
reset_system() {
	uint32_t volatile* const AIRCR   = (uint32_t*)0xE000ED0C;
	*AIRCR = (0x5FA << 16);			// register key
	*AIRCR |= (1 << 2);				// request a reset
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
update_firmware() {
	flash_erase_sector(SECTOR_0);
	flash_program_byte((void*)SECTOR_0_BASE_ADDR, rx_dma_buffer, sizeof(rx_dma_buffer));
	reset_system();
}
