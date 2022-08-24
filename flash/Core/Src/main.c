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
void tim_systick_init();
void sys_delay_ms(uint32_t time_milisec);
void flash_lock();
void flash_unlock();
void flash_erase_sector(eSERTOR_t sector);
void flash_program_byte(void* address, uint8_t* buffer, uint8_t size);



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

	uint8_t msg[] = "xin chao STM32F4";
	flash_erase_sector(SECTOR_7);
	flash_program_byte((void*)SECTOR_7_BASE_ADDR, msg, sizeof(msg));

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
flash_program_byte(void* address, uint8_t* buffer, uint8_t size) {
	uint32_t volatile* const FLASH_SR   = (uint32_t*)(0x40023c00 + 0x0C);
	uint32_t volatile* const FLASH_CR   = (uint32_t*)(0x40023c00 + 0x10);

	flash_unlock();

	/*check BUSY bit*/
	while (((*FLASH_SR >> 16) & 1) == 1) {}
	/*SET programming mode*/
	*FLASH_CR |= (1 << 0);
	/*write data*/
	for (uint8_t i = 0; i < size; i++) {
		*((uint8_t*)(address)++) = buffer[i];
	}
	/*CLEAR programming mode*/
	*FLASH_CR &= ~(1 << 0);
	/*check BUSY bit*/
	while (((*FLASH_SR >> 16) & 1) == 1) {}

	flash_lock();
}
