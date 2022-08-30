#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

void vectortable_move();
void tim_systick_init();
void sys_delay_ms(uint32_t time_milisec);
void adc_init();
void adc_dma_receive(void* adc_buffer);
void adc_start();
uint32_t adc_read();
double adc_temp_read();

uint32_t adc_dma_buffer[128];
double Temp;

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
	adc_init();
	adc_dma_receive((void*)adc_dma_buffer);
	adc_start();

	while (1) {
		//Temp = adc_temp_read();

		static double temp = 0, volt = 0;
		static uint16_t adc_val;
		adc_val = adc_read();
		volt = (float)adc_val * (3.3 / 4095);

		if (volt > 0.76) {
			temp = ((volt - 0.76) / 0.0025) + 25;
		}
		else {
			temp = 25.0;
		}
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
adc_dma_receive(void* adc_buffer) {
	__HAL_RCC_DMA2_CLK_ENABLE();

	/*-----------------------Rx DMA-----------------------*/
	uint32_t volatile *const ADC_DR  	  = (uint32_t *)(0x40004400 + 0x04);
	uint32_t volatile *const ADC_CR2   	  = (uint32_t *)(0x40012000 + 0x08);
	uint32_t volatile *const DMA2_S0CR    = (uint32_t *)(0x40026400 + 0x10 + (0x18 * 0));
	uint32_t volatile *const DMA2_S0NDTR  = (uint32_t *)(0x40026400 + 0x14 + (0x18 * 0));
	uint32_t volatile *const DMA2_S0PAR   = (uint32_t *)(0x40026400 + 0x18 + (0x18 * 0));
	uint32_t volatile *const DMA2_S0M0AR  = (uint32_t *)(0x40026400 + 0x1c + (0x18 * 0));

	*ADC_CR2 |= (1 << 8);						// Enable DMA for ADC
	*ADC_CR2 |= (1 << 9);						// Enable Continuous Request

	*DMA2_S0NDTR = sizeof(adc_buffer);			/*number of data*/
	*DMA2_S0PAR = (uint32_t)ADC_DR;				/*peripheral address*/
	*DMA2_S0M0AR = (uint32_t)adc_buffer;		/*memory address*/

	*DMA2_S0CR &= ~(0b111 << 25);				/*channel 6*/
	*DMA2_S0CR |= (1 << 8);						/*circular mode*/
	*DMA2_S0CR |= (1 << 10);					/*memory increment mode*/
	*DMA2_S0CR |= (1 << 0);						/*DMA stream enable*/
}

/*
 * \brief
 * \param[in]
 * \param[out]
 * \retval
 */
void
adc_init() {
	__HAL_RCC_ADC1_CLK_ENABLE();
	uint32_t volatile *const ADC_CCR   	 = (uint32_t *)(0x40012304 + 0x00);
	uint32_t volatile *const ADC_CR1   	 = (uint32_t *)(0x40012000 + 0x04);
	uint32_t volatile *const ADC_CR2   	 = (uint32_t *)(0x40012000 + 0x08);
	uint32_t volatile *const ADC_SMPR1   = (uint32_t *)(0x40012000 + 0x0c);

	*ADC_CCR |= (1 << 16);			// ADCCLK = Fclk / 4
	*ADC_CR1 &= ~(0b11 << 24);		// 12-bit resolution
	*ADC_SMPR1 |= (7 << 24);
	*ADC_CCR |= (1 << 23);			// wake up temp sensor and enable Vrefint
	*ADC_CR2 |= (1 << 0);   		// wake up to enable ADC1
}

/*
 * \brief
 * \param[in]
 * \param[out]
 * \retval
 */
void
adc_start() {
	uint32_t volatile *const ADC_SR   	 = (uint32_t *)(0x40012000 + 0x00);
	uint32_t volatile *const ADC_CR2   	 = (uint32_t *)(0x40012000 + 0x08);

	*ADC_SR = 0;        	// clear the status register
	*ADC_CR2 |= (1<<30);  	// start the conversion
}

/*
 * \brief
 * \param[in]
 * \param[out]
 * \retval
 */
uint32_t
adc_read() {
	uint16_t volatile *const ADC_DR   	 = (uint16_t *)(0x40012000 + 0x4c);
	return *ADC_DR;
}

/*
 * \brief
 * \param[in]
 * \param[out]
 * \retval
 */
double
adc_temp_read() {
	double temp = 0, volt = 0;
	uint16_t adc_val = adc_read();

	volt = (float)adc_val * (3.3 / 4095);

	if (volt > 0.76) {
		temp = ((volt - 0.76) / 0.0025) + 25;
	}
	else {
		temp = 25.0;
	}

	return temp;
}
