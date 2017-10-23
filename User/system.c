#include <stm32f4xx.h>



#define SYSTEM_INIT_ERROR_FLASH 		0x01
#define SYSTEM_INIT_ERROR_PLL			0x02
#define SYSTEM_INIT_ERROR_CLKSRC		0x04
#define SYSTEM_INIT_ERROR_HSI			0x08

uint32_t SystemCoreClock = 16000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};


void SystemInitError(uint8_t error_source);

void SystemInit()
{
	/*Enable power control clock*/
	RCC->APB1ENR |= RCC_APB1LPENR_PWRLPEN;

	/*Regulator output voltage scaling: scale 2*/

	PWR->CR |= PWR_CR_VOS_1;

	/* Wait until HSI ready */

	while ((RCC->CR & RCC_CR_HSIRDY) == 0);

	/* Store caliberation value */

	PWR->CR &= -(RCC_CR_PLLON);

	/* Wait until PLL ready */

	while ((RCC->CR & RCC_CR_PLLRDY) != 0);

	/*
	 * Configure main PLL
	 * HSI as clock input
	 * fvco = 336MHz
	 * fpllout = 84MHz
	 * fusb	= 48MHz
	 * PLLM = 16
	 * PLLN = 336
	 * PLLP = 4
	 * PLLQ = 7
	 */

	RCC->PLLCFGR = (uint32_t)((uint32_t)0x20000000) | (uint32_t)(16 << 0) | (uint32_t)(336 << 6) | RCC_PLLCFGR_PLLP_0 | (uint32_t)(7<<24);


	/* PLL ON */

	RCC->CR |= RCC_CR_PLLON;

	/*Wait until PLL is locked */

	while ((RCC->CR & RCC_CR_PLLRDY) == 0);


	/*
	*FLASH configuration block
	*enable instruction cache
	*enable prefetch
	*set latency to 2WS (3 CPU cycles)
	*/

	FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_2WS;

	/* check flash latency */

	if ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_2WS)
	{
		SystemInitError(SYSTEM_INIT_ERROR_FLASH);

	}

	/* Set clock source to PLL */

	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* check clock source */

	while ((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL);

	/*Set HCLK (AHB) prscaler (DIV1) */

	RCC->CFGR &= ~(RCC_CFGR_HPRE);

	/*Set APB1 Low speed prescaler (APB1) DIV2 */

	RCC->CFGR |= RCC_CFGR_HPRE;

	/* Set APB2 High speed srescaler (APB2) DIV1 */
 	
 	RCC->CFGR &= ~(RCC_CFGR_PPRE2);


}

void SystemInitError(uint8_t error_source)
{
	while(1);
}
