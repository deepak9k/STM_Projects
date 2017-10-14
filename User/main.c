#include "stm32f4xx.h"

#define LED_PIN 		5
#define LED_ON() GPIOA->BSRR |= (1 << 5)
#define LED_OFF() GPIOA->BSRR |= (1 << 5)

int main()
{
	/* Enable GPIOA clock */

	 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	  /*configure GPIOA pin 5 as output */

	 GPIOA->MODER |= (1 << (LED_PIN << 1));

	 GPIOA->OSPEEDR |= (3 << (LED_PIN << 1));

	 LED_ON();

	 return 0;

}