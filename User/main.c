#include "stm32f4xx.h"
#include "stm_uart.h"
#include <stdio.h>

/*Def variable */
Usart_Handle_Type usart2;

uint8_t data;

void USART_Config();


int main()
{
	

	 /* Initialize the USART */

	USART_Config();

	while(1)
	{
		Stm_Usart_Receive(&usart2, &data, sizeof(data));
		Stm_Usart_Transmitt(&usart2, &data, sizeof(data));

		//pritnf("data received: %c",data);
	}


	 return 0;

}

void USART_Config()
{
	usart2.Instance 				= 		USART2;
	usart2.init.Data_Word_length 	= 		DATA_WORD_8_BIT;			
	usart2.init.Stop_Bit			=		STOP_BIT_ONE;
	usart2.init.Baud_Rate			=		9600;
	usart2.init.Parity_Bit			=		PARITY_NONE;
	usart2.init.Usart_Mode			=		UART_MODE_TX_RX;
	usart2.init.Oversampling		=		OVERSAMPLING_BY_16;


	if(Stm_Usart_Init(&usart2) != Success)
	{
		/* add code for error handling*/
	}

}