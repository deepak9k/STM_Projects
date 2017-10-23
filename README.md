# STM_Projects

New Usart Lib created by Deepak kumar

stm_uart.h and stm_uart.c

It works for Transmission of the  data from the USART2 

----------------For using USART-----------------------
--location of Include file         STM_Projects/Drivers/Device/New_Lib/Include
--source file 				       STM_Projects/Drivers/Device/New_Lib/src


(1) define the USART varible in main file of type    
				--var        Usart_Handle_Type <name of handle>
				--var 		 uint8_t  <data var>     

(2)			function declaration   and initialization

Example:- 




void USART_Config()
{ 
	usart2.Instance 			= 		USART2;			
	usart2.init.Data_Word_length 		= 		DATA_WORD_8_BIT;	
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

(3)			fuction for transmission 

		
		Stm_Usart_Receive(&usart2, &data, sizeof(data));                 *****for receiving  @note it will block the process due to 																								while loop
		Stm_Usart_Transmitt(&usart2, &data, sizeof(data));				 *****for sending    	
 


 		Use handle var in place of "usart2"
 		Use data   var in place of "data"


----------------------------------------------------------------------------------

for more information look into "main.c" and "stm_uart.h" files


