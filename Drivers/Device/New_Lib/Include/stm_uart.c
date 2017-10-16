/*
*******************************************************************
* @file 			stm_uart.c
* @author 			Deepak Kumar
* @version 			New version 
* @Date				20-Oct-2017
* @brief			Source file for the USART function 
*******************************************************************
*==================================================================
*					USING THIS DRIVER
*==================================================================
*
*  (1) Declare Usart_Handle_Type Handle
*	
*  (2) Initialize The USART 
*       (a) Enable the USART interface clock
*		(b) Enable the pin configuration 
*       (C) Enable the interrupt function if you using the interrupt
*  (3) Set the baud rate, Word length, stop bit, parity bit, mode       
*      and oversampling
*  (4) For UART mode init the UART by calling the function
*
* Two type of operation mode available
* 
*  ***Polling mode*** 
* ===================================================================
*	--Sending data in the blocking mode using Stm_Usart_Transmitt()
*   --Receiving data in the blocking mode using Stm_Usart_Receive()
*
*  ***Interrupt mode***
* ===================================================================
*    --Sending data in the non blocking mode using Stm_Usart_Transmitt_IT()
*    ----Receiving data in the non blocking mode using Stm_Usart_Receive_IT()
*
*
*******************************************************************
*  @attention
*    
*
*       		COPYRIGHT(c) 2017 Deepak Kumar
*
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* *********************************************************************
*/



/* Include ---------------------------------------------------------------*/

#include "stm_uart.h"

/* Exported Function ------------------------------------------------------*/



/*
 * =========================================================================
 * 				***Initialization of UART***
 * =========================================================================
 *    
 *   (1) Set the following parameter for the UART init
 *         -baud rate
 *		   -Word length
 *         -Oversampling
 *         -mode
 *         -parity
 *         -stop bit 
 */



/*
 * @about  		Initialize the USART according to the parameter define in the 
 *				Uart_Init_Type
 * @param 		usart handle
 *
 * @retval		USART_Status of the function   
 */

 USART_Status Stm_Usart_Init(Usart_Handle_Type *usart)
 {

 	uint32_t reg;
 	/* check the usart handle */
 	if(usart == NULL)
 	{
 		return Error;
 	}
 	/* Disable the peripheral */
	_STM_UART_DISABLE(usart);

 	/*-------------- USART CR2 Configuration----------------------------- */
   
      reg = usart->Instance->CR2;

    /* clear the stop bit */
	reg &= (uint32_t)~STOP_BIT_ONE_HALF;

	/* set the stop bit */
	reg |= (uint32_t)usart->init.Stop_Bit;

	/* Write the register */
	WRITE_REG(usart->Instance->CR2, (uint32_t)reg );

	/*-------------------USART CR1 Configuration---------------------------*/

    reg = usart->Instance->CR1;

    /* clear the Word lenght bit, Parity, oversampling and mode */

    reg &= (uint32_t)~(PARITY_ODD | DATA_WORD_9_BIT | UART_MODE_TX_RX | OVERSAMPLING_BY_8);

    /* set the value of reg according to the usart parameter */
    reg |= (uint32_t)(usart->init.Data_Word_length | usart->init.Parity_Bit | usart->init.Usart_Mode | usart->init.Oversampling);

    /* Write the reg value */
    WRITE_REG(usart->Instance->CR1, (uint32_t)reg);
  
  	


 }