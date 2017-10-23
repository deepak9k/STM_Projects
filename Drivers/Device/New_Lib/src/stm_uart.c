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

#include "stm32f4xx.h"
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

 	/* Enable GPIOA clock */

	

 	/* Enable the USART CLOCK */
 	_STM_USART2_CLKENABLE();
  
  _STM_GPIOA_CLKENABLE();

 	/* Disable the peripheral */
	_STM_USART_DISABLE(usart);

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

    /*-----------------Setting the USART BRR Register----------------------------*/

    /* For now it is available for USART 2 */

    if(usart->init.Oversampling == OVERSAMPLING_BY_8 )
    {
    	usart->Instance->BRR = STM_USART_BRR(Stm_Get_Pclk1_Freq(), usart->init.Baud_Rate,0x01);
    }
    else
    {
    	usart->Instance->BRR = STM_USART_BRR(Stm_Get_Pclk1_Freq(), usart->init.Baud_Rate,0x00);
    }
  
  	/* In asynchronous mode, the following bits must be kept cleared: 
     - LINEN and CLKEN bits in the USART_CR2 register,
     - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
  		CLEAR_BIT(usart->Instance->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
  		CLEAR_BIT(usart->Instance->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

  	/* Enable the USART */	
  	_STM_USART_ENABLE(usart);

  	return Success;

 }

 /*
  * @about Usart transmitt function  for sending the data only for the 8 frame;
  *
  *	@param      -Usart_Handle_Type for  the handle of usart that contain configuration information
  *				-pData  is a data address which going to be transmitted
  *				-size  is the size of the data
  *
  * @retvalue   -status of the transmitter function
  */


 
 USART_Status Stm_Usart_Transmitt(Usart_Handle_Type *usart, uint8_t *pData, uint16_t size)
 {
 	uint16_t temp=size;

 	if((size == 0)||(pData == NULL))
 	{
 		return Error;
 	}
 	usart->pTxBuffer = pData;
 	usart->TxSize = size;

 	while(temp>0U)
 	{
 		temp--;
 		if(FlagCheck(usart, FLAG_TXE, HIGH) != Success)
 		{
 			return Error;
 		}

 		usart->Instance->DR = (*pData++ &(uint8_t)0xFF);

	}
	if(FlagCheck(usart, FLAG_TC, HIGH) != Success)
	{
		return Error;
	}

	return Success;
 }


 /*
  * @about Usart Receiver function  for Receiving  the data only for the 8 frame;
  *
  *	@param      -Usart_Handle_Type for  the handle of usart that contain configuration information
  *				-pData  is a data address which going to be received
  *			-size  is the size of the data
  *
  * @retvalue   -status of the transmitter function
  */

 USART_Status Stm_Usart_Receive(Usart_Handle_Type *usart, uint8_t *pData, uint16_t size)
 {
 	uint16_t temp=size;

 	if((size == 0)||(pData == NULL))
 	{
 		return Error;
 	}
 	usart->pRxBuffer = pData;
 	usart->RxSize = size;

 	while(temp>0U)
 	{
 		temp--;
 		if(FlagCheck(usart, FLAG_RXNE, HIGH) != Success)
 		{
 			return Error;
 		}
 		 if(usart->init.Parity_Bit == PARITY_NONE)
        {
          *pData++ = (uint8_t)(usart->Instance->DR & (uint8_t)0x00FF);
        }
        else
        {
          *pData++ = (uint8_t)(usart->Instance->DR & (uint8_t)0x007F);
        }

 	

	}

	return Success;
 }


 /* Private Function-----------------------------------------------------------------*/


 uint32_t Stm_Get_Hclk_Freq()
 {
 	return SystemCoreClock;
 }

 uint32_t Stm_Get_Pclk1_Freq()
 {
 	return (Stm_Get_Hclk_Freq() >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> POSITION_VAL(RCC_CFGR_PPRE1)]);
 }

 uint32_t Stm_Get_Pclk2_Freq()
 {
 	return (Stm_Get_Hclk_Freq() >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2)>> POSITION_VAL(RCC_CFGR_PPRE2)]);
 }



 USART_Status FlagCheck(Usart_Handle_Type *usart, uint32_t flag, Flag_Status status)
 {
 	Flag_Status temp=NONE;

 	while(temp != status)
 	{
 		if(_STM_READ_FLAG(usart ,flag ))
 		{
 			temp=HIGH;
 		}
 		else
 		{
 			temp=LOW;
 		}
 	}

 	return Success;
 }




























