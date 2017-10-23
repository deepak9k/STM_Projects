/*
*******************************************************************
* @file 			stm_uart.h
* @author 			Deepak Kumar
* @version 			New version 
* @Date				20-Oct-2017
* @brief			Header file for the UART communication
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

/*Defining the Header file------------------------------------------- */

#ifndef __STM_UART_H
#define __STM_UART_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes file ----------------------------------------------------*/

//#include "stm_uart_conf.h"
 #include "core_cm4.h"


/* Exported Types ----------------------------------------------------*/



/*   
 *  @about   Init Uart struture   
 */

typedef struct 
{
	uint32_t Data_Word_length;      /* <This parameter specify the word length 
										of the data frame > */

	uint32_t Stop_Bit;				/* <This parameter indicate about the stop 
										stop bit used in the frame> */

	uint32_t Baud_Rate;				/* <This field indicating the Baud Rate of the 
										of the frame It can be fractional generated 
										by the clock  USARTDIV = DIV_Mantissa + (DIV_Fraction /8x (2-OVER8))> */
    
    uint32_t Parity_Bit;            /* <This field indicate the partiy bit enable> */

    uint32_t Usart_Mode;            /* <This field define the mode of the Usart operation> */

    uint32_t Oversampling;			/* <This field is used to select the oversampling by 
    										-  8
    										-  16  >*/

}Uart_Init_Type;



/*
 * @about Error
 */

typedef enum 
{
	Success,                    
	Error
}USART_Status;


/*
 * @about Flag status
 */

typedef enum 
{
	LOW,
	HIGH,
	NONE
}Flag_Status;
/*
 * @about uart handle
 */

typedef struct 
{
	USART_TypeDef 				*Instance;                      	/* <USART register base address> */

	Uart_Init_Type 				init;  							/* <USART Init Type Parameter>   */

	uint8_t    				*pTxBuffer;						/* <pointer to the data to be transferred> */

	uint16_t 					TxSize;							/* <size of the data to be transferred>  */

	uint8_t 					*pRxBuffer;						/* <pointer to the data to be received>	*/

	uint16_t 					RxSize;							/* <Size of the data to be received> */	 


}Usart_Handle_Type;


/* Exported Constants --------------------------------------------------*/



/*
 * @about stop bits selection
 */

#define STOP_BIT_ONE  							0x00000000U								/* <Stop bit 1> */
#define STOP_BIT_TWO 							((uint32_t) USART_CR2_STOP_1 )	        /* <stop bit 2> */
#define STOP_BIT_HALF   						((uint32_t) USART_CR2_STOP_0 )			/* <stop bit 0.5> */
#define STOP_BIT_ONE_HALF                       ((uint32_t) USART_CR2_STOP )	        /* <stop bit 1.5> */

/*
 * @about type of Error 
 */

#define ERROR_NONE 								0x00000000U								/* <NO Error> */
#define ERROR_OVERRUN							((uint32_t) USART_SR_ORE )				/* <Overrun Error> */
#define ERROR_NOISE_DETECTION					((uint32_t) USART_SR_NF )				/* <Noise Error> */
#define ERROR_FRAME								((uint32_t) USART_SR_FE )				/* <Frame Error> */
#define ERROR_PARITY							((uint32_t) USART_SR_PE )				/* <Parity Error> */

/*
 * @about type of Mode used in Usart
 */


#define UART_MODE_RX                        ((uint32_t)USART_CR1_RE)
#define UART_MODE_TX                        ((uint32_t)USART_CR1_TE)
#define UART_MODE_TX_RX                     ((uint32_t)(USART_CR1_TE |USART_CR1_RE))


/*
 * @about parity 
 */

#define PARITY_NONE							0x00000000U
#define PARITY_EVEN							((uint32_t) USART_CR1_PCE )
#define PARITY_ODD							((uint32_t)(USART_CR1_PCE |USART_CR1_PS))


/*
 * @about DATA WORD Length 
 */

#define DATA_WORD_8_BIT						0x00000000U
#define DATA_WORD_9_BIT						((uint32_t)USART_CR1_M )

/*
 * @about Uart state
 */

#define UART_ENABLE 						((uint32_t) USART_CR1_UE ) 						
#define UART_DISABLE							0x00000000U

/*
 * @about Wakeup Method
 */

#define WAKEUP_IDLE_LINE					0x00000000U
#define WAKEUP_ADD_MARK						((uint32_t) USART_CR1_WAKE ) 


/*
 * @about oversampling
 */

#define OVERSAMPLING_BY_16					0x00000000U
#define OVERSAMPLING_BY_8					((uint32_t) USART_CR1_OVER8 )

/*
 * @about Interrupt 
 */
#define INTERRUPT_PE						((uint32_t)USART_CR1_PEIE )
#define INTERRUPT_TXE						((uint32_t)USART_CR1_TXEIE )	
#define INTERRUPT_TCI						((uint32_t)USART_CR1_TCIIE )
#define INTERRUPT_RXNE						((uint32_t)USART_CR1_RXNEIE )
#define INTERRUPT_IDLE						((uint32_t)USART_CR1_IDLEIE )

/*
 * @about Clock Enable
 */

#define CLOCK_DISABLE						0x00000000U
#define CLOCK_ENABLE						((uint32_t)USART_CR2_CLKEN )

/*
 * @about Error Intrrupt
 */
#define ERROR_INTERRUPT_DISABLE				0x00000000U
#define ERROR_INTERRUPT_ENABLE				((uint32_t)USART_CR3_EIE)

/*
 * @about USART flag
 */
#define FLAG_TXE   							((uint32_t)USART_SR_TXE)	
#define FLAG_TC								((uint32_t)USART_SR_TC)
#define FLAG_RXNE							((uint32_t)USART_SR_RXNE)
#define FLAG_IDLE							((uint32_t)USART_SR_IDLE)



/*
 * @about NULL
 */

#define NULL						       ((void*)0)
/* Exported Macros ---------------------------------------------------------*/

/*
 * @about Get Flag status and Clear Flag
 *				
 * @param  		__HANDLE__ :- It is handle used for the USART
 *
 *
 * @param 		__FLAG__ :- Flag which going to be read 
 *						    -TXE Flag is read and clear by Clear function 
 *							-TC Flag is read  and clear by read from USART_SR followed by write to USART_DATA or by clear function
 *						    -RXNE Flag is read and clear by read or by the write '0' to it
 *							-IDLE Flag is read and clear by reading from USART_SR followed by the USART_DATA
 *							-ERROR_OVERRUN flag is read  and clear by reading from USART_SR followed by the USART_DATA
 *							-ERROR_FRAME  flag is read and clear by reading  from USART_SR followed by the USART_DATA
 *							-ERROR_PARITY flag is read and clear a read from the status register followed by a read or write access to the
 *										USART_DR data register. The software must wait for the RXNE flag to be set before
 *										clearing the PE bit.
 *							-ERROR_NOISE_DETETION flag is read and clear by from USART_SR followed by the USART_DATA
 * 	@retval		  Return the flag status by reading function
 */


#define _STM_READ_FLAG(__HANDLE__,__FLAG__)		(((__HANDLE__)->Instance->SR & (__FLAG__)) == (__FLAG__))



#define _STM_CLEAR_FLAG(__HANDLE__,__FLAG__)	(((__HANDLE__)->Instance->SR) = ~ (__FLAG__))


#define _STM_CLEAR_PENDING_FLAG(__HANDLE__)				\
do 														\
	{													\
		__IO uint32_t reg = 0x00U;						\
		reg = (__HANDLE__)->Instance->SR;				\
		reg = (__HANDLE__)->Instance->DR;				\
		UNUSED(reg);									\
	} while (0U)										



/*
 *	@about USART interrupt Enable
 *
 *	@param 		__HANDLE__ :- Handle for the usart
 *
 *  @param 		__INTERRUPT__ :- Type of interrupt to be enable
 *							-INTERRUPT_PE	parity error					
 *							-INTERRUPT_TXE	Transmission data register empty					
 *							-INTERRUPT_TCI	Transmission complete			
 *							-INTERRUPT_RXNE	Read data register not empty 					
 *							-INTERRUPT_IDLE	Idle line 
 *	@retval		none
 */

#define INTERRUPT_MASK	  0x0000FFFFU	

#define _STM_INTERRUPT_ENABLE(__HANDLE__, __INTERRUPT__)		((__HANDLE__)->Instance->CR1 |= (__INTERRUPT__) & INTERRUPT_MASK)


/*
 *	@about USART interrupt Disable
 *
 *	@param 		__HANDLE__ :- Handle for the usart
 *
 *  @param 		__INTERRUPT__ :- Type of interrupt to be Disable
 *							-INTERRUPT_PE	parity error					
 *							-INTERRUPT_TXE	Transmission data register empty					
 *							-INTERRUPT_TCI	Transmission complete			
 *							-INTERRUPT_RXNE	Read data register not empty 					
 *							-INTERRUPT_IDLE	Idle line 
 *	@retval		none
 */

#define _STM_INTERRUPT_DISABLE(__HANDLE__, __INTERRUPT__)		((__HANDLE__)->Instance->CR1 &= ~(__INTERRUPT__) & INTERRUPT_MASK)


/*
 * @about flush the data register
 */

 #define _STM_FLUSH_DR(__HANDLE__)								((__HANDLE__)->Instance->DR)	


/*
 *	@about USART Enable of Disable
 */


#define _STM_USART_ENABLE(__HANDLE__)							((__HANDLE__)->Instance->CR1 |=  USART_CR1_UE)
#define _STM_USART_DISABLE(__HANDLE__)							((__HANDLE__)->Instance->CR1 &= ~ USART_CR1_UE)



/* Exported Function -----------------------------------------------------*/


/*
 *	@about Function for initialization
 */

USART_Status Stm_Usart_Init(Usart_Handle_Type *usart);


/*
 *	@about Function for Transmitt and Receive
 */

USART_Status Stm_Usart_Transmitt(Usart_Handle_Type *usart, uint8_t *pData, uint16_t size);

USART_Status Stm_Usart_Receive(Usart_Handle_Type *usart, uint8_t *pData, uint16_t size);



/* Private constants-----------------------------------------------------------*/








#define STM_USART_DIV(__PCLK__, __BAUD__, OVER8)       		((__PCLK__)*1000U/(8U*(2-OVER8)*__BAUD__))

#define STM_USART_MANTDIV(__PCLK__, __BAUD__, OVER8) 		((STM_USART_DIV(__PCLK__, __BAUD__, OVER8))/1000U)			

#define STM_USART_FRAQDIV(__PCLK__, __BAUD__, OVER8)		((STM_USART_DIV(__PCLK__, __BAUD__, OVER8)-STM_USART_MANTDIV(__PCLK__, __BAUD__, OVER8)*1000U)/8U*(2-OVER8))

#define STM_USART_BRR(__PCLK__, __BAUD__, OVER8)			 (uint32_t)(OVER8)?((STM_USART_MANTDIV(__PCLK__, __BAUD__, OVER8)<<4)+   \
																	    (STM_USART_FRAQDIV(__PCLK__, __BAUD__, OVER8)&0xF0U)+ \
																	    (STM_USART_FRAQDIV(__PCLK__, __BAUD__, OVER8)&0x0FU)) :  \
																																	\
																		((STM_USART_MANTDIV(__PCLK__, __BAUD__, OVER8)<<4)+   \
																	    ((STM_USART_FRAQDIV(__PCLK__, __BAUD__, OVER8)&0xF8U)<<1U)+ \
																	    (STM_USART_FRAQDIV(__PCLK__, __BAUD__, OVER8)&0x07U))

/* Clock Enable Def for USART2*/

#define _STM_USART2_CLKENABLE()		 					do { \
                                        						uint32_t reg = 0x00U; \
                                        						SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);\
                                        						/* Delay after an RCC peripheral clock enabling */ \
                                        						reg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);\
                                          					} while(0U)

/* Enable the clock of the GPIOA */
#define _STM_GPIOA_CLKENABLE()							do {	\
                                          					 uint32_t reg =0x00U;	\
                                          					 SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);  \
                                          					 /* Delay after an RCC peripheral clock enabling */ \
                                          					 reg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN); \
                                          					}while(0U)


/* Private Function -----------------------------------------------------------*/


uint32_t Stm_Get_Hclk_Freq();
uint32_t Stm_Get_Pclk1_Freq();
uint32_t Stm_get_Pclk2_Freq();

USART_Status FlagCheck(Usart_Handle_Type *usart, uint32_t flag, Flag_Status status);

#endif