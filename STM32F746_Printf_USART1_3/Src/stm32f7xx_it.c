/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#include "stm32f7xx_it.h"

//#include "protocol.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

#if 1	//LJH
uint8_t Rx_data[10];
/////////////////
extern volatile uint8_t SerFlag;          /* Flags for serial communication */
                                       /* Bit 0 - Overrun error */
                                       /* Bit 1 - Common error of RX */
                                       /* Bit 2 - Char in RX buffer */
                                       /* Bit 3 - Interrupt is in progress */
                                       /* Bit 4 - Full RX buffer */
                                       /* Bit 5 - Full TX buffer */

extern uint16_t UART1_InpLen;                     /* Length of the input buffer content */
extern uint16_t InpIndxR;                  /* Index for reading from input buffer */
extern uint16_t InpIndxW;                  /* Index for writing to input buffer */
extern uint8_t InpBuffer[UART1_INP_BUF_SIZE]; /* Input buffer for SCI commmunication */

extern uint8_t uart1_ring_read_pointer;
extern uint8_t uart1_ring_write_pointer;
#endif

/******************************************************************************/
/*            Cortex-M7 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles USART3 global interrupt.
*/
#if 1		//LJH
/*
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
	 	if(huart->Instance == USART1){
			printf("USART1=%c",USART1->RDR);
	 	}

		HAL_UART_Receive_IT(&huart1, (uint8_t*)Rx_data,1);
	}
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  #if 0	//LJH, original
  	HAL_UART_IRQHandler(&huart1);
#else		//LJH
		uint32_t tmp_flag = __HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE);
 		uint32_t tmp_it_source = __HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_RXNE);
		
  		if((tmp_flag != RESET) && (tmp_it_source != RESET)) {

			printf("USART1=%c\r\n",USART1->RDR);
			//host_rxb.data[host_rxb.head] = USART6->DR;
		        //host_rxb.head++;
	        	//if (host_rxb.head >= HOST_RXBUFF_SIZE) 
			//host_rxb.head = 0;          
			//systick_host = 0;
			//__HAL_UART_CLEAR_PEFLAG(&huart1);
			//__HAL_UART_CLEAR_IT(&huart1, UART_FLAG_RXNE);
			//__HAL_UART_CLEAR_IT(&huart1, UART_IT_RXNE);

	        	//USART_ClearITPendingBit(USART3, USART_IT_RXNE);	
	        	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
		}
		else 
		{
		}	
		HAL_UART_IRQHandler(&huart1);
  #endif
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  #if 0	//LJH, original
  	HAL_UART_IRQHandler(&huart3);
#else		//LJH
	uint32_t tmp_flag = __HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE);
 		uint32_t tmp_it_source = __HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_RXNE);
		uint8_t OnFlags = 0x00U;                /* Temporary variable for flags */
		uint8_t Data = USART3->RDR;         /* Read data from the receiver into temporary variable for data */
  
  		if((tmp_flag != RESET) && (tmp_it_source != RESET)) {

			//printf("USART3=%c\r\n",USART3->RDR);
		       
			//////////////////////////////////////////
			if (UART1_InpLen < UART1_INP_BUF_SIZE)
			{ /* Is number of bytes in the receive buffer lower than size of buffer? */
				UART1_InpLen++;                    /* Increse number of chars in the receive buffer */
				InpBuffer[InpIndxW] = Data;        /* Save received char to the receive buffer */
				InpIndxW = (uint16_t)((InpIndxW + 1U) & (UART1_INP_BUF_SIZE - 1U)); /* Update index */
				OnFlags |= ON_RX_CHAR;             /* Set flag "OnRXChar" */
				if (UART1_InpLen== UART1_INP_BUF_SIZE)
				{ /* Is number of bytes in the receive buffer equal as a size of buffer? */
					OnFlags |= ON_FULL_RX;           /* If yes then set flag "OnFullRxBuff" */
				}
			}
			else
			{
				//SerFlag |= FULL_RX;                /* If yes then set flag buffer overflow */
				OnFlags |= ON_ERROR;               /* Set flag "OnError" */
			}
			
			if (OnFlags & ON_RX_CHAR)
			{      /* Is OnRxChar flag set? */
				//EVENT_UART1_OnRxChar();        /* If yes then invoke user event */
				PTC_Uart1ReceivePacket();
			}
			if (OnFlags & ON_FULL_RX)
			{      /* Is OnFullRxBuf flag set? */
				//EVENT_UART1_OnFullRxBuf();           /* If yes then invoke user event */
			}
			
			//////////////////////////////////////////
	        	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
		}
		else 
		{
			//USART_ClearITPendingBit(USART3, USART_IT_ERR);    
		}	
		HAL_UART_IRQHandler(&huart3);
  #endif
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

#endif
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
