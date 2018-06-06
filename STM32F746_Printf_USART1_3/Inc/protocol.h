
#ifndef __PROTOCOL_H
#define __PROTOCOL_H
  /* Includes ------------------------------------------------------------------*/
#include "stm32f7xx.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define UART1_INP_BUF_SIZE 0x0080U     /* Input buffer size */

#define PROTOCOL_UART1_EXT_RX_LENGTH	12
#define UART1_RING_BUFFER_LENGTH	8

#define PROTOCOL_UART1_LENGTH		12

#define ON_ERROR        0x01U
#define ON_FULL_RX      0x02U
#define ON_RX_CHAR      0x04U
#define ON_IDLE_CHAR    0x08U
#define ON_RX_CHAR_EXT  0x10U
#define ERR_OK           0U            /* OK */
#define ERR_RXEMPTY      10U           /* No data in receiver. */
#define TRUE 1

#define PROTOCOL_UART1_IDX_STX		0
#define PROTOCOL_UART1_IDX_CMD		1
#define PROTOCOL_UART1_IDX_DATA	2u
#define PROTOCOL_UART1_IDX_DATA1	3u
#define PROTOCOL_UART1_IDX_DATA2	4u
#define PROTOCOL_UART1_IDX_DATA3	5u
#define PROTOCOL_UART1_IDX_DATA4	6u
#define PROTOCOL_UART1_IDX_DATA5	7u
#define PROTOCOL_UART1_IDX_DATA6	8u
#define PROTOCOL_UART1_IDX_DATA7	9u
#define PROTOCOL_UART1_IDX_ETX		10u
#define PROTOCOL_UART1_IDX_CHK		11u

#define PROTOCOL_UART1_CODE_STX		0xFEu
#define PROTOCOL_UART1_CODE_ETX		0xEFu

typedef struct{
	// INCLUDE DATA
	uint8_t	pData[PROTOCOL_UART1_EXT_RX_LENGTH];
	// EXT DATA
	uint8_t 	ucBufFull;
	uint8_t 	ucBufCount;
	uint8_t 	ucLength;
} ST_UART1_DATA_RX_PACKET;

volatile uint8_t SerFlag;          /* Flags for serial communication */
                                       /* Bit 0 - Overrun error */
                                       /* Bit 1 - Common error of RX */
                                       /* Bit 2 - Char in RX buffer */
                                       /* Bit 3 - Interrupt is in progress */
                                       /* Bit 4 - Full RX buffer */
                                       /* Bit 5 - Full TX buffer */

uint16_t UART1_InpLen;                     /* Length of the input buffer content */
uint16_t InpIndxR;                  /* Index for reading from input buffer */
uint16_t InpIndxW;                  /* Index for writing to input buffer */
uint8_t InpBuffer[UART1_INP_BUF_SIZE]; /* Input buffer for SCI commmunication */

uint8_t uart1_ring_read_pointer;
uint8_t uart1_ring_write_pointer;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern ST_UART1_DATA_RX_PACKET s_Uart1RecvPacket[UART1_RING_BUFFER_LENGTH];

static uint8_t Uart1CheckSum(uint8_t *pData);
static uint8_t Uart1ValidCommand(uint8_t cmd);
void PTC_Uart1ClearBuffer(void);
uint8_t UART1_RecvChar(uint8_t *Chr);
void PTC_Uart1ReceivePacket(void);

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __PROTOCOL_H */
































