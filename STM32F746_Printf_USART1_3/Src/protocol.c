#include "protocol.h"

ST_UART1_DATA_RX_PACKET s_Uart1RecvPacket[UART1_RING_BUFFER_LENGTH];
//extern uint8_t uart1_ring_read_pointer;
//extern uint8_t uart1_ring_write_pointer;

static uint8_t Uart1CheckSum(uint8_t *pData)
{
	uint8_t i, sum = 0;

	for(i = 0; i < PROTOCOL_UART1_LENGTH - 1; i++) sum += pData[i];

	return(sum);
}

static uint8_t Uart1ValidCommand(uint8_t cmd)
{
	switch(cmd)
	{
		//case PROTOCOL_UART1_MSG_SET_AV2_MODE:	/* CPU -> MCU */
		//	return(TRUE);
		//	break;

		default :
			return(TRUE);	//return(FALSE);
			break;
	}
}

void PTC_Uart1ClearBuffer(void)
{
	uint8_t i, j;

	for(j = 0; j < UART1_RING_BUFFER_LENGTH; j++)
	{
		for(i = 0; i < PROTOCOL_UART1_EXT_RX_LENGTH; i++) s_Uart1RecvPacket[j].pData[i] = 0;
		s_Uart1RecvPacket[j].ucBufFull = 0;
		s_Uart1RecvPacket[j].ucBufCount = 0;
		s_Uart1RecvPacket[j].ucLength = 0;
	}
/*
	for(i = 0; i < PROTOCOL_UART1_EXT_LENGTH; i++) s_Uart1SendPacket.pData[i] = 0;
	s_Uart1SendPacket.ucBufFull = 0;
	s_Uart1SendPacket.ucBufCount = 0;
	s_Uart1SendPacket.ucLength = 0;
*/
	uart1_ring_read_pointer = uart1_ring_write_pointer = 0;
}

uint8_t UART1_RecvChar(uint8_t *Chr)
{
	uint8_t Result = ERR_OK;                /* Prepare default error code */
	if (UART1_InpLen > 0U)
	{             /* Is number of received chars greater than 0? */
		//EnterCritical();                   /* Save the PS register */
		UART1_InpLen--;                    /* Decrease number of received chars */
		*Chr = InpBuffer[InpIndxR];        /* Received char */
		InpIndxR = (uint16_t)((InpIndxR + 1U) & (UART1_INP_BUF_SIZE - 1U)); /* Update index */
		//Result = (uint8_t)((SerFlag & (OVERRUN_ERR|COMMON_ERR|FULL_RX)) ? ERR_COMMON : ERR_OK);
		//SerFlag &= (uint8_t)(~(uint8_t)(OVERRUN_ERR|COMMON_ERR|FULL_RX|CHAR_IN_RX)); /* Clear all errors in the status variable */
		//ExitCritical();                    /* Restore the PS register */
	}
	else
	{
		return ERR_RXEMPTY;                /* Receiver is empty */
	}
	return Result;                       /* Return error code */
}

void PTC_Uart1ReceivePacket(void)
{
	uint8_t ucData;
	uint8_t checksum = 0;
	uint8_t prev_uart1_ring_write_pointer = 0;

	while (UART1_RecvChar(&ucData) != ERR_RXEMPTY)
	{
		{
			if(s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount == PROTOCOL_UART1_IDX_STX)
			{
					if(ucData == PROTOCOL_UART1_CODE_STX)
					{
						s_Uart1RecvPacket[uart1_ring_write_pointer].pData[s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount] = ucData;
						s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount++;
					}
					else
					{
						s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount = 0;
					}
			}
			else
			if(s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount == PROTOCOL_UART1_IDX_CMD)
			{
					if(Uart1ValidCommand(ucData) == TRUE)
					{
						s_Uart1RecvPacket[uart1_ring_write_pointer].pData[s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount] = ucData;
						s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount++;
					}
					else
					{
						s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount = 0;
					}
			}
			else
			if(	(s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount > PROTOCOL_UART1_IDX_CMD) &&
				(s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount < PROTOCOL_UART1_IDX_ETX))
			{
					s_Uart1RecvPacket[uart1_ring_write_pointer].pData[s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount] = ucData;
					s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount++;
			}
			else
			if(s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount == PROTOCOL_UART1_IDX_ETX)
			{
					if(ucData == PROTOCOL_UART1_CODE_ETX)
					{
						s_Uart1RecvPacket[uart1_ring_write_pointer].pData[s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount] = ucData;
						s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount++;
					}
					else
					{
						s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount = 0;
					}
			}
			else
			if(s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount == PROTOCOL_UART1_IDX_CHK)
			{
					//if(ucData == Uart1CheckSum(s_Uart1RecvPacket[uart1_ring_write_pointer].pData))
					if(1)
					{
						s_Uart1RecvPacket[uart1_ring_write_pointer].pData[s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount] = ucData;
						s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount = 0;
						/*
						if (s_Uart1RecvPacket[uart1_ring_write_pointer].pData[PROTOCOL_UART1_IDX_CMD] == PROTOCOL_UART1_MSG_WATCHDOG)
						{
							CTRL_NaviWatchdogTimerClear();
							return;
						}
						if (s_Uart1RecvPacket[uart1_ring_write_pointer].pData[PROTOCOL_UART1_IDX_CMD] == PROTOCOL_UART1_MSG_ACK)
						{
							//DEBUG_SendStr("RECV ACK[%d]", s_Uart1RecvPacket[uart1_ring_write_pointer].pData[PROTOCOL_UART1_IDX_DATA]);
							
							OSD_SET_UART1PACKET_TIMEOUT(OFF);
							return;
						}
						*/
						prev_uart1_ring_write_pointer = uart1_ring_write_pointer;
						s_Uart1RecvPacket[uart1_ring_write_pointer++].ucBufFull  = TRUE;

						if(uart1_ring_read_pointer == uart1_ring_write_pointer)
							//uart1_ring_read_pointer = prev_uart1_ring_write_pointer;
							PTC_Uart1ClearBuffer();
						
						if(uart1_ring_write_pointer == UART1_RING_BUFFER_LENGTH) 
							uart1_ring_write_pointer = 0;						
					}
					else
					{
						s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount = 0;
					}
			}
			else
			{
				s_Uart1RecvPacket[uart1_ring_write_pointer].ucBufCount = 0;
			}
		}
	}
}




























