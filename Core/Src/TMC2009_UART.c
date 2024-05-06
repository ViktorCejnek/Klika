/*
 * TMC2009_UART.c
 *
 *  Created on: May 6, 2024
 *      Author: Viktor Cejnek
 */

#include "TMC2009_UART.h"
#include "stm32wbxx_hal.h"

void test(void){
	uint8_t buffer[9] = {0};
	m_UART_rFrame[1] = 0;
	m_UART_rFrame[2] = REG_IFCNT;

	uint8_t crc = 0;
	for(int i = 0; i < 3; i++) {
		uint8_t byte = m_UART_rFrame[i];
		for(int j = 0; j < 8; j++) {
			if((crc >> 7) ^ (byte & 0x01)) {
				crc = ((crc << 1) ^ 0x07) & 0xFF;
			} else {
				crc = (crc << 1) & 0xFF;
			}
			byte = byte >> 1;
		}
	}
	m_UART_rFrame[3] = crc;

	HAL_UART_Transmit(TMC_UART, m_UART_rFrame, 4, m_UART_communication_timeout);

	HAL_UART_Receive(TMC_UART, buffer, 9, m_UART_communication_timeout);
}
