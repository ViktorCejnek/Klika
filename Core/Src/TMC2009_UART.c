/*
 * TMC2009_UART.c
 *
 *  Created on: May 6, 2024
 *      Author: Viktor Cejnek
 */

#include "TMC2009_UART.h"
#include "stm32wbxx_hal.h"

UART_HandleTypeDef *TMC_UART;

uint8_t m_UART_rFrame[4] = {0x05, 0x0, 0, 0};
uint8_t m_UART_wFrame[8] = {0xA0, 0, 0, 0 , 0, 0, 0, 0};
uint16_t m_UART_communication_pause = 52083; // int(500/baudrate*1000000)
uint16_t m_UART_communication_timeout = 2083; // int(20000/baudrate*1000)

void test(void){
	uint8_t buffer[9] = {0};
	//m_UART_rFrame[2] = 0b00001100;

	uint8_t crc = 0;
	uint8_t datagramLength = 3;
	uint8_t currentByte;
	/*for(int i = 0; i < 3; i++) {
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
	m_UART_rFrame[3] = crc;*/

	for (int i=0; i<(datagramLength-1); i++) { // Execute for all bytes of a message
		currentByte = m_UART_rFrame[i]; // Retrieve a byte to be sent from Array
		for (int j=0; j<8; j++) {
			if ((crc >> 7) ^ (currentByte&0x01)) // update CRC based result of XOR operation
			{
				crc = (crc << 1) ^ 0x07;
			}
			else
			{
				crc = (crc << 1);
			}
			currentByte = currentByte >> 1;
		} // for CRC bit
	} // for message byte

	m_UART_rFrame[3] = crc;

	HAL_UART_Transmit(TMC_UART, m_UART_rFrame, 4, m_UART_communication_timeout);

	HAL_UART_Receive(TMC_UART, buffer, 9, m_UART_communication_timeout);
}
