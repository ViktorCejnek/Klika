/**
 * @file TMC2009_UART.c
 *
 *  Created on: May 6, 2024
 *      Author: Viktor Cejnek
 */

#include "main.h"
#include "TMC2009_UART.h"
#include "stm32wbxx_hal.h"
#include <math.h>
#include "rev.h"


UART_HandleTypeDef *TMC_UART;

uint8_t crc_32(uint32_t data);
uint8_t crc_64(uint64_t data);
uint8_t check_register(uint8_t reg_address);

uint16_t m_UART_communication_pause = 52083; // int(500/baudrate*1000000)
uint16_t m_UART_communication_timeout = 2083; // int(20000/baudrate*1000)


uint32_t TMC_read (uint8_t reg_address){
	// Check if the provided reg_address is READ
	if(check_register(reg_address) & R){
		write_read_reply_datagram_t UART_response = {0};
		read_request_datagram_t UART_read = {0};
		UART_read.sync = REG_SYNC;
		UART_read.register_address = reg_address;
		UART_read.crc = crc_32(UART_read.bytes);


		HAL_HalfDuplex_EnableTransmitter(TMC_UART);
		HAL_UART_Transmit(TMC_UART, (uint8_t *)&UART_read, 4, m_UART_communication_timeout);

		HAL_HalfDuplex_EnableReceiver(TMC_UART);
		HAL_UART_Receive(TMC_UART, (uint8_t *)&UART_response, 8, m_UART_communication_timeout);


		if(UART_response.crc == crc_64(UART_response.bytes))
			return rev32(UART_response.data);		//return the read data in readable format

	}

	return 0;
}


uint32_t TMC_write_only(uint8_t reg_address, uint32_t data){
	// Check if the provided reg_address is WRITE
	if(check_register(reg_address) & W){
		write_read_reply_datagram_t UART_write = {0};
		UART_write.sync = REG_SYNC;
		UART_write.register_address = reg_address + REG_WRITE;
		UART_write.data = rev32(data);
		UART_write.crc = crc_64(UART_write.bytes);

		HAL_HalfDuplex_EnableTransmitter(TMC_UART);
		HAL_UART_Transmit(TMC_UART, (uint8_t *)&UART_write, 8, m_UART_communication_timeout);

		return UART_write.data;
	}

	return 0;
}


uint32_t TMC_write_bit (uint8_t reg_address, uint32_t reg_mask, uint8_t data){
	// Check if the provided reg_address is READ-WRITE
	if(check_register(reg_address) & RW){
		write_read_reply_datagram_t UART_write_response = {0};
		read_request_datagram_t UART_read = {0};
		UART_read.sync = REG_SYNC;
		UART_read.register_address = reg_address;
		UART_read.crc = crc_32(UART_read.bytes);

		HAL_HalfDuplex_EnableTransmitter(TMC_UART);
		HAL_UART_Transmit(TMC_UART, (uint8_t *)&UART_read, 4, m_UART_communication_timeout);

		HAL_HalfDuplex_EnableReceiver(TMC_UART);
		HAL_UART_Receive(TMC_UART, (uint8_t *)&UART_write_response, 8, m_UART_communication_timeout);

		if (UART_write_response.crc == crc_64(UART_write_response.bytes)) {
			UART_write_response.register_address += 0x80;
			UART_write_response.serial_address = 0x00;
			reg_mask = rev32(reg_mask);

			// This part changes the only the data that reg_mask let's us access.
			if (data) {
				UART_write_response.data |= reg_mask;
			} else {
				UART_write_response.data &= ~reg_mask;
			}

			// Calculate CRC
			UART_write_response.crc = crc_64(UART_write_response.bytes);

			HAL_HalfDuplex_EnableTransmitter(TMC_UART);
			HAL_UART_Transmit(TMC_UART, (uint8_t *)&UART_write_response, 8, m_UART_communication_timeout);

			return rev32(UART_write_response.data);
		}
	}

	return 0;
}


uint32_t TMC_write_word (uint8_t reg_address, uint32_t reg_mask, uint32_t data){
	// Check if the provided reg_address is READ-WRITE
	if(check_register(reg_address) & RW){
		write_read_reply_datagram_t UART_write_response = {0};
		read_request_datagram_t UART_read = {0};
		UART_read.sync = REG_SYNC;
		UART_read.register_address = reg_address;
		UART_read.crc = crc_32(UART_read.bytes);


		HAL_HalfDuplex_EnableTransmitter(TMC_UART);
		HAL_UART_Transmit(TMC_UART, (uint8_t *)&UART_read, 4, m_UART_communication_timeout);

		HAL_HalfDuplex_EnableReceiver(TMC_UART);
		HAL_UART_Receive(TMC_UART, (uint8_t *)&UART_write_response, 8, m_UART_communication_timeout);

		if (UART_write_response.crc == crc_64(UART_write_response.bytes)) {
			UART_write_response.register_address += 0x80;
			UART_write_response.serial_address = 0x00;

			// This part changes the only the data that reg_mask let's us access.
			UART_write_response.data = rev32(UART_write_response.data);
			UART_write_response.data = (UART_write_response.data & ~reg_mask) | (data << (uint32_t)(log2(reg_mask & -reg_mask)));
			UART_write_response.data = rev32(UART_write_response.data);

			// Calculate CRC
			UART_write_response.crc = crc_64(UART_write_response.bytes);

			HAL_HalfDuplex_EnableTransmitter(TMC_UART);
			HAL_UART_Transmit(TMC_UART, (uint8_t *)&UART_write_response, 8, m_UART_communication_timeout);

			return UART_write_response.data;

		}
	}

	return 0;
}


uint32_t TMC_write_IHOLD_IRUN(uint8_t IHOLD, uint8_t IRUN, uint8_t IHOLDDELAY){
	if (IHOLD <= 31 && IRUN <= 31 && IHOLDDELAY <= 15) {
		write_read_reply_datagram_t UART_write = {0};
		UART_write.sync = REG_SYNC;
		UART_write.register_address = REG_IHOLD_IRUN + REG_WRITE;
		UART_write.data = IHOLD | IRUN << 8 | IHOLDDELAY << 16;
		UART_write.data = rev32(UART_write.data);
		UART_write.crc = crc_64(UART_write.bytes);

		HAL_HalfDuplex_EnableTransmitter(TMC_UART);
		HAL_UART_Transmit(TMC_UART, (uint8_t *)&UART_write, 8, m_UART_communication_timeout);

		return rev32(UART_write.data);
	}

	return 0;
}


/**
 * @brief Calculates the CRC for 32bit message.
 */
uint8_t crc_32(uint32_t data){
	uint8_t crc = 0;
	data = data&0x00FFFFFF;
	for (int i = 0; i < 24; ++i) {
		if((crc >> 7) ^ (data & 0x01)) {
			crc = ((crc << 1) ^ 0x07) & 0xFFFFFF;
		} else {
			crc = (crc << 1) & 0xFFFFFF;
		}
		data = data >> 1;
	}

	return crc;
}

/**
 * @brief Calculates the CRC for 64bit message.
 */
uint8_t crc_64(uint64_t data){
	uint8_t crc = 0;
	data = data & 0x00FFFFFFFFFFFFFF;

	for (int i = 0; i < 56; ++i) {
		if((crc >> 7) ^ (data & 0x01)) {
			crc = ((crc << 1) ^ 0x07) & 0xFFFFFFFFFFFFFFFF;
		} else {
			crc = (crc << 1) & 0xFFFFFFFFFFFFFFFF;
		}
		data = data >> 1;
	}
	return crc;
}

/**
 *
 */
uint8_t check_register(uint8_t reg_adress){
	for (int i = 0; i < sizeof(register_filter)/2; ++i) {
		if(reg_adress == register_filter[i][0])
			return register_filter[i][1];
	}
	return 0x0;
}
