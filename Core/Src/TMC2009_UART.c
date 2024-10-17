/*
 * TMC2009_UART.c
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

uint16_t m_UART_communication_pause = 52083; // int(500/baudrate*1000000)
uint16_t m_UART_communication_timeout = 2083; // int(20000/baudrate*1000)

void TMC_turn (int16_t angle){

}


uint32_t TMC_read (uint8_t reg_address){
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
		return rev(UART_response.data);		//return the read data in readable format
	else
		return 0;
}


uint32_t TMC_write_bit (uint8_t reg_address, uint32_t reg_mask, uint8_t data){
	write_read_reply_datagram_t UART_response = {0};
	write_read_reply_datagram_t UART_write = {0};
	read_request_datagram_t UART_read = {0};
	UART_read.sync = REG_SYNC;
	UART_read.register_address = reg_address;

	uint8_t crc = 0;
	uint64_t byte = UART_read.bytes;
	crc = 0;
	for (int i = 0; i < 24; ++i) {
		if((crc >> 7) ^ (byte & 0x01)) {
			crc = ((crc << 1) ^ 0x07) & 0xFFFFFF;
		} else {
			crc = (crc << 1) & 0xFFFFFF;
		}
		byte = byte >> 1;
	}
	UART_read.crc = crc;


	HAL_HalfDuplex_EnableTransmitter(TMC_UART);
	HAL_UART_Transmit(TMC_UART, (uint8_t *)&UART_read, 4, m_UART_communication_timeout);

	HAL_HalfDuplex_EnableReceiver(TMC_UART);
	HAL_UART_Receive(TMC_UART, (uint8_t *)&UART_response, 8, m_UART_communication_timeout);

	byte = (UART_response.bytes & 0x00FFFFFFFFFFFFFF);
	crc = 0;
	for (int i = 0; i < 56; ++i) {
		if((crc >> 7) ^ (byte & 0x01)) {
			crc = ((crc << 1) ^ 0x07) & 0xFFFFFFFFFFFFFFFF;
		} else {
			crc = (crc << 1) & 0xFFFFFFFFFFFFFFFF;
		}
		byte = byte >> 1;
	}

	if (crc == UART_response.crc) {
		UART_write = UART_response;
		UART_write.register_address += 0x80;
		UART_write.serial_address = 0x00;

		if (data) {
			UART_write.data |= reg_mask;
		} else {
			UART_write.data &= ~reg_mask;
		}

		byte = (UART_write.bytes & 0x00FFFFFFFFFFFFFF);
		crc = 0;

		for (int i = 0; i < 56; ++i) {
			if((crc >> 7) ^ (byte & 0x01)) {
				crc = ((crc << 1) ^ 0x07) & 0xFFFFFFFFFFFFFFFF;
			} else {
				crc = (crc << 1) & 0xFFFFFFFFFFFFFFFF;
			}
			byte = byte >> 1;
		}
		UART_write.crc = crc;

		HAL_HalfDuplex_EnableTransmitter(TMC_UART);
		HAL_UART_Transmit(TMC_UART, (uint8_t *)&UART_write, 8, m_UART_communication_timeout);

		return UART_write.data;

	} else {
		return 0;
	}

}

uint32_t TMC_write_word (uint8_t reg_address, uint32_t reg_mask, uint32_t data){
	write_read_reply_datagram_t UART_response = {0};
	write_read_reply_datagram_t UART_write = {0};
	read_request_datagram_t UART_read = {0};
	UART_read.sync = REG_SYNC;
	UART_read.register_address = reg_address;

	if(data > reg_mask){
		Error_Handler();
	}

	uint8_t crc = 0;
	uint64_t byte = UART_read.bytes;
	crc = 0;
	for (int i = 0; i < 24; ++i) {
		if((crc >> 7) ^ (byte & 0x01)) {
			crc = ((crc << 1) ^ 0x07) & 0xFFFFFF;
		} else {
			crc = (crc << 1) & 0xFFFFFF;
		}
		byte = byte >> 1;
	}
	UART_read.crc = crc;


	HAL_HalfDuplex_EnableTransmitter(TMC_UART);
	HAL_UART_Transmit(TMC_UART, (uint8_t *)&UART_read, 4, m_UART_communication_timeout);

	HAL_HalfDuplex_EnableReceiver(TMC_UART);
	HAL_UART_Receive(TMC_UART, (uint8_t *)&UART_response, 8, m_UART_communication_timeout);

	byte = (UART_response.bytes & 0x00FFFFFFFFFFFFFF);
	crc = 0;
	for (int i = 0; i < 56; ++i) {
		if((crc >> 7) ^ (byte & 0x01)) {
			crc = ((crc << 1) ^ 0x07) & 0xFFFFFFFFFFFFFFFF;
		} else {
			crc = (crc << 1) & 0xFFFFFFFFFFFFFFFF;
		}
		byte = byte >> 1;
	}

	if (crc == UART_response.crc) {
		UART_write = UART_response;
		UART_write.register_address += 0x80;
		UART_write.serial_address = 0x00;

		if (data) {
			UART_write.data = (UART_write.data & ~reg_mask) | (data << (uint32_t)(log2(reg_mask & -reg_mask)));
		}

		byte = (UART_write.bytes & 0x00FFFFFFFFFFFFFF);
		crc = 0;

		for (int i = 0; i < 56; ++i) {
			if((crc >> 7) ^ (byte & 0x01)) {
				crc = ((crc << 1) ^ 0x07) & 0xFFFFFFFFFFFFFFFF;
			} else {
				crc = (crc << 1) & 0xFFFFFFFFFFFFFFFF;
			}
			byte = byte >> 1;
		}
		UART_write.crc = crc;

		HAL_HalfDuplex_EnableTransmitter(TMC_UART);
		HAL_UART_Transmit(TMC_UART, (uint8_t *)&UART_write, 8, m_UART_communication_timeout);

		return UART_write.data;

	} else {
		return 0;
	}

}

uint32_t TMC_write_IHOLD_IRUN(uint8_t IHOLD, uint8_t IRUN, uint8_t IHOLDDELAY){

	if (IHOLD <= 31 && IRUN <= 31 && IHOLDDELAY <= 15) {
		write_read_reply_datagram_t UART_write = {0};
		UART_write.sync = REG_SYNC;
		UART_write.register_address = REG_IHOLD_IRUN + 0x80;
		UART_write.data = IHOLD << 24 | IRUN << 16 | IHOLDDELAY << 8;

		/*
		uint8_t crc = 0;
		uint32_t byte = (UART_write.bytes & 0x00FFFFFFFFFFFFFF);

		for (int i = 0; i < 56; ++i) {
			if((crc >> 7) ^ (byte & 0x01)) {
				crc = ((crc << 1) ^ 0x07) & 0xFFFFFFFFFFFFFFFF;
			} else {
				crc = (crc << 1) & 0xFFFFFFFFFFFFFFFF;
			}
			byte = byte >> 1;
		}
		UART_write.crc = crc;
		*/

		UART_write.crc = crc_64(UART_write.bytes);

		HAL_HalfDuplex_EnableTransmitter(TMC_UART);
		HAL_UART_Transmit(TMC_UART, (uint8_t *)&UART_write, 8, m_UART_communication_timeout);


		return UART_write.data;
	} else {
		return 0;
	}

}

//vactual = velocity/0.715 Hz

// Datasheet vs Actual mask position
//  7  6  5  4  3  2  1  0		15 14 13 12 11 10  9  8		23 22 21 20 19 18 17 16		31 30 29 28 27 26 25 24
// 31 30 29 28 27 26 25 24		23 22 21 20 19 18 17 16		15 14 13 12 11 10  9  8		 7  6  5  4  3  2  1  0

uint32_t TMC_write_move_angle(uint32_t speed){
	//uint32_t NUM_OF_STEPS = 51200*angle/360;

	write_read_reply_datagram_t UART_write = {0};
	UART_write.sync = REG_SYNC;
	UART_write.register_address = REG_VACTUAL + 0x80;

	UART_write.data = speed;
	uint8_t crc = 0;
	uint32_t byte = (UART_write.bytes & 0x00FFFFFFFFFFFFFF);

	for (int i = 0; i < 56; ++i) {
		if((crc >> 7) ^ (byte & 0x01)) {
			crc = ((crc << 1) ^ 0x07) & 0xFFFFFFFFFFFFFFFF;
		} else {
			crc = (crc << 1) & 0xFFFFFFFFFFFFFFFF;
		}
		byte = byte >> 1;
	}
	UART_write.crc = crc;

	HAL_HalfDuplex_EnableTransmitter(TMC_UART);
	HAL_UART_Transmit(TMC_UART, (uint8_t *)&UART_write, 8, m_UART_communication_timeout);

	return UART_write.data;
}


uint32_t TMC_write_stop(void){
	write_read_reply_datagram_t UART_write = {0};
	UART_write.sync = REG_SYNC;
	UART_write.register_address = REG_VACTUAL + 0x80;
	UART_write.data = 0x00000000;

	uint8_t crc = 0;
	uint32_t byte = (UART_write.bytes & 0x00FFFFFFFFFFFFFF);

	for (int i = 0; i < 56; ++i) {
		if((crc >> 7) ^ (byte & 0x01)) {
			crc = ((crc << 1) ^ 0x07) & 0xFFFFFFFFFFFFFFFF;
		} else {
			crc = (crc << 1) & 0xFFFFFFFFFFFFFFFF;
		}
		byte = byte >> 1;
	}
	UART_write.crc = crc;

	HAL_HalfDuplex_EnableTransmitter(TMC_UART);
	HAL_UART_Transmit(TMC_UART, (uint8_t *)&UART_write, 8, m_UART_communication_timeout);

	return UART_write.data;
}

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
