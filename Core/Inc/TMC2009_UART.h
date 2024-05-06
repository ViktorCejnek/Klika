/*
 * TMC2009_UART.h
 *
 *  Created on: May 6, 2024
 *      Author: Viktor Cejnek
 */

#ifndef INC_TMC2009_UART_H_
#define INC_TMC2009_UART_H_

#include "stm32wbxx_hal.h"

void test(void);

UART_HandleTypeDef *TMC_UART;

// ADDR write offset
#define REG_OFFSET				0x80

// addresses
#define REG_GCONF           	0x00
#define REG_GSTAT           	0x01
#define REG_IFCNT           	0x02
#define REG_IOIN            	0x06
#define REG_IHOLD_IRUN      	0x10
#define REG_TSTEP           	0x12
#define REG_TCOOLTHRS       	0x14
#define REG_SGTHRS          	0x40
#define REG_SG_RESULT       	0x41
#define REG_MSCNT          		0x6A
#define REG_CHOPCONF        	0x6C
#define REG_DRVSTATUS       	0x6F

// GCONF
#define REG_i_scale_analog      1<<0
#define REG_internal_rsense     1<<1
#define REG_en_spreadcycle      1<<2
#define REG_shaft               1<<3
#define REG_index_otpw          1<<4
#define REG_index_step          1<<5
#define REG_mstep_reg_select    1<<7

// GSTAT
#define REG_reset               1<<0
#define REG_drv_err             1<<1
#define REG_uv_cp               1<<2

// CHOPCONF
#define REG_vsense              1<<17
#define REG_msres0              1<<24
#define REG_msres1              1<<25
#define REG_msres2              1<<26
#define REG_msres3              1<<27
#define REG_intpol              1<<28

// IOIN
#define REG_io_enn              1<<0
#define REG_io_step             1<<7
#define REG_io_spread           1<<8
#define REG_io_dir              1<<9

// DRVSTATUS
#define REG_stst                1<<31
#define REG_stealth             1<<30
#define REG_cs_actual           31<<16
#define REG_t157                1<<11
#define REG_t150                1<<10
#define REG_t143                1<<9
#define REG_t120                1<<8
#define REG_olb                 1<<7
#define REG_ola                 1<<6
#define REG_s2vsb               1<<5
#define REG_s2vsa               1<<4
#define REG_s2gb                1<<3
#define REG_s2ga                1<<2
#define REG_ot                  1<<1
#define REG_otpw                1<<0

// IHOLD_IRUN
#define REG_ihold               31<<0
#define REG_irun                31<<8
#define REG_iholddelay          15<<16

// SGTHRS
#define REG_sgthrs              255<<0

// others
#define REG_mres_256 			0
#define REG_mres_128 			1
#define REG_mres_64 			2
#define REG_mres_32 			3
#define REG_mres_16 			4
#define REG_mres_8 				5
#define REG_mres_4 				6
#define REG_mres_2 				7
#define REG_mres_1 				8

uint8_t m_UART_rFrame[4] = {0x55, 0, 0, 0};
uint8_t m_UART_wFrame[8] = {0x55, 0, 0, 0 , 0, 0, 0, 0};
uint16_t m_UART_communication_pause = 4340; // int(500/baudrate*1000000)
uint16_t m_UART_communication_timeout = 173; // int(20000/baudrate*1000)

#endif /* INC_TMC2009_UART_H_ */
