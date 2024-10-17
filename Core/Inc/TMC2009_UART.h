/*
 * TMC2009_UART.h
 *
 *  Created on: May 6, 2024
 *      Author: Viktor Cejnek
 */

#include "stm32wbxx_hal.h"

#ifndef INC_TMC2009_UART_H
#define INC_TMC2009_UART_H

void TMC_turn (int16_t angle);
uint32_t TMC_read (uint8_t address);
uint32_t TMC_write_bit (uint8_t reg_address, uint32_t reg_mask, uint8_t data);
uint32_t TMC_write_word (uint8_t reg_address, uint32_t reg_mask, uint32_t data);
uint32_t TMC_write_IHOLD_IRUN(uint8_t IHOLD, uint8_t IRUN, uint8_t IHOLDDELAY);
uint32_t TMC_write_stop(void);
uint32_t TMC_write_move_angle(uint32_t angle);


typedef union
{
  struct
  {
    uint64_t sync : 8;
    uint64_t serial_address : 8;
    uint64_t register_address : 8;
    uint64_t data : 32;
    uint64_t crc : 8;
  };
  uint64_t bytes;
} write_read_reply_datagram_t;

typedef union
{
  struct
  {
	uint32_t sync : 8;
	uint32_t serial_address : 8;
	uint32_t register_address : 8;
	uint32_t crc : 8;
  };
  uint32_t bytes;
} read_request_datagram_t;


extern UART_HandleTypeDef *TMC_UART;


/*
uint8_t m_UART_rFrame[4];
uint8_t m_UART_wFrame[8];
uint16_t m_UART_communication_pause; // int(500/baudrate*1000000)
uint16_t m_UART_communication_timeout; // int(20000/baudrate*1000)*/


// SYNC byte
#define REG_SYNC				0x05

// ADDR write offset
#define REG_OFFSET				0x80

// addresses
#define REG_GCONF           	0x00
#define REG_GSTAT           	0x01
#define REG_IFCNT           	0x02
#define REG_IOIN            	0x06
#define REG_IHOLD_IRUN      	0x10
#define REG_TSTEP           	0x12
#define REG_TPWMTHRS			0x13
#define REG_TCOOLTHRS       	0x14
#define REG_VACTUAL				0x22
#define REG_SGTHRS          	0x40
#define REG_SG_RESULT       	0x41
#define REG_MSCNT          		0x6A
#define REG_CHOPCONF        	0x6C
#define REG_DRVSTATUS       	0x6F
#define REG_PWMCONF				0x70


/*
// GCONF
#define REG_i_scale_analog      1<<0
#define REG_internal_rsense     1<<1
#define REG_en_spreadcycle      1<<2
#define REG_shaft               1<<3
#define REG_index_otpw          1<<4
#define REG_index_step          1<<5
#define REG_pdn_disable			1<<6
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
*/

// Datasheet vs Actual mask position
//  7  6  5  4  3  2  1  0		15 14 13 12 11 10  9  8		23 22 21 20 19 18 17 16		31 30 29 28 27 26 25 24
// 31 30 29 28 27 26 25 24		23 22 21 20 19 18 17 16		15 14 13 12 11 10  9  8		 7  6  5  4  3  2  1  0

// GCONF
#define REG_i_scale_analog      1<<24	//e.g. bit 0 translates to bit 24
#define REG_internal_rsense     1<<25
#define REG_en_spreadcycle      1<<26
#define REG_shaft               1<<27
#define REG_index_otpw          1<<28
#define REG_index_step          1<<29
#define REG_pdn_disable			1<<30
#define REG_mstep_reg_select    1<<31
#define REG_multistep_filt		1<<16

// PWMCONF
#define REG_PWM_LIM				15<<4
#define REG_PWM_REG				15<<0
#define REG_freewheel1			1<<13
#define REG_freewheel0			1<<12
#define REG_pwm_autograd		1<<11
#define REG_pwm_autoscale		1<<10
#define REG_pwm_freq1			1<<9
#define REG_pwm_freq0			1<<8
#define REG_PWM_GRAD			255<<16
#define REG_PWM_OFS				255<<24

// GSTAT
#define REG_reset               1<<24
#define REG_drv_err             1<<25
#define REG_uv_cp               1<<26

// CHOPCONF
#define REG_intpol              1<<4
#define REG_mres0               1<<0
#define REG_mres1               1<<1
#define REG_mres2				1<<2
#define REG_mres3            	1<<3
#define REG_vsense              1<<9
#define REG_tbl1				1<<8
#define REG_tbl0				1<<23
#define REG_hend3				1<<18
#define REG_hend2				1<<17
#define REG_hend1				1<<16
#define REG_hend0				1<<31
#define REG_hstrt				7<<28
#define REG_toff				15<<24

// IOIN
#define REG_io_enn              1<<24
#define REG_io_step             1<<31
#define REG_io_spread           1<<16
#define REG_io_dir              1<<17

// DRVSTATUS
/*#define REG_stst                1<<31
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
#define REG_otpw                1<<0*/

// IHOLD_IRUN
#define REG_ihold               31<<24
#define REG_irun                31<<16
#define REG_iholddelay          15<<8

// SGTHRS
#define REG_sgthrs              255<<24

// TPWMTHRS - 20 bits
#define REG_tpwmthrs_val		0xFFFFF<<8

// VACTUAL - 24 bits
#define REG_vactual_val			0xFFFFFF<<8

#endif /* INC_TMC2009_UART_H_ */
