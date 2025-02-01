/**
 *	@file TMC2009_UART.h
 *
 *  @date
 *  May 6, 2024
 *	@author
 *	Viktor Cejnek
 */

#include "stm32wbxx_hal.h"

#ifndef INC_TMC2009_UART_H
#define INC_TMC2009_UART_H

uint32_t TMC_read(uint8_t address);
uint32_t TMC_read_word(uint8_t reg_address, uint32_t reg_mask);
uint32_t TMC_write_only(uint8_t reg_address, uint32_t data);
uint32_t TMC_write_bit(uint8_t reg_address, uint32_t reg_mask, uint8_t data);
uint32_t TMC_write_word(uint8_t reg_address, uint32_t reg_mask, uint32_t data);
uint32_t TMC_write_IHOLD_IRUN(uint8_t IHOLD, uint8_t IRUN, uint8_t IHOLDDELAY);


void TMC_VACTUAL(int32_t velocity);
/*
uint32_t TMC_write_stop(void);
uint32_t TMC_write_move_angle(uint32_t angle);
*/

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
#define REG_WRITE				0x80

// Registers
// 	General Registers
#define REG_GCONF           	0x00	//RW
#define REG_GSTAT           	0x01	//R+WC
#define REG_IFCNT           	0x02	//R
#define REG_SLAVECONF			0x03	//W
#define REG_OTP_PROG			0x04	//W
#define REG_OTP_READ			0x05	//R
#define REG_IOIN            	0x06	//R
#define FACTORY_CONF			0x07	//RW
//	Velocity Dependent Control
#define REG_IHOLD_IRUN      	0x10	//W
#define REG_TPOWERDOWN			0x11	//W
#define REG_TSTEP           	0x12	//R
#define REG_TPWMTHRS			0x13	//W
#define REG_VACTUAL				0x22	//W
//	StallGuard Control
#define REG_TCOOLTHRS       	0x14	//W
#define REG_SGTHRS          	0x40	//W
#define REG_SG_RESULT       	0x41	//R
#define REG_COOLCONF			0x42	//W
//	Sequencer registers
#define REG_MSCNT          		0x6A	//R
#define REG_MSCURACT			0x6B	//R
//	Chopper Control Registers
#define REG_CHOPCONF        	0x6C	//RW			Reset default=0x10000053
#define REG_DRVSTATUS       	0x6F	//R
#define REG_PWMCONF				0x70	//RW			Reset default=0xC10D0024
#define REG_PWM_SCALE			0x71	//R
#define REG_PWM_AUTO			0x72	//R



// GCONF		0x02	10bits	RW
#define MASK_i_scale_analog		1<<0
#define MASK_internal_rsense 	1<<1
#define MASK_en_spreadcycle     1<<2
#define MASK_shaft              1<<3
#define MASK_index_otpw         1<<4
#define MASK_index_step         1<<5
#define MASK_pdn_disable		1<<6
#define MASK_mstep_reg_select	1<<7
#define MASK_multistep_filt		1<<8
#define MASK_test_mode			1<<9

// GSTAT		0x01	3bits	R+WC
#define MASK_reset              1<<0
#define MASK_drv_err            1<<1
#define MASK_uv_cp              1<<2

// IFCNT		0x02	8bits	R
#define MASK_IFCNT				255<<0

// SLAVECONF	0x03	4bits	W
#define MASK_SENDDELAY			15<<8

// OTP_PROG		0x04	16bits	W
#define MASK_OTPBIT				7<<0
#define MASK_OTPBYTE			3<<4
#define MASK_OTPMAGIC			255<<8

// OTP_READ		0x05	24bits	R
#define MASK_OTP0				255<<0
#define MASK_OTP1				255<<8
#define MASK_OTP2				255<<16

// IOIN			0x06	18bits	R
#define MASK_ENN 	            1<<0
#define MASK_MS1				1<<2
#define MASK_MS2				1<<3
#define MASK_DIAG				1<<4
#define MASK_PDN_UART			1<<6
#define MASK_STEP				1<<7
#define MASK_SPREAD				1<<8
#define MASK_DIR				1<<9
#define MASK_VERSION			8<<24

// FACTORY_CONF	0x07	7bits	RW
#define MASK_FCLKTRIM			31<<0
#define MASK_OTTRIM				3<<8



// IHOLD_IRUN	0x10	14bits	W
#define MASK_IHOLD              31<<0
#define MASK_IRUN				31<<8
#define MASK_IHOLDDELAY			15<<16

// TPOWERDOWN	0x11	8bits	W
#define MASK_TPOWERDOWN			255<<0

// TSTEP		0x12	20bits	R
#define MASK_TSTEP				0xFFFFF

// TPWMTHRS		0x13	20bits	W
#define MASK_TPWMTHRS			0xFFFFF

// VACTUAL		0x22	24bits	W
#define MASK_VACTUAL			0xFFFFFF



// TCOOLTHRS	0x14	20bits	W
#define MASK_TCOOLTHRS			0xFFFFF

// SGTHRS		0x40	8bits	W
#define MASK_SGTHRS             255<<0

// SG_RESULT	0x41	10bits	R
#define MASK_SG_RESULT			1023<<0

// COOLCONF		0x42	16bits	W
#define MASK_semin				15<<0
#define MASK_seup				3<<5
#define MASK_semax				15<<8
#define MASK_sedn				3<<13
#define MASK_seimin				1<<15



// MSCNT		0x6A	10bits	R
#define MASK_MSCNT				1023<<0

// MSCURACT		0x6B	18bits	R
#define MASK_CUR_A				511<<0
#define MASK_CUR_B				511<<16



// CHOPCONF		0x6C	32bits	RW
#define MASK_toff				15<<0
#define MASK_hstrt				7<<4
#define MASK_hend				15<<7
#define MASK_tbl				3<<15
#define MASK_vsense             1<<17
#define MASK_msres				15<<24
#define MASK_msres0             1<<24
#define MASK_msres1             1<<25
#define MASK_msres2             1<<26
#define MASK_msres3             1<<27
#define MASK_intpol             1<<28
#define MASK_dedge				1<<29
#define MASK_diss2vs			1<<30
#define MASK_diss2g				1<<31

// DRVSTATUS	0x6F	32bits	R
#define MASK_otpw               1<<0
#define MASK_ot                 1<<1
#define MASK_s2ga               1<<2
#define MASK_s2gb               1<<3
#define MASK_s2vsa              1<<4
#define MASK_s2vsb              1<<5
#define MASK_ola                1<<6
#define MASK_olb                1<<7
#define MASK_t120               1<<8
#define MASK_t143               1<<9
#define MASK_t150               1<<10
#define MASK_t157               1<<11
#define MASK_cs_actual          31<<16
#define MASK_stealth            1<<30
#define MASK_stst               1<<31

// PWMCONF		0x70	22bits	RW
#define MASK_PWM_LIM			15<<28
#define MASK_PWM_REG			15<<24
#define MASK_freewheel1			1<<21
#define MASK_freewheel0			1<<20
#define MASK_freewheel			3<<20
#define MASK_pwm_autograd		1<<19
#define MASK_pwm_autoscale		1<<18
#define MASK_pwm_freq1			1<<17
#define MASK_pwm_freq0			1<<16
#define MASK_pwm_freq			3<<16
#define MASK_PWM_GRAD			255<<8
#define MASK_PWM_OFS			255<<0

// PWM_SCALE	0x71	17bits	R
#define MASK_PWM_SCALE_SUM		255<<0
#define MASK_PWM_SCALE_AUTO		1023<<16

// PWM_AUTO		0x72	16bits	R
#define MASK_PWM_OFS_AUTO		255<<0
#define MASK_PWM_GRAD_AUTO		255<<16



// others
#define TMC_mres_256 			0
#define TMC_mres_128 			1
#define TMC_mres_64 			2
#define TMC_mres_32 			3
#define TMC_mres_16 			4
#define TMC_mres_8 				5
#define TMC_mres_4 				6
#define TMC_mres_2 				7
#define TMC_mres_1 				8



// Filter registers to R, RW and W
#define W	0x1
#define R	0x2
#define RW	0x3

static const uint8_t register_filter[][2]={
		{REG_GCONF,		RW},
		{REG_GSTAT, 	R},
		{REG_GSTAT, 	RW},
		{REG_SLAVECONF,	W},
		{REG_OTP_PROG,	W},
		{REG_OTP_READ,	R},
		{REG_IOIN,		R},
		{FACTORY_CONF,	RW},

		{REG_IHOLD_IRUN,W},
		{REG_TPOWERDOWN,W},
		{REG_TSTEP,		R},
		{REG_TPWMTHRS,	W},
		{REG_VACTUAL,	W},

		{REG_TCOOLTHRS,	W},
		{REG_SGTHRS,	W},
		{REG_SG_RESULT,	R},
		{REG_COOLCONF,	W},

		{REG_MSCNT,		R},
		{REG_MSCURACT,	R},

		{REG_CHOPCONF,	RW},
		{REG_DRVSTATUS,	R},
		{REG_PWMCONF,	RW},
		{REG_PWM_SCALE,	R},
		{REG_PWM_AUTO,	R}
};



// Datasheet vs Actual mask position
//  7  6  5  4  3  2  1  0		15 14 13 12 11 10  9  8		23 22 21 20 19 18 17 16		31 30 29 28 27 26 25 24
// 31 30 29 28 27 26 25 24		23 22 21 20 19 18 17 16		15 14 13 12 11 10  9  8		 7  6  5  4  3  2  1  0



/*
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
#define REG_ihold               31<<24
#define REG_irun                31<<16
#define REG_iholddelay          15<<8

// SGTHRS
#define REG_sgthrs              255<<24

// TPWMTHRS - 20 bits
#define REG_tpwmthrs_val		0xFFFFF<<8

// VACTUAL - 24 bits
#define REG_vactual_val			0xFFFFFF<<8*/

#endif /* INC_TMC2009_UART_H_ */
