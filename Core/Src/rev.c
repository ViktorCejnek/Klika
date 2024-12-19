/**
 * @file rev.c
 * @brief Utilizes REV and RBIT functions
 *
 *  Created on: Oct 17, 2024
 *      Author: Viktor Cejnek
 */
#include "main.h"
#include "rev.h"
#include <arm_acle.h>

uint32_t rev16(uint32_t value){
	value = __REV16(value); // Generates the REV16 instruction
	return value;
}

uint32_t rev32(uint32_t value){
	value = __REV(value); // Generates the REV instruction
	return value;
}

uint64_t rev64(uint64_t value){
	return (uint64_t)rev32(value & 0xffffffff)<<32 | rev32(value>>32);
}

uint32_t revsh(uint32_t value) {
    __REVSH(value); // Generates the REVSH instruction
    return value;
}

uint32_t rbit(uint32_t value) {
    __RBIT(value); // Generates the RBIT instruction
    return value;
}
