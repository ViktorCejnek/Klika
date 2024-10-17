/*
 * endian.h
 *
 *  Created on: Oct 17, 2024
 *      Author: Viktor Cejnek
 */

#ifndef INC_REV_H_
#define INC_REV_H_

#include <arm_acle.h>

uint32_t rev(uint32_t value);

uint32_t rev16(uint32_t value);

uint64_t rev64(uint64_t value);

uint32_t revsh(uint32_t value);

uint32_t rbit(uint32_t value);

#endif /* INC_REV_H_ */
