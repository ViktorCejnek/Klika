/*
 * Calibration.h
 *
 *  Created on: Aug 26, 2025
 *      Author: Viktor Cejnek
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include <main.h>

#define set_SGTHRS(threshold) TMC_write_only(REG_SGTHRS, threshold);

void start_Calibration();

enum calib_FSMSTATE
{
  calib_Start, calib_Rotate, calib_Turning, calib_Timeout, calib_Stall, calib_Done, calib_Exit
};

#endif /* INC_CALIBRATION_H_ */
