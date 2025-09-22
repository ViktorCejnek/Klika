/*
 * Calibration.c
 *
 *  Created on: Aug 26, 2025
 *      Author: Viktor Cejnek
 */

#include <calibration.h>

void start_calibration(){
  //FSM?

  /*
   * 1 set default parameters
   * 2 start turning
   * 3 wait
   *  a f_diag => finished
   *  b timeout => current too small or stallguard threshold wrong.
  */

  int8_t direction = 1;
  uint32_t timer_start;
  uint32_t timer_now;
  uint32_t threshold = 255;
  uint8_t max_SGTHRS = 0;
  uint8_t min_SGTHRS = 255;
  enum calib_FSMSTATE curr_state = calib_Start;

  while(curr_state!=calib_Exit){
    switch (curr_state) {
    case (calib_Start):
      HAL_GPIO_WritePin(ENN_GPIO_Port, ENN_Pin, RESET);
      curr_state = calib_Rotate;
      break;

    case (calib_Rotate):
      TMC_write_only(REG_SGTHRS, threshold);
      HAL_Delay(200);
      TMC_VACTUAL(speed * direction);
      HAL_Delay(200);
      //HAL_GPIO_WritePin(ENN_GPIO_Port, ENN_Pin, RESET);
      timer_start = HAL_GetTick();
      direction = -direction;
      threshold -= 5;
      curr_state = calib_Turning;
      break;

    case (calib_Turning):
      timer_now = HAL_GetTick();
      if (f_Diag) {
        curr_state = calib_Stall;
      }
      else if ((timer_now - timer_start) > (TMC_timeout))
      {
        TMC_VACTUAL(0);
        if (min_SGTHRS > max_SGTHRS)
        {
          curr_state = calib_Rotate;
        }
        else
        {
          curr_state = calib_Timeout;
        }
      }
      else
      {
        curr_state = calib_Turning;
      }
      break;

    case (calib_Stall):
      TMC_VACTUAL(0);
      HAL_GPIO_WritePin(ENN_GPIO_Port, ENN_Pin, RESET);
      f_Diag = 0;
      if (((timer_now - timer_start) > (TMC_timeout/3)) && (threshold > max_SGTHRS))
      {
        max_SGTHRS = threshold;
      }
      else if (threshold < min_SGTHRS)
      {
        min_SGTHRS = threshold;
      }
      curr_state = calib_Rotate;
      break;

    case (calib_Timeout):
      TMC_VACTUAL(0);
      curr_state = calib_Done;
      break;

    case (calib_Done):
      threshold = (min_SGTHRS + (max_SGTHRS - min_SGTHRS) / 2);
      TMC_write_only(REG_SGTHRS, threshold);
      curr_state = calib_Exit;
      break;

    default:
      curr_state = calib_Exit;
      break;
    }
  }
}

