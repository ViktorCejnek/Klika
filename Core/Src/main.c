/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "ipcc.h"
#include "usart.h"
#include "rf.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TMC2009_UART.h"
#include "rev.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

  const int32_t speed = 200000;
  //const int32_t speed = 50000;

  enum FLAG f_Lock = no_flag;
  enum FLAG f_Unlock = no_flag;
  enum FLAG f_Calibration = no_flag;
  enum FLAG f_Demo = no_flag;
  enum FLAG f_Stop = no_flag;
  enum FLAG f_Diag = no_flag;

  uint32_t timer_start;
  uint32_t timer_now;


  //uint32_t sgresult_shifted = 0;
  //uint32_t tstep = 0;
  //uint32_t gstat = 0;
  volatile static uint32_t SG_RESULT = 0;
  volatile static uint32_t SGTHRS = 0;
  //const int32_t  speed = 200000;

  volatile static uint32_t PWM_SCALE_SUM = 0;
  volatile static int32_t  PWM_SCALE_AUTO = 0;
  volatile static uint32_t PWM_OFS_AUTO = 0;
  volatile static uint32_t PWM_GRAD_AUTO = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
  MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_LPUART1_UART_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_RF_Init();
  /* USER CODE BEGIN 2 */

  ///Assign correct UART to TMC_uart variable.
  TMC_UART = &hlpuart1;


  HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MS2_GPIO_Port, MS2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENN_GPIO_Port, ENN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SPREAD_GPIO_Port, SPREAD_Pin, GPIO_PIN_RESET);

  INIT();

  //start_calibration();
  //HAL_Delay(10000);

  //---------------------------------------------------------
  //**test loop for stallguard threshold**
  //---------------------------------------------------------

/*
  //HAL_GPIO_WritePin(ENN_GPIO_Port, ENN_Pin, GPIO_PIN_SET);

  	  	  	  //execute automatic tuning procedure AT

  	  	  	  //slowly accelerate to MAX velocity e.g. VMAX

  	  	  	  //may have to tweak TMWMTHRS (disable with 0)

  	  	  	  //continue with SC2

*/




  volatile static enum FSMSTATE curr_state;
  curr_state = s_Idle;


  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_APPE_Process();

    /* USER CODE BEGIN 3 */
    ///------------------------------------------------------------------------------------------
    /// FSM:
    ///------------------------------------------------------------------------------------------

    switch (curr_state) {
      case (s_Idle):
        if (f_Lock)
        {
          f_Lock = no_flag;
          HAL_GPIO_WritePin(ENN_GPIO_Port, ENN_Pin, RESET);
          curr_state = s_Lock;
        } else if (f_Unlock)
        {
          f_Unlock = no_flag;
          curr_state = s_Unlock;
          HAL_GPIO_WritePin(ENN_GPIO_Port, ENN_Pin, RESET);
        } else if (f_Calibration)
        {
          f_Calibration = no_flag;
          curr_state = s_Calibration;
        } else if (f_Demo)
        {
          f_Demo = no_flag;
          curr_state = s_Demo;
        } else
        {
          curr_state = s_Idle;
        }
        break;

      case (s_Lock):
        Lock();
        timer_start = HAL_GetTick();
        curr_state = s_Turning;
        break;

      case (s_Unlock):
        Unlock();
        timer_start = HAL_GetTick();
        curr_state = s_Turning;
        break;

      case (s_Calibration):
        start_Calibration();
        curr_state = s_Idle;
        break;

      case (s_Demo):
        //start_Demo();
        curr_state = s_Idle;
        break;

      case (s_Turning):
        //timeout, stopped too soon or in correct time
        timer_now = HAL_GetTick();
        if ((timer_now - timer_start) > TMC_timeout) {
          curr_state = s_Timeout;
        } else if (f_Diag) {
          if ((timer_now - timer_start) < TMC_time_per_rot) {
            curr_state = s_Soon;
          } else {
            curr_state = s_Finnished;
          }
          f_Diag = 0;
          Stop();
          HAL_GPIO_WritePin(ENN_GPIO_Port, ENN_Pin, RESET);
        } else {
          curr_state = s_Turning;
        }
        break;

      case (s_Timeout):
        Stop();
        //HAL_GPIO_WritePin(ENN_GPIO_Port, ENN_Pin, SET);
        curr_state = s_Idle;
        break;

      case (s_Soon):

        curr_state = s_Idle;
        break;

      case (s_Finnished):

        curr_state = s_Idle;
        break;

      default: // Error
        curr_state = s_Idle;
        break;
    }




    /*old example
    if(DIAG_flag == 1) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		speed = -speed;
		HAL_Delay(1000);
		HAL_GPIO_WritePin(ENN_GPIO_Port, ENN_Pin, RESET);
		HAL_Delay(1);
		TMC_VACTUAL(0);
		INIT();
		HAL_Delay(1000);
		TMC_VACTUAL(speed);
		DIAG_flag = 0;
    }*/

    /*
    SG_RESULT		= TMC_read(REG_SG_RESULT);
    PWM_SCALE_SUM	= TMC_read_word(REG_PWM_SCALE, MASK_PWM_SCALE_SUM);
    PWM_SCALE_AUTO	= TMC_read_word(REG_PWM_SCALE, MASK_PWM_SCALE_AUTO);
    PWM_OFS_AUTO	= TMC_read_word(REG_PWM_AUTO, MASK_PWM_OFS_AUTO);
    PWM_GRAD_AUTO	= TMC_read_word(REG_PWM_AUTO, MASK_PWM_GRAD_AUTO);
    printf("%lu\n", SG_RESULT);*/

    //change ccr1 register and therefore change rotation speed of motor
    //TIM1->CCR1 = 200;


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
    ITM_SendChar(ch);  // Send character via SWO
    return ch;
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{				// EXTI interrupt
	if(GPIO_Pin == DIAG_Pin) {									// Detect stall, set DIAG_flag and disable the driver
		if(f_Diag == no_flag) {
			HAL_GPIO_WritePin(ENN_GPIO_Port, ENN_Pin, SET);
			f_Diag = flag;
		}
	} else {
	  __NOP();
	}
}


/**
 * @brief This function runs number of settings of the TMC2209 stepper motor driver.
 *
 */
void INIT()
{
///------------------------------------------------------------------------------------------
///	Setup sequence:
///------------------------------------------------------------------------------------------


	///------------------------------------------------------------------------------------------
	///	GCONF
	///------------------------------------------------------------------------------------------
	TMC_write_bit(REG_GCONF, MASK_pdn_disable, 1);			//1 = disable Power down input/enable UART
	TMC_write_bit(REG_GCONF, MASK_i_scale_analog, 0);		//0 = disable external Vref
	TMC_write_bit(REG_GCONF, MASK_en_spreadcycle, 0);		//0 = clear en_spreadcycle in GCONF
	TMC_write_bit(REG_GCONF, MASK_internal_rsense, 0);		//0 = use external Rsense
	TMC_write_bit(REG_GCONF, MASK_mstep_reg_select, 1);		//1 = use value from MSTEP register
	TMC_write_bit(REG_GCONF, MASK_multistep_filt, 1);		//1 = software pulse filtering


	///------------------------------------------------------------------------------------------
	///	CHOPCONF
	///------------------------------------------------------------------------------------------
	TMC_write_bit(REG_CHOPCONF, MASK_vsense, 1);			//1 = use VSENSE (lower current)
	TMC_write_bit(REG_CHOPCONF, MASK_intpol, 1);			//The actual microstep resolution (MRES) becomes extrapolated to 256 microsteps
	TMC_write_word(REG_CHOPCONF, MASK_msres, 0b0000);		//%0000 = 256 ... 128, 64, 32, 16, 8, 4, 2, FULLSTEP

	TMC_write_word(REG_CHOPCONF, MASK_toff, 5);	  	  	//CHOPCONF set basic setting e.g.: TOFF=5, TBL=2, HSTART=4, HEND=0
	TMC_write_word(REG_CHOPCONF, MASK_tbl, 2);
	TMC_write_word(REG_CHOPCONF, MASK_hstrt, 4);
	TMC_write_word(REG_CHOPCONF, MASK_hend, 0);


	///------------------------------------------------------------------------------------------
	///	Velocity Dependent Control
	///------------------------------------------------------------------------------------------
	TMC_write_IHOLD_IRUN(1, 20, 4);						//setup IHOLD, IRUN and I_HOLD_DELAY
	TMC_write_only(REG_TPWMTHRS, 0b0);					//disable TPWMTHRS for only StealthChop
	TMC_write_only(REG_TCOOLTHRS, 1024);				//if TSTEP accedes this value CoolStep is disabled
														//TPOWERDOWN 2-255 default = 20


	///------------------------------------------------------------------------------------------
	///	PWMCONF
	///------------------------------------------------------------------------------------------
	TMC_write_bit(REG_PWMCONF, MASK_pwm_autoscale, 0b1);		//set pwm_autoscale and pwm_autograd in PWMCONF
	TMC_write_bit(REG_PWMCONF, MASK_pwm_autograd, 0b1);

	TMC_write_word(REG_PWMCONF, MASK_pwm_freq, 1);			//select PWM_FREQ in PWMCONF



	///------------------------------------------------------------------------------------------
	///	SGTHRS
	///------------------------------------------------------------------------------------------
	TMC_write_only(REG_SGTHRS, 75);



}

/**
 * @brief This function sets constant speed of the lock.
 *
 */
void Lock()
{
  TMC_VACTUAL(-speed);
}

/**
 * @brief This function sets constant reverse speed of the lock.
 *
 */
void Unlock()
{
  TMC_VACTUAL(speed);
}

/**
 * @brief This function stops the lock.
 *
 */
void Stop()
{
  TMC_VACTUAL(0);
}


void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
