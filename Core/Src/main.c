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
#include "memorymap.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint32_t sgresult_shifted = 0;
//uint32_t tstep = 0;
//uint32_t gstat = 0;
volatile static uint32_t SG_RESULT = 0;
volatile static uint32_t SGTHRS = 0;
uint32_t after = 0;
volatile static int32_t speed = 200000;
volatile static uint8_t DIAG_flag = 0;
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
  MX_RF_Init();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_LPUART1_UART_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  //Assigns correct UART to TMC_uart variable.
  TMC_UART = &hlpuart1;


  HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MS2_GPIO_Port, MS2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENN_GPIO_Port, ENN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SPREAD_GPIO_Port, SPREAD_Pin, GPIO_PIN_RESET);

  ///------------------------------------------------------------------------------------------
  /// Testing of UART communication
  ///------------------------------------------------------------------------------------------


  uint32_t test = 0x01234567;
  test = rev32(0x01234567);
  uint64_t test2 = (uint64_t)test<<32 | 0x89abcdef;
  test2 = rev64(test2);

  test = TMC_read(REG_GCONF);
  test = TMC_read(REG_IOIN);
  //TMC_write_IHOLD_IRUN(16, 15, 4);
  HAL_GPIO_WritePin(ENN_GPIO_Port, ENN_Pin, GPIO_PIN_RESET);

  uint32_t before = 0;
  //uint32_t after = 0;
  //uint32_t sgresult = 0;

  ///------------------------------------------------------------------------------------------
  ///	Setup sequence:
  ///------------------------------------------------------------------------------------------
  INIT();

  //---------------------------------------------------------
  //**NEW CODE ADDED**
  //---------------------------------------------------------
  //TIM1_CC_IRQHandler(void) generated function is in stm32wbxx_it.c
  /*
  //timer initialization
  TIM1->CR1 |= TIM_CR1_CEN;		//timer enable
  TIM1->DIER |= TIM_DIER_UIE;
  TIM1->DIER |= TIM_DIER_CC1IE;

  //change ccr1 register and therefore change rotation speed of motor
  TIM1->CCR1 = 5000;
  */

  //---------------------------------------------------------

  //---------------------------------------------------------
  //**test loop for stallguard threshold**
  //---------------------------------------------------------

  TMC_VACTUAL(speed);
  //TMC_VACTUAL(0);




/*
  //HAL_GPIO_WritePin(ENN_GPIO_Port, ENN_Pin, GPIO_PIN_SET);

  	  	  	  //execute automatic tuning procedure AT

  	  	  	  //slowly accelerate to MAX velocity e.g. VMAX

  	  	  	  //may have to tweak TMWMTHRS (disable with 0)

  	  	  	  //continue with SC2

*/

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
    SG_RESULT = TMC_read(REG_SG_RESULT);

    if(DIAG_flag == 1) {
    	HAL_GPIO_WritePin(ENN_GPIO_Port, ENN_Pin, SET);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		speed = -speed;
		HAL_Delay(1000);
		HAL_GPIO_WritePin(ENN_GPIO_Port, ENN_Pin, RESET);
		HAL_Delay(2);
		INIT();
		HAL_Delay(2);
		TMC_VACTUAL(speed);
		DIAG_flag = 0;
    }



    //change ccr1 register and therefore change rotation speed of motor
    TIM1->CCR1 = 200;


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI1
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSI;
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == DIAG_Pin) {
		DIAG_flag = 1;
	} else {
	  __NOP();
	}
}

void INIT() {
///------------------------------------------------------------------------------------------
///	Setup sequence:
///------------------------------------------------------------------------------------------


	///------------------------------------------------------------------------------------------
	///	GCONF
	///------------------------------------------------------------------------------------------
	TMC_write_bit(REG_GCONF, MASK_pdn_disable, 1);		//1 = disable Power down input/enable UART
	TMC_write_bit(REG_GCONF, MASK_i_scale_analog, 0);		//0 = disable external Vref
	TMC_write_bit(REG_GCONF, MASK_en_spreadcycle, 0);		//0 = clear en_spreadcycle in GCONF
	TMC_write_bit(REG_GCONF, MASK_internal_rsense, 0);	//0 = use external Rsense
	TMC_write_bit(REG_GCONF, MASK_mstep_reg_select, 1);	//1 = use value from MSTEP register
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
	TMC_write_bit(REG_PWMCONF, MASK_pwm_autoscale, 0b0);		//set pwm_autoscale and pwm_autograd in PWMCONF
	TMC_write_bit(REG_PWMCONF, MASK_pwm_autograd, 0b0);

	TMC_write_word(REG_PWMCONF, MASK_pwm_freq, 1);			//select PWM_FREQ in PWMCONF



	///------------------------------------------------------------------------------------------
	///	SGTHRS
	///------------------------------------------------------------------------------------------
	TMC_write_only(REG_SGTHRS, 100);



}

void delay_us (uint16_t us) {
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

#ifdef  USE_FULL_ASSERT
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
