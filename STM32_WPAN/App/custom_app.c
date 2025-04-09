/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* phone_communication */
  uint8_t               Char_status_Indication_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* phone_communication */
static void Custom_Char_status_Update_Char(void);
static void Custom_Char_status_Send_Indication(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* phone_communication */
    case CUSTOM_STM_CHAR_COMMAND_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CHAR_COMMAND_WRITE_EVT */
      if(pNotification->DataTransfered.pPayload[0] == 0x00){          /* ALL Device selected - may be necessary as LB Router informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01) {
                /*HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
                APP_DBG_MSG("-- P2P APPLICATION SERVER  : Led ON\n");
                APP_DBG_MSG(" \n\r");
                P2P_Server_App_Context.LedControl.Led=0x01; /* Led ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00) {
                /*HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
                APP_DBG_MSG("-- P2P APPLICATION SERVER  : Led OFF\n");
                APP_DBG_MSG(" \n\r");
                P2P_Server_App_Context.LedControl.Led=0x00; /* Led OFF */
        }
      }
      if(pNotification->DataTransfered.pPayload[0] == 0x01){        /* end device 1 selected - may be necessary as LB Router informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01) {
                /*HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
                TMC_turn(360);
                TMC_read(REG_DRVSTATUS);
                APP_DBG_MSG("-- P2P APPLICATION SERVER 1 : Led ON\n");
                APP_DBG_MSG(" \n\r");
                P2P_Server_App_Context.LedControl.Led=0x01; /* Led ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00) {
                /*HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
                TMC_turn(-360);
                TMC_read(REG_DRVSTATUS);
                APP_DBG_MSG("-- P2P APPLICATION SERVER 1 : Led OFF\n");
                APP_DBG_MSG(" \n\r");
                P2P_Server_App_Context.LedControl.Led=0x00; /* Led OFF */
         }
      }
      /* USER CODE END CUSTOM_STM_CHAR_COMMAND_WRITE_EVT */
      break;

    case CUSTOM_STM_CHAR_STATUS_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CHAR_STATUS_INDICATE_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_CHAR_STATUS_INDICATE_ENABLED_EVT */
      break;

    case CUSTOM_STM_CHAR_STATUS_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CHAR_STATUS_INDICATE_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_CHAR_STATUS_INDICATE_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* phone_communication */
__USED void Custom_Char_status_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Char_status_UC_1*/

  /* USER CODE END Char_status_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_CHAR_STATUS, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Char_status_UC_Last*/

  /* USER CODE END Char_status_UC_Last*/
  return;
}

void Custom_Char_status_Send_Indication(void) /* Property Indication */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Char_status_IS_1*/

  /* USER CODE END Char_status_IS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_CHAR_STATUS, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Char_status_IS_Last*/

  /* USER CODE END Char_status_IS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
