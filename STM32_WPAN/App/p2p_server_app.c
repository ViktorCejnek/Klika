/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/p2p_server_app.c
  * @author  MCD Application Team
  * @brief   Peer to peer Server Application
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
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "p2p_server_app.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint8_t				Device_Led_Selection;
	uint8_t				Led;
} P2P_LedCharValue_t;

typedef struct{
	uint8_t             Device_Button_Selection;
	uint8_t             ButtonStatus;
} P2P_ButtonCharValue_t;

typedef struct
{
	uint8_t               Notification_Status; /* used to check if P2P Server is enabled to Notify */
	P2P_LedCharValue_t    LedControl;
	P2P_ButtonCharValue_t ButtonControl;
	uint16_t              ConnectionHandle;
} P2P_Server_App_Context_t;
/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static P2P_Server_App_Context_t P2P_Server_App_Context;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_STM_App_Notification_1 */

/* USER CODE END P2PS_STM_App_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_STM_App_Notification_P2P_Evt_Opcode */
	#if(BLE_CFG_OTA_REBOOT_CHAR != 0)
		case P2PS_STM_BOOT_REQUEST_EVT:
			APP_DBG_MSG("-- P2P APPLICATION SERVER : BOOT REQUESTED\n");
			APP_DBG_MSG(" \n\r");

			*(uint32_t*)SRAM1_BASE = *(uint32_t*)pNotification->DataTransfered.pPayload;
			NVIC_SystemReset();
		break;
	#endif
/* USER CODE END P2PS_STM_App_Notification_P2P_Evt_Opcode */

    case P2PS_STM__NOTIFY_ENABLED_EVT:
/* USER CODE BEGIN P2PS_STM__NOTIFY_ENABLED_EVT */
		P2P_Server_App_Context.Notification_Status = 1;
		APP_DBG_MSG("-- P2P APPLICATION SERVER : NOTIFICATION ENABLED\n");
		APP_DBG_MSG(" \n\r");
/* USER CODE END P2PS_STM__NOTIFY_ENABLED_EVT */
      break;

    case P2PS_STM_NOTIFY_DISABLED_EVT:
/* USER CODE BEGIN P2PS_STM_NOTIFY_DISABLED_EVT */
		P2P_Server_App_Context.Notification_Status = 0;
		APP_DBG_MSG("-- P2P APPLICATION SERVER : NOTIFICATION DISABLED\n");
		APP_DBG_MSG(" \n\r");
/* USER CODE END P2PS_STM_NOTIFY_DISABLED_EVT */
      break;

    case P2PS_STM_WRITE_EVT:
/* USER CODE BEGIN P2PS_STM_WRITE_EVT */
    	if(pNotification->DataTransfered.pPayload[0] == 0x00){					/* ALL Device selected - may be necessary as LB Router informs all connection */
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

		#if(P2P_SERVER1 != 0)
    		if(pNotification->DataTransfered.pPayload[0] == 0x01){ 				/* end device 1 selected - may be necessary as LB Router informs all connection */
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
		#endif
/* USER CODE END P2PS_STM_WRITE_EVT */
      break;

    default:
/* USER CODE BEGIN P2PS_STM_App_Notification_default */

/* USER CODE END P2PS_STM_App_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_STM_App_Notification_2 */

/* USER CODE END P2PS_STM_App_Notification_2 */
  return;
}

void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_APP_Notification_1 */

/* USER CODE END P2PS_APP_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_APP_Notification_P2P_Evt_Opcode */

/* USER CODE END P2PS_APP_Notification_P2P_Evt_Opcode */
  case PEER_CONN_HANDLE_EVT :
/* USER CODE BEGIN PEER_CONN_HANDLE_EVT */

/* USER CODE END PEER_CONN_HANDLE_EVT */
    break;

    case PEER_DISCON_HANDLE_EVT :
/* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */

/* USER CODE END PEER_DISCON_HANDLE_EVT */
    break;

    default:
/* USER CODE BEGIN P2PS_APP_Notification_default */

/* USER CODE END P2PS_APP_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_APP_Notification_2 */

/* USER CODE END P2PS_APP_Notification_2 */
  return;
}

void P2PS_APP_Init(void)
{
/* USER CODE BEGIN P2PS_APP_Init */

/* USER CODE END P2PS_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
