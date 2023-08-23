/* BEGIN Header */
/**
  ******************************************************************************
  * @file    project4_freertos.c
  * @brief   FreeRTOS project operations.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 Tubitak BILGEM.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "project4_freertos.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static ACCELERO_Message_t accelero;
static ACCELERO_CTRL_REG1_t ctrl_reg1;
static uint8_t u8X,u8Y,u8Z;

static uint8_t u8Flag = 0;

static TaskHandle_t tReadAcc_Handle;
static TaskHandle_t tWriteAcc_Handle;
static TaskHandle_t tEx_Handle;

static QueueHandle_t qData;

/* Private function prototypes -----------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/******************************************************************************/
/*           				  Accelero Functions	  				          */
/******************************************************************************/

/**
  * @brief This function initialize Accerelerometer.
  */

void vAccelero_Init(void){
	ctrl_reg1.u8DR = CTRL_REG1_DR_100HZ;
	ctrl_reg1.u8PD = CTRL_REG1_PD_ACTIVE_MODE;
	ctrl_reg1.u8FS = CTRL_REG1_FS_2G;
	ctrl_reg1.u8STP_STM = CTRL_REG1_STP_STM_NORMAL_MODE;
	ctrl_reg1.u8Zen = CTRL_REG1_ZEN_ENABLED;
	ctrl_reg1.u8Yen = CTRL_REG1_YEN_ENABLED;
	ctrl_reg1.u8Xen = CTRL_REG1_XEN_ENABLED;
	uint8_t u8Data;
	vMemcpy(&u8Data,&ctrl_reg1);
	vAccelero_Write(SPI_ADDR_CONTROL_REG_1, u8Data);
}

/**
  * @brief This function copy datas in ACCELERO_CTRL_REG1_t struct to u8Data.
  */

void vMemcpy(uint8_t *u8Data, ACCELERO_CTRL_REG1_t *ctrl_reg1){
	*u8Data = (ctrl_reg1->u8DR << 7) | (ctrl_reg1->u8PD << 6) | (ctrl_reg1->u8FS << 5) | \
			(ctrl_reg1->u8STP_STM << 3) | (ctrl_reg1->u8Zen << 2) | (ctrl_reg1->u8Yen << 1) | \
			(ctrl_reg1->u8Xen << 0);
}

/**
  * @brief This function read Accerelerometer data from spesific address.
  */

uint8_t u8Accelero_Read(uint8_t u8Addr){
	accelero.u8Address = u8Addr | SPI_READ;
	accelero.u8Data = 0x00;

	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, &(accelero.u8Address), 1, 100);
	HAL_SPI_Receive(&hspi1, &(accelero.u8Data), 1, 100);

	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
	return accelero.u8Data;
}

/**
  * @brief This function write Accerelerometer data to spesific address.
  */

void vAccelero_Write(uint8_t u8Addr, uint8_t u8Data){
	accelero.u8Address = u8Addr;
	accelero.u8Address = accelero.u8Address & SPI_WRITE;
	accelero.u8Data = u8Data;

	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, &(accelero.u8Address), 1, 100);
	HAL_SPI_Transmit(&hspi1, &(accelero.u8Data), 1, 100);

	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}

/******************************************************************************/
/*           				  FreeRTOS Functions	  				          */
/******************************************************************************/

/**
  * @brief This function initialize Tasks.
  */

void vTasks_Init(void){
	if(xTaskCreate(vRead_Acc_Handler, "Task-1 Read Acc", 250, NULL, 2, &tReadAcc_Handle) != pdPASS){
		Error_Handler();
	}
	if(xTaskCreate(vWrite_Acc_Handler, "Task-2 Write Acc", 250, NULL, 2, &tWriteAcc_Handle) != pdPASS){
		Error_Handler();
	}
	vQueue_Init();
}

/**
  * @brief This function for Read Accelerometer Task.
  */

void vRead_Acc_Handler(void *vParameters){

	while(1){
		if(xQueueReset(qData) != pdPASS){
			continue;
		}

		uint8_t u8Temp = u8Accelero_Read(SPI_ADDR_STATUS_REG);

		if((u8Temp & 0x08) != 0x00){
			u8X = u8Accelero_Read(SPI_ADDR_OUTX);
			u8Y = u8Accelero_Read(SPI_ADDR_OUTY);
			u8Z = u8Accelero_Read(SPI_ADDR_OUTZ);

			char cBuffer[100] = {0};

			sprintf(cBuffer,"{\"X\":%u,\"Y\":%u,\"Z\":%u}\r\n",u8X,u8Y,u8Z);
			if(xQueueSend(qData,(void *)&cBuffer,0) != pdTRUE){
				continue;
			}
		}
	}
}

/**
  * @brief This function for Write Accelerometer Task.
  */

void vWrite_Acc_Handler(void *vParameters){
	while(1){
		if(xTaskNotifyWait(0,0,NULL,portMAX_DELAY) == pdTRUE){
			if(uxQueueMessagesWaiting(qData)){
				continue;
			}
			else{
				uint8_t *msg;
				if(xQueueReceive(qData, (void *)&msg, 0) == pdFALSE){
					continue;
				}
				CDC_Transmit_FS(msg, sizeof(msg));
			}
		}
	}
}

/**
  * @brief This function Notify Write Acc Task.
  */

void vRead_Task_Notifyer(void){
	xTaskNotify(tWriteAcc_Handle,0,eNoAction);
}

/******************************************************************************/
/*           				  	Queue Functions		  				          */
/******************************************************************************/

/**
  * @brief This function initialize Queues.
  */

void vQueue_Init(void){
	qData = xQueueCreate (100, sizeof(char));

	if(qData == NULL){
		Error_Handler();
	}
}

/******************************************************************************/
/*           				  Interrupt Functions	  				          */
/******************************************************************************/

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	vRead_Task_Notifyer();
}
