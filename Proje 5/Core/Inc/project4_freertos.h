/* BEGIN Header */
/**
  ******************************************************************************
  * @file    project4_freertos.h
  * @brief   This file contains the FreeRTOS project operations.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_PROJECT4_FREERTOS_H_
#define INC_PROJECT4_FREERTOS_H_


/* Private includes ----------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
	uint8_t u8Address;
	uint8_t u8Data;
}ACCELERO_Message_t;

typedef struct {
	uint8_t u8DR:1;
	uint8_t u8PD:1;
	uint8_t u8FS:1;
	uint8_t u8STP_STM:2;
	uint8_t u8Zen:1;
	uint8_t u8Yen:1;
	uint8_t u8Xen:1;
}ACCELERO_CTRL_REG1_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define SPI_READ						(0x80)
#define SPI_WRITE						(0x7F)

#define SPI_ADDR_OUTX					0x29
#define SPI_ADDR_OUTY					0x2B
#define SPI_ADDR_OUTZ					0x2D
#define SPI_ADDR_STATUS_REG				0x27
#define SPI_ADDR_CONTROL_REG_1			0x20
#define SPI_ADDR_INIT_CONF				0x47

#define CTRL_REG1_DR_100HZ				0
#define CTRL_REG1_DR_400HZ				1
#define CTRL_REG1_PD_POWER_DOWN_MODE	0
#define CTRL_REG1_PD_ACTIVE_MODE		1
#define CTRL_REG1_FS_2G					0
#define CTRL_REG1_FS_8G					1
#define CTRL_REG1_STP_STM_NORMAL_MODE	00
#define CTRL_REG1_STP_STM_SELF_TEST_P	10
#define CTRL_REG1_STP_STM_SELF_TEST_M	01
#define CTRL_REG1_STP_STM_FORBIDDEN		11
#define CTRL_REG1_ZEN_DISABLED			0
#define CTRL_REG1_ZEN_ENABLED			1
#define CTRL_REG1_YEN_DISABLED			0
#define CTRL_REG1_YEN_ENABLED			1
#define CTRL_REG1_XEN_DISABLED			0
#define CTRL_REG1_XEN_ENABLED			1

/* Exported functions prototypes ---------------------------------------------*/
void vAccelero_Init(void);
void vMemcpy(uint8_t *u8Data, ACCELERO_CTRL_REG1_t *ctrl_reg1);
uint8_t u8Accelero_Read(uint8_t u8Addr);
void vAccelero_Write(uint8_t u8Addr, uint8_t data);

void vTasks_Init(void);
void vRead_Acc_Handler(void *vParameters);
void vWrite_Acc_Handler(void *vParameters);
void vRead_Task_Notifyer(void);

void vQueue_Init(void);

#endif /* INC_PROJECT4_FREERTOS_H_ */
