/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
enum {
	emR, emG, emB, emHZ, emS, em0D, em0A
};
//7

#define LEDNUM 8

#define ENDMARK_0D 0x0D
#define ENDMARK_0A 0x0A

#define BREATH_WHITE     0x01
#define BREATH_GREEN     0x02

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId DefaultTaskHandle;
osThreadId MotorTaskHandle;
osThreadId MonitorTaskHandle;
osThreadId EncoderTaskHandle;
osThreadId BreathingTaskHandle;
osSemaphoreId serialSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void startDefaultTask(void const * argument);
void startMotorTask(void const * argument);
void startMonitorTask(void const * argument);
void startEncoderTask(void const * argument);
void startBreathingTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* definition and creation of serialSem */
	osSemaphoreDef(serialSem);
	serialSemHandle = osSemaphoreCreate(osSemaphore(serialSem), 1);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of DefaultTask */
	osThreadDef(DefaultTask, startDefaultTask, osPriorityNormal, 0, 128);
	DefaultTaskHandle = osThreadCreate(osThread(DefaultTask), NULL);

	/* definition and creation of MotorTask */
	osThreadDef(MotorTask, startMotorTask, osPriorityIdle, 0, 128);
	MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

	/* definition and creation of MonitorTask */
	osThreadDef(MonitorTask, startMonitorTask, osPriorityIdle, 0, 128);
	MonitorTaskHandle = osThreadCreate(osThread(MonitorTask), NULL);

	/* definition and creation of EncoderTask */
	osThreadDef(EncoderTask, startEncoderTask, osPriorityIdle, 0, 128);
	EncoderTaskHandle = osThreadCreate(osThread(EncoderTask), NULL);

	/* definition and creation of BreathingTask */
	osThreadDef(BreathingTask, startBreathingTask, osPriorityIdle, 0, 128);
	BreathingTaskHandle = osThreadCreate(osThread(BreathingTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_startDefaultTask */
/**
 * @brief  Function implementing the DefaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_startDefaultTask */
void startDefaultTask(void const * argument) {

	/* USER CODE BEGIN startDefaultTask */
	/* Infinite loop */
	for (;;) {
//		LL_TIM_GetCounter(TIM3);
		osDelay(1000);
	}
	/* USER CODE END startDefaultTask */
}

/* USER CODE BEGIN Header_startMotorTask */
/**
 * @brief Function implementing the motorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startMotorTask */
void startMotorTask(void const * argument) {
	/* USER CODE BEGIN startMotorTask */
	/* Infinite loop */
	for (;;) {
//		LL_TIM_OC_SetCompareCH2(TIM2, 2400);
//		osDelay(2000);
//		LL_TIM_OC_SetCompareCH2(TIM2, 3600);
//		osDelay(2000);
//		LL_TIM_OC_SetCompareCH2(TIM2, 4800);
//		osDelay(2000);
//		LL_TIM_OC_SetCompareCH2(TIM2, 6000);
//		osDelay(2000);
		LL_TIM_OC_SetCompareCH2(TIM2, 7200);
		osDelay(2000);
	}
	/* USER CODE END startMotorTask */
}

/* USER CODE BEGIN Header_startMonitorTask */
/**
 * @brief Function implementing the MonitorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startMonitorTask */
void startMonitorTask(void const * argument) {
	/* USER CODE BEGIN startMonitorTask */
	/* Infinite loop */
	volatile uint32_t encode = 0;
	for (;;) {
//		encode = LL_TIM_GetCounter(TIM2);
//		LL_TIM_SetCounter(TIM2, 0);

//		osSemaphoreWait(serialSemHandle, portMAX_DELAY);
//		printf("TIM2 encode: %lu\n", encode);
//		osSemaphoreRelease(serialSemHandle);

		osDelay(250);
	}
	/* USER CODE END startMonitorTask */
}

/* USER CODE BEGIN Header_startEncoderTask */
/**
 * @brief Function implementing the EncoderTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startEncoderTask */
void startEncoderTask(void const * argument) {
	/* USER CODE BEGIN startEncoderTask */
	/* Infinite loop */
	uint32_t cnt = 0;
	for (;;) {
		cnt = LL_TIM_GetCounter(TIM3);
//		LL_TIM_SetCounter(TIM3, 0);

		osSemaphoreWait(serialSemHandle, portMAX_DELAY);
//		printf("TIM3->CNT: %lu\n", cnt);
		osSemaphoreRelease(serialSemHandle);

		osDelay(250);
	}
	/* USER CODE END startEncoderTask */
}

/* USER CODE BEGIN Header_startBreathingTask */
/**
 * @brief Function implementing the BreathingTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startBreathingTask */
void startBreathingTask(void const * argument) {
	/* USER CODE BEGIN startBreathingTask */
	/* Infinite loop */
	//    uint8_t red = 0;
	uint8_t green = 0, dirc = 1;
	//    uint8_t blue = 0;
	for (;;) {
		//MY_DEBUG_PRINT_INFO ( "Breathing_Task\r\n" );
		osSemaphoreWait(serialSemHandle, portMAX_DELAY);
//		printf("TIM3->CNT: %lu\n", cnt);

		if (dirc == 0) {

			green += 5;
			SetWholeColor( GROUP_B, 0, green, 0);
			WS2812_update( GROUP_B);

			if (green == 255)
				dirc = 1;
		} else {
			green -= 5;
			SetWholeColor( GROUP_B, 0, green, 0);
			WS2812_update( GROUP_B);

			if (green == 0)
				dirc = 0;

		}

		osDelay(10); //延时10ms
		osSemaphoreRelease(serialSemHandle);
		osDelay(10); //延时10ms
	}
	/* USER CODE END startBreathingTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**************************************************************************
 function: car motion model
 param: velocity, angle
 **************************************************************************/
void Kinematic_Analysis(float velocity, float angle) {
//	Target_A = velocity * (1 + T * tan(angle) / 2 / L);
//	Target_B = velocity * (1 - T * tan(angle) / 2 / L);      //后轮差�??
//	Servo = SERVO_INIT + angle * K;                    //舵机转向
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
