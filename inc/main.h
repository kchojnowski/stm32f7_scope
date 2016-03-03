  /******************************************************************************
  * @file    FreeRTOS/FreeRTOS_DelayUntil/Inc/main.h
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    18-November-2015 
  * @brief   This file contains all the functions prototypes for the main.c 
  *          file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_i2s.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_sdram.h"
#include "stm32746g_discovery_audio.h"
#include "stm32746g_discovery_ts.h"
#include "wm8994.h"
#include "GUI.h"
#include "WM.h"
#include "GRAPH.h"
#include "mtouch.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "arm_common_tables.h"

#define SIGNAL_LENGTH	470
#define FFT_LENGTH		470
#define GRAPH_RANGE_Y 	120
#define GRAPH_OFFSET_Y 	2

#define TASK_EVENT_DMA_HALF_DONE 			0x00000001
#define TASK_EVENT_DMA_DONE 				0x00000002
#define TASK_EVENT_CHANGE_VIEW_MOVE_LEFT	0x00000100
#define TASK_EVENT_CHANGE_VIEW_MOVE_RIGHT	0x00000200
#define TASK_EVENT_CHANGE_VIEW_MOVE_UP		0x00000400
#define TASK_EVENT_CHANGE_VIEW_MOVE_DOWN	0x00000800
#define TASK_EVENT_CHANGE_VIEW_ZOOM_IN_X	0x00001000
#define TASK_EVENT_CHANGE_VIEW_ZOOM_OUT_X	0x00002000
#define TASK_EVENT_CHANGE_VIEW_ZOOM_IN_Y	0x00004000
#define TASK_EVENT_CHANGE_VIEW_ZOOM_OUT_Y	0x00008000



#define DMA_BUFFER_LENGTH			16384
#define SIGNAL_SAMPLES				DMA_BUFFER_LENGTH/4

typedef struct {
	TaskHandle_t guiTaskId;
	TaskHandle_t signalTaskId;
	TaskHandle_t fftTaskId;
	QueueHandle_t gestureQueue;
	TimerHandle_t touchPanelTimer;
	int32_t	touchPanelTimerId;

	GRAPH_Handle signalGraph;
	GRAPH_DATA_Handle signalGraphData;
	GRAPH_Handle fftGraph;
	GRAPH_DATA_Handle fftGraphData;
	GRAPH_SCALE_Handle fftGraphScale;

	int16_t dmaBuffer[DMA_BUFFER_LENGTH];
	float32_t fftInput[SIGNAL_SAMPLES];
	float32_t fftOutput[SIGNAL_SAMPLES];
	int16_t signalDisplay[SIGNAL_SAMPLES];
	int16_t fftDisplay[SIGNAL_LENGTH];
} AppGlobals_s;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

