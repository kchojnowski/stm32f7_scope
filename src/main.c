/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_DelayUntil/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    18-November-2015
  * @brief   Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "timers.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "arm_common_tables.h"

#define SIGNAL_LENGTH	470
#define FFT_LENGTH	470

uint64_t global_tiime_ms=0;
uint64_t zoomin_counter=0;
uint64_t zoomout_counter=0;

uint16_t DMA_Buffer[4096];

typedef struct {
	GRAPH_DATA_Handle graphData;
	GRAPH_DATA_Handle graphStep;
	int16_t signal[4096];
	uint8_t ready;
} SignalData_s;

typedef SignalData_s* SignalData_p;

typedef struct {

	GRAPH_DATA_Handle graphData;
	float32_t input[1024];
	float32_t output[1024];
	uint8_t ready;
} FFTData_s;

typedef FFTData_s* FFTData_p;

GRAPH_Handle graphSignal;
GRAPH_Handle graphFFT;

TaskHandle_t GUI_ThreadId;
TaskHandle_t Signal_ThreadId;
TaskHandle_t FFT_ThreadId;

SignalData_s signalData;
FFTData_s fftData;

TimerHandle_t TouchScreenTimer;

QueueHandle_t gestureQueue;

static void GUI_Thread(void const *argument);
static void Signal_Thread(void const *argument);
static void FFT_Thread(void const *argument);
static void vTimerCallback(TimerHandle_t pxTimer);

static void SystemClock_Config(void);
//static void CPU_CACHE_Enable(void);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  //CPU_CACHE_Enable();
  HAL_Init();  
  SystemClock_Config();

  xTaskCreate(GUI_Thread, "GUI_Thread", 2048, NULL, 1, &GUI_ThreadId);
  xTaskCreate(Signal_Thread, "Signal_Thread", 512, NULL, 1, &Signal_ThreadId);
  xTaskCreate(FFT_Thread, "FFT_Thread", 512, NULL, 1, &FFT_ThreadId);
  TouchScreenTimer = xTimerCreate ("Timer", 50, pdTRUE, ( void * ) 1, vTimerCallback );
  gestureQueue = xQueueCreate(1, sizeof(MTOUCH_GestureData_s));

  vTaskStartScheduler();
  
  for(;;);
}

void zoomin() {
	zoomin_counter++;
}

void zoomout() {
	zoomout_counter++;
}


void GUI_Thread(void const *argument) {

	BSP_SDRAM_Init();
	BSP_TS_Init(LCD_GetXSize(), LCD_GetYSize());
	xTimerStart(TouchScreenTimer, 0);
	__HAL_RCC_CRC_CLK_ENABLE();
	WM_SetCreateFlags(WM_CF_MEMDEV);
	GUI_Init();
	GUI_Clear();
/*
	graphSignal = GRAPH_CreateEx(0, 0, LCD_GetXSize(), LCD_GetYSize()/2, WM_HBKWIN, WM_CF_SHOW, 0, GUI_ID_GRAPH0);
	GRAPH_SetBorder(graphSignal, 5, 5, 5, 5);
	signalData.graphStep = 1;
	signalData.graphData = GRAPH_DATA_YT_Create(GUI_RED, SIGNAL_LENGTH, NULL, 0);
	GRAPH_DATA_YT_SetAlign(signalData.graphData, GRAPH_ALIGN_LEFT);
	GRAPH_AttachData(graphSignal, signalData.graphData);

	graphFFT = GRAPH_CreateEx(0, LCD_GetYSize()/2, LCD_GetXSize(), LCD_GetYSize()/2, WM_HBKWIN, WM_CF_SHOW, 0, GUI_ID_GRAPH1);
	GRAPH_SetBorder(graphFFT, 5, 5, 5, 5);
	fftData.graphData = GRAPH_DATA_YT_Create(GUI_BLUE,FFT_LENGTH, NULL, 0);
	GRAPH_DATA_YT_SetAlign(fftData.graphData, GRAPH_ALIGN_LEFT);
	GRAPH_AttachData(graphFFT, fftData.graphData);

	xTaskNotifyGive(Signal_ThreadId);

	GUI_MTOUCH_EVENT Event;
	GUI_MTOUCH_INPUT Input;
	int8_t deltaXCurrent = 0;
	int8_t deltaXPrevious = 0;
	*/
	char str[32];
	MTOUCH_GestureData_s gestureData;
	for (;;) {
		GUI_Delay(10);
		//GUI_Clear();

		if (xQueueReceive(gestureQueue, &gestureData, 0)) {
			GUI_DispStringAt("                         ", 10, 10);
			GUI_DispStringAt("                         ", 10, 20);

			sprintf(str, "Origin [%d]", gestureData.origin_x);
			GUI_DispStringAt(str, 10, 10);

			switch (gestureData.gesture) {
			case NONE:
				GUI_DispStringAt("NONE", 10, 20);
				break;
			case MOVE_LEFT:
				GUI_DispStringAt("MOVE_LEFT", 10, 20);
				break;
			case MOVE_RIGHT:
				GUI_DispStringAt("MOVE_RIGHT", 10, 20);
				break;
			case MOVE_UP:
				GUI_DispStringAt("MOVE_UP", 10, 20);
				break;
			case MOVE_DOWN:
				GUI_DispStringAt("MOVE_DOWN", 10, 20);
				break;
			case ZOOM_IN_X:
				GUI_DispStringAt("ZOOM_IN_X", 10, 20);
				break;
			case ZOOM_OUT_X:
				GUI_DispStringAt("ZOOM_OUT_X", 10, 20);
				break;
			case ZOOM_IN_Y:
				GUI_DispStringAt("ZOOM_IN_Y", 10, 20);
				break;
			case ZOOM_OUT_Y:
				GUI_DispStringAt("ZOOM_OUT_Y", 10, 20);
				break;
			case TOUCH:
				GUI_DispStringAt("TOUCH", 10, 20);
				break;
			}
		}
	}
}

void Signal_Thread(void const *argument) {

	 ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	 BSP_AUDIO_IN_Init(INPUT_DEVICE_INPUT_LINE_1, DEFAULT_AUDIO_IN_VOLUME, DEFAULT_AUDIO_IN_FREQ);
	 BSP_AUDIO_IN_Record(DMA_Buffer, 4096);

	 signalData.ready = 1;

	 uint16_t idx1 = 0;
	 uint16_t idx2 = 0;
	 for(;;) {
		 vTaskDelay(200/portTICK_PERIOD_MS);
		 ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		 GRAPH_DATA_YT_Clear(signalData.graphData);
		 for(idx1 = 0, idx2 = 0; idx1 < SIGNAL_LENGTH && idx2<4096; idx1++, idx2+=signalData.graphStep)
			 GRAPH_DATA_YT_AddValue(signalData.graphData, signalData.signal[idx2]+60);

		 signalData.ready = 1;
	 }
}

void FFT_Thread(void const *argument) {
	arm_rfft_fast_instance_f32 fftInit;
	arm_rfft_fast_init_f32(&fftInit, 1024);

	fftData.ready = 1;

	I16 idx = 0;
	 for(;;) {
		 vTaskDelay(200/portTICK_PERIOD_MS);
		 ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		 for(idx = 0; idx < 1024; idx++)
			 fftData.input[idx] = signalData.signal[idx];

		 arm_rfft_fast_f32(&fftInit, fftData.input, fftData.output, 0);
		 arm_abs_f32(fftData.output, fftData.output, 512);
		 GRAPH_DATA_YT_Clear(fftData.graphData);
		 for(idx = 0; idx < FFT_LENGTH; idx++)
			 GRAPH_DATA_YT_AddValue(fftData.graphData, fftData.output[idx]/40);

		 fftData.ready = 1;
	 }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);

}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
/*
static void CPU_CACHE_Enable(void)
{

  SCB_EnableICache();


  SCB_EnableDCache();
}
*/
void vApplicationTickHook( void )
{
	HAL_IncTick();
	global_tiime_ms++;
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName ) {
	while (1) {

	}
}

void BSP_AUDIO_IN_TransferComplete_CallBack(void) {
	if(signalData.ready==0 || fftData.ready==0)
		return;

	signalData.ready=0;
	fftData.ready=0;

	int idx;
	for (idx = 0; idx < 4096; idx++)
		signalData.signal[idx] = DMA_Buffer[idx];

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(Signal_ThreadId, &xHigherPriorityTaskWoken);
	vTaskNotifyGiveFromISR(FFT_ThreadId, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void vTimerCallback(TimerHandle_t pxTimer) {

	TS_StateTypeDef tsState;
	MTOUCH_TouchData_s touchData;
	MTOUCH_GestureData_s gestureData;

	BSP_TS_GetState(&tsState);

	touchData.points = tsState.touchDetected;
	touchData.x[0] = tsState.touchX[0];
	touchData.x[1] = tsState.touchX[1];
	touchData.y[0] = tsState.touchY[0];
	touchData.y[1] = tsState.touchY[1];

	MTOUCH_AddTouchData(&touchData);
	MTOUCH_GetGesture(&gestureData);

	xQueueOverwrite(gestureQueue, &gestureData);

}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

