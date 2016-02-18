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

#include "main.h"

static void GUI_Thread(void const *argument);
static void Signal_Thread(void const *argument);
static void FFT_Thread(void const *argument);
static void vTouchTimerCallback(TimerHandle_t pxTimer);
static void SystemClock_Config(void);
//static void CPU_CACHE_Enable(void);

static AppGlobals_s appGlobals;
uint64_t timeDMAHalf = 0;
uint64_t timeDMAFull = 0;
uint64_t timeSignalDMAHalf = 0;
uint64_t timeSignalDMAFull = 0;
uint64_t timeFFTDMAHalf = 0;
uint64_t timeFFTDMAFull = 0;
uint64_t timeSignalDMAHalfStop = 0;
uint64_t timeSignalDMAFullStop = 0;
uint64_t timeFFTDMAHalfStop = 0;
uint64_t timeFFTDMAFullStop = 0;
uint64_t globalTime=0;
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

  xTaskCreate(GUI_Thread, "GUI_Thread", 512, NULL, 1, &appGlobals.guiThreadId);
  xTaskCreate(Signal_Thread, "Signal_Thread", 1024, NULL, 1, &appGlobals.signalThreadId);
  xTaskCreate(FFT_Thread, "FFT_Thread", 1024, NULL, 1, &appGlobals.fftThreadId);
  appGlobals.touchScreenTimer = xTimerCreate ("Timer", 100, pdTRUE, ( void * ) 1, vTouchTimerCallback );
  appGlobals.gestureQueue = xQueueCreate(1, sizeof(MTOUCH_GestureData_s));

  vTaskStartScheduler();
  
  for(;;);
}

void GUI_Thread(void const *arg) {

	BSP_SDRAM_Init();
	BSP_TS_Init(LCD_GetXSize(), LCD_GetYSize());
	xTimerStart(appGlobals.touchScreenTimer, 0);

	__HAL_RCC_CRC_CLK_ENABLE();
	WM_SetCreateFlags(WM_CF_MEMDEV | WM_CF_MEMDEV_ON_REDRAW);
	GUI_Init();
	GUI_Clear();

	appGlobals.signalGraph = GRAPH_CreateEx(0, 0, LCD_GetXSize(), LCD_GetYSize()/2, WM_HBKWIN, WM_CF_SHOW, 0, GUI_ID_GRAPH0);
	GRAPH_SetBorder(appGlobals.signalGraph, 5, 5, 5, 5);
	appGlobals.signalGraphData = GRAPH_DATA_YT_Create(GUI_RED, SIGNAL_LENGTH, NULL, 0);
	GRAPH_DATA_YT_SetAlign(appGlobals.signalGraphData, GRAPH_ALIGN_LEFT);
	GRAPH_AttachData(appGlobals.signalGraph, appGlobals.signalGraphData);

	appGlobals.fftGraph = GRAPH_CreateEx(0, LCD_GetYSize()/2, LCD_GetXSize(), LCD_GetYSize()/2, WM_HBKWIN, WM_CF_SHOW, 0, GUI_ID_GRAPH1);
	GRAPH_SetBorder(appGlobals.fftGraph, 5, 5, 5, 5);
	appGlobals.fftGraphScale = GRAPH_SCALE_Create(LCD_GetYSize()/2-12, GUI_TA_LEFT, GRAPH_SCALE_CF_HORIZONTAL, 50);
	GRAPH_SCALE_SetFactor(appGlobals.fftGraphScale, (2.0*DEFAULT_AUDIO_IN_FREQ/DMA_BUFFER_LENGTH));
	GRAPH_AttachScale(appGlobals.fftGraph, appGlobals.fftGraphScale);
	appGlobals.fftGraphData = GRAPH_DATA_YT_Create(GUI_BLUE,FFT_LENGTH, NULL, 0);
	GRAPH_DATA_YT_SetAlign(appGlobals.fftGraphData, GRAPH_ALIGN_LEFT);
	GRAPH_AttachData(appGlobals.fftGraph, appGlobals.fftGraphData);

	xTaskNotifyGive(appGlobals.signalThreadId);

	//char str[32];
	MTOUCH_GestureData_s gestureData;
	for (;;) {
		GUI_Delay(10);
		if (xQueueReceive(appGlobals.gestureQueue, &gestureData, 0)) {
			if (gestureData.origin_y > LCD_GetYSize() / 2) {
				switch (gestureData.gesture) {
				case MOVE_LEFT:
					xTaskNotify(appGlobals.fftThreadId, TASK_EVENT_CHANGE_VIEW_MOVE_LEFT, eSetValueWithOverwrite);
					break;
				case MOVE_RIGHT:
					xTaskNotify(appGlobals.fftThreadId, TASK_EVENT_CHANGE_VIEW_MOVE_RIGHT, eSetValueWithOverwrite);
					break;
				case ZOOM_IN_X:
					xTaskNotify(appGlobals.fftThreadId, TASK_EVENT_CHANGE_VIEW_ZOOM_IN_X, eSetValueWithOverwrite);
					break;
				case ZOOM_OUT_X:
					xTaskNotify(appGlobals.fftThreadId, TASK_EVENT_CHANGE_VIEW_ZOOM_OUT_X, eSetValueWithOverwrite);
					break;
				default:
					break;
				}
			} else {
				switch (gestureData.gesture) {
				case MOVE_LEFT:
					xTaskNotify(appGlobals.signalThreadId, TASK_EVENT_CHANGE_VIEW_MOVE_LEFT, eSetValueWithOverwrite);
					break;
				case MOVE_RIGHT:
					xTaskNotify(appGlobals.signalThreadId, TASK_EVENT_CHANGE_VIEW_MOVE_RIGHT, eSetValueWithOverwrite);
					break;
				case ZOOM_IN_X:
					xTaskNotify(appGlobals.signalThreadId, TASK_EVENT_CHANGE_VIEW_ZOOM_IN_X, eSetValueWithOverwrite);
					break;
				case ZOOM_OUT_X:
					xTaskNotify(appGlobals.signalThreadId, TASK_EVENT_CHANGE_VIEW_ZOOM_OUT_X, eSetValueWithOverwrite);
					break;
				default:
					break;
				}
			}
		}
	}
}

static void scaleAxisXFloat(float32_t* tab, uint32_t len, uint32_t scale) {
	uint32_t index;
	for (index = 0; index < len/scale; index++)
		arm_mean_f32(tab + (index * scale), scale, tab+index);
}

static void scaleAxisYFloat(float32_t* tab, uint32_t len) {
	float32_t min;
	float32_t max;
	uint32_t index;

	arm_max_f32(tab, len, &max, &index);
	arm_min_f32(tab, len, &min, &index);

	for (index = 0; index < len; index++)
		tab[index] = (tab[index]-min)*GRAPH_RANGE_Y/(max-min)+GRAPH_OFFSET_Y;
}

static void scaleAxisXInt(int16_t* tab, uint32_t len, uint32_t scale) {
	uint32_t index;
	for (index = 0; index < len/scale; index++)
		arm_mean_q15(tab + (index * scale), scale, tab+index);
}

static void scaleAxisYInt(int16_t* tab, uint32_t len) {
	int16_t min;
	int16_t max;
	uint32_t index;

	arm_max_q15(tab, len, &max, &index);
	arm_min_q15(tab, len, &min, &index);

	for (index = 0; index < len; index++)
		tab[index] = (tab[index]-min)*GRAPH_RANGE_Y/(max-min)+GRAPH_OFFSET_Y;
}

void Signal_Thread(void const *arg) {

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	BSP_AUDIO_IN_Init(INPUT_DEVICE_INPUT_LINE_1, DEFAULT_AUDIO_IN_VOLUME, DEFAULT_AUDIO_IN_FREQ);
	BSP_AUDIO_IN_Record((uint16_t*)appGlobals.dmaBuffer, DMA_BUFFER_LENGTH);

	timeDMAHalf = 0;
	timeDMAFull = 0;
	timeSignalDMAHalf = 0;
	timeSignalDMAFull = 0;
	timeFFTDMAHalf = 0;
	timeFFTDMAFull = 0;
	timeSignalDMAHalfStop = 0;
	timeSignalDMAFullStop = 0;
	timeFFTDMAHalfStop = 0;
	timeFFTDMAFullStop = 0;
	globalTime = 0;

	int16_t displayOffsetX = 0;
	int16_t displayScaleX = 1;

	uint32_t notificationValue;
	uint16_t idx;
	for (;;) {
		if (xTaskNotifyWait(0, UINT32_MAX, &notificationValue, portMAX_DELAY)) {
			switch (notificationValue) {
			case TASK_EVENT_DMA_HALF_DONE:
				timeSignalDMAHalf = globalTime;
				for (idx = 0; idx < SIGNAL_SAMPLES; idx++)
					appGlobals.signalDisplay[idx] = appGlobals.dmaBuffer[idx*2];

				scaleAxisXInt(appGlobals.signalDisplay, SIGNAL_SAMPLES, displayScaleX);
				scaleAxisYInt(appGlobals.signalDisplay, SIGNAL_SAMPLES/displayScaleX);

				for (idx = 0; idx < SIGNAL_LENGTH; idx++)
					appGlobals.signalDisplay[idx] = appGlobals.signalDisplay[idx+displayOffsetX];

 				GRAPH_DetachData(appGlobals.signalGraph, appGlobals.signalGraphData);
				GRAPH_DATA_YT_Delete(appGlobals.signalGraphData);
				appGlobals.signalGraphData = GRAPH_DATA_YT_Create(GUI_RED, SIGNAL_LENGTH, appGlobals.signalDisplay, SIGNAL_LENGTH);
				GRAPH_AttachData(appGlobals.signalGraph, appGlobals.signalGraphData);
				timeSignalDMAHalfStop= globalTime;
				break;

			case TASK_EVENT_DMA_DONE:
				timeSignalDMAFull = globalTime;
				for (idx = 0; idx < SIGNAL_SAMPLES; idx++)
					appGlobals.signalDisplay[idx] = appGlobals.dmaBuffer[DMA_BUFFER_LENGTH/2 + idx*2];

				scaleAxisXInt(appGlobals.signalDisplay, SIGNAL_SAMPLES, displayScaleX);
				scaleAxisYInt(appGlobals.signalDisplay, SIGNAL_SAMPLES/displayScaleX);

				for (idx = 0; idx < SIGNAL_LENGTH; idx++)
					appGlobals.signalDisplay[idx] = appGlobals.signalDisplay[idx+displayOffsetX];

				GRAPH_DetachData(appGlobals.signalGraph, appGlobals.signalGraphData);
				GRAPH_DATA_YT_Delete(appGlobals.signalGraphData);
				appGlobals.signalGraphData = GRAPH_DATA_YT_Create(GUI_RED, SIGNAL_LENGTH,  appGlobals.signalDisplay, SIGNAL_LENGTH);
				GRAPH_AttachData(appGlobals.signalGraph, appGlobals.signalGraphData);
				timeSignalDMAFullStop = globalTime;
				break;
			case TASK_EVENT_CHANGE_VIEW_MOVE_LEFT:
				if((SIGNAL_LENGTH-1+displayOffsetX+5)*(displayScaleX)<SIGNAL_SAMPLES)
					displayOffsetX+= 5;
				break;
			case TASK_EVENT_CHANGE_VIEW_MOVE_RIGHT:
				if(displayOffsetX >= 5)
					displayOffsetX-=5;
				break;
			case TASK_EVENT_CHANGE_VIEW_ZOOM_IN_X:
				if(displayScaleX>1)
					displayScaleX--;
				break;
			case TASK_EVENT_CHANGE_VIEW_ZOOM_OUT_X:
				if((FFT_LENGTH-1+displayOffsetX)*(displayScaleX+1)<SIGNAL_SAMPLES)
					displayScaleX++;
				break;
			}
		}
	 }
}

void FFT_Thread(void const *arg) {

	arm_rfft_fast_instance_f32 fftInit;
	arm_rfft_fast_init_f32(&fftInit, SIGNAL_SAMPLES);

	int16_t displayOffsetX = 0;
	int16_t displayScaleX = 1;

	uint32_t notificationValue;
	uint16_t idx;
	float32_t scaleFactor = 2.0*DEFAULT_AUDIO_IN_FREQ/DMA_BUFFER_LENGTH;
	for (;;) {

		if (xTaskNotifyWait(0, UINT32_MAX, &notificationValue, portMAX_DELAY)) {
			switch (notificationValue) {
			case TASK_EVENT_DMA_HALF_DONE:
				timeFFTDMAHalf = globalTime;
				for(idx=0; idx<SIGNAL_SAMPLES; idx++)
					appGlobals.fftInput[idx] = appGlobals.dmaBuffer[idx*2];
				arm_rfft_fast_f32(&fftInit, appGlobals.fftInput, appGlobals.fftOutput, 0);
				arm_abs_f32(appGlobals.fftOutput, appGlobals.fftOutput, SIGNAL_SAMPLES);

				scaleAxisXFloat(appGlobals.fftOutput, SIGNAL_SAMPLES, displayScaleX);
				scaleAxisYFloat(appGlobals.fftOutput, SIGNAL_SAMPLES/displayScaleX);
				for (idx = 0; idx < FFT_LENGTH; idx++) {
					appGlobals.fftDisplay[idx] = appGlobals.fftOutput[idx+displayOffsetX/displayScaleX];
				}

				GRAPH_DetachData(appGlobals.fftGraph, appGlobals.fftGraphData);
				GRAPH_DATA_YT_Delete(appGlobals.fftGraphData);
				appGlobals.fftGraphData = GRAPH_DATA_YT_Create(GUI_BLUE, FFT_LENGTH, appGlobals.fftDisplay, FFT_LENGTH);
				GRAPH_AttachData(appGlobals.fftGraph, appGlobals.fftGraphData);
				timeFFTDMAHalfStop = globalTime;
				break;
			case TASK_EVENT_DMA_DONE:
				timeFFTDMAFull = globalTime;
				for(idx=0; idx<SIGNAL_SAMPLES; idx++)
					appGlobals.fftInput[idx] = appGlobals.dmaBuffer[DMA_BUFFER_LENGTH/2+idx*2];
				arm_rfft_fast_f32(&fftInit, appGlobals.fftInput, appGlobals.fftOutput, 0);
				arm_abs_f32(appGlobals.fftOutput, appGlobals.fftOutput, SIGNAL_SAMPLES);

				scaleAxisXFloat(appGlobals.fftOutput, SIGNAL_SAMPLES, displayScaleX);
				scaleAxisYFloat(appGlobals.fftOutput, SIGNAL_SAMPLES/displayScaleX);
				for (idx = 0; idx < FFT_LENGTH; idx++) {
					appGlobals.fftDisplay[idx] = appGlobals.fftOutput[idx+displayOffsetX/displayScaleX];
				}

				GRAPH_DetachData(appGlobals.fftGraph, appGlobals.fftGraphData);
				GRAPH_DATA_YT_Delete(appGlobals.fftGraphData);
				appGlobals.fftGraphData = GRAPH_DATA_YT_Create(GUI_BLUE, FFT_LENGTH, appGlobals.fftDisplay, FFT_LENGTH);
				GRAPH_AttachData(appGlobals.fftGraph, appGlobals.fftGraphData);
				timeFFTDMAFullStop = globalTime;
				break;
			case TASK_EVENT_CHANGE_VIEW_MOVE_LEFT:
				 if(displayOffsetX + displayScaleX*(FFT_LENGTH+10)<SIGNAL_SAMPLES) {
					displayOffsetX+= 10*displayScaleX;
					GRAPH_SCALE_SetOff(appGlobals.fftGraphScale, -1*displayOffsetX/displayScaleX);
				}
				break;
			case TASK_EVENT_CHANGE_VIEW_MOVE_RIGHT:
				if(displayOffsetX >= 10*displayScaleX) {
					displayOffsetX-=10*displayScaleX;
					GRAPH_SCALE_SetOff(appGlobals.fftGraphScale, -1*displayOffsetX/displayScaleX);
				}
				break;
			case TASK_EVENT_CHANGE_VIEW_ZOOM_IN_X:
				if(displayScaleX>1) {
					displayOffsetX*=scaleFactor;
					displayOffsetX += FFT_LENGTH*displayScaleX/4;
					displayScaleX--;
					displayOffsetX /= scaleFactor;

					GRAPH_SCALE_SetFactor(appGlobals.fftGraphScale,  scaleFactor*displayScaleX);
					GRAPH_SCALE_SetOff(appGlobals.fftGraphScale, -1*displayOffsetX/displayScaleX);
				}
				break;
			case TASK_EVENT_CHANGE_VIEW_ZOOM_OUT_X:
				if((FFT_LENGTH-1)*(displayScaleX+1)<SIGNAL_SAMPLES) {
					displayOffsetX *= scaleFactor;
					displayOffsetX -= FFT_LENGTH * displayScaleX / 2;
					displayScaleX++;
					displayOffsetX /= scaleFactor;

					if(displayOffsetX<0)
						displayOffsetX=0;
					else if(displayScaleX*(displayOffsetX+FFT_LENGTH)>=SIGNAL_SAMPLES)
						displayOffsetX = SIGNAL_SAMPLES/displayScaleX-FFT_LENGTH;
					GRAPH_SCALE_SetFactor(appGlobals.fftGraphScale,  scaleFactor*displayScaleX);
					GRAPH_SCALE_SetOff(appGlobals.fftGraphScale, -1*displayOffsetX/displayScaleX);
				}
				break;
			}
		}
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

  SystemCoreClockUpdate();

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
	globalTime++;
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName ) {
	while (1) {

	}
}

void BSP_AUDIO_IN_HalfTransfer_CallBack(void) {

	timeDMAHalf = globalTime;
	//uint16_t idx;
	//for(idx=0; idx<DMA_BUFFER_LENGTH/4; idx++) {
	//	appGlobals.dmaBuffer[idx*2] = sinTable_q15[(idx*10)%512]/1000 + sinTable_q15[(idx*20)%512]/1000;
	//	appGlobals.dmaBuffer[idx*2+1] = 1;
	//}

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(appGlobals.fftThreadId, TASK_EVENT_DMA_HALF_DONE, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
	xTaskNotifyFromISR(appGlobals.signalThreadId, TASK_EVENT_DMA_HALF_DONE, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void BSP_AUDIO_IN_TransferComplete_CallBack(void) {
	timeDMAFull = globalTime;
	//uint16_t idx;
	//for(idx=0; idx<DMA_BUFFER_LENGTH/4; idx++) {
	//	appGlobals.dmaBuffer[DMA_BUFFER_LENGTH/2+idx*2] = (sinTable_q15[(idx*10)%512]/1000 + sinTable_q15[(idx*20)%512]/1000)*-1;
	//	appGlobals.dmaBuffer[DMA_BUFFER_LENGTH/2+idx*2+1] = 1;
	//}
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(appGlobals.fftThreadId, TASK_EVENT_DMA_DONE, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
	xTaskNotifyFromISR(appGlobals.signalThreadId, TASK_EVENT_DMA_DONE, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void vTouchTimerCallback(TimerHandle_t pxTimer) {

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

	xQueueOverwrite(appGlobals.gestureQueue, &gestureData);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

