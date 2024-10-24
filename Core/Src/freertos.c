/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "hal.h"
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
/* USER CODE BEGIN Variables */
static bool initialed_task=false;
static uint32_t time_ms=0;

/* USER CODE END Variables */
/* Definitions for initTask */
osThreadId_t initTaskHandle;
const osThreadAttr_t initTask_attributes = {
  .name = "initTask",
  .stack_size = 600 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for loop400hzTask */
osThreadId_t loop400hzTaskHandle;
const osThreadAttr_t loop400hzTask_attributes = {
  .name = "loop400hzTask",
  .stack_size = 600 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for loop200hzTask */
osThreadId_t loop200hzTaskHandle;
const osThreadAttr_t loop200hzTask_attributes = {
  .name = "loop200hzTask",
  .stack_size = 600 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for loop100hzTask */
osThreadId_t loop100hzTaskHandle;
const osThreadAttr_t loop100hzTask_attributes = {
  .name = "loop100hzTask",
  .stack_size = 600 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for loop50hzTask */
osThreadId_t loop50hzTaskHandle;
const osThreadAttr_t loop50hzTask_attributes = {
  .name = "loop50hzTask",
  .stack_size = 500 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for heartbeatTask */
osThreadId_t heartbeatTaskHandle;
const osThreadAttr_t heartbeatTask_attributes = {
  .name = "heartbeatTask",
  .stack_size = 500 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mavSendTask */
osThreadId_t mavSendTaskHandle;
const osThreadAttr_t mavSendTask_attributes = {
  .name = "mavSendTask",
  .stack_size = 500 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for buzzerTask */
osThreadId_t buzzerTaskHandle;
const osThreadAttr_t buzzerTask_attributes = {
  .name = "buzzerTask",
  .stack_size = 500 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sdLogTask */
osThreadId_t sdLogTaskHandle;
const osThreadAttr_t sdLogTask_attributes = {
  .name = "sdLogTask",
  .stack_size = 1500 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gnssTask */
osThreadId_t gnssTaskHandle;
const osThreadAttr_t gnssTask_attributes = {
  .name = "gnssTask",
  .stack_size = 500 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uwbTask */
osThreadId_t uwbTaskHandle;
const osThreadAttr_t uwbTask_attributes = {
  .name = "uwbTask",
  .stack_size = 500 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 500 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for magTask */
osThreadId_t magTaskHandle;
const osThreadAttr_t magTask_attributes = {
  .name = "magTask",
  .stack_size = 500 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for offboardTask */
osThreadId_t offboardTaskHandle;
const osThreadAttr_t offboardTask_attributes = {
  .name = "offboardTask",
  .stack_size = 500 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/// (1) Declare task handler
osThreadId_t idleTaskHandle;
/// (2) Define task attributes
const osThreadAttr_t idleTask_attributes = {
  .name = "idleTask",							//task name
  .stack_size = 400 * 4,						//how many bytes the running task using
  .priority = (osPriority_t) osPriorityNormal,	//task priority: do not higher than osPriorityNormal(larger value means higher priority)
};
/// (3) Declare task function
void IdleTask(void *argument);

/* *************************************************
 * ****************Dev code begin*******************/
// Warning! Developer can add your new code here!

/* ****************Dev code end*********************
 * *************************************************/

/* USER CODE END FunctionPrototypes */

void InitTask(void *argument);
void Loop400hzTask(void *argument);
void Loop200hzTask(void *argument);
void Loop100hzTask(void *argument);
void Loop50hzTask(void *argument);
void HeartbeatTask(void *argument);
void MavSendTask(void *argument);
void BuzzerTask(void *argument);
void SDLogTask(void *argument);
void GnssTask(void *argument);
void UWBTask(void *argument);
void IMUTask(void *argument);
void MagTask(void *argument);
void OffboardTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of initTask */
  initTaskHandle = osThreadNew(InitTask, NULL, &initTask_attributes);

  /* creation of loop400hzTask */
  loop400hzTaskHandle = osThreadNew(Loop400hzTask, NULL, &loop400hzTask_attributes);

  /* creation of loop200hzTask */
  loop200hzTaskHandle = osThreadNew(Loop200hzTask, NULL, &loop200hzTask_attributes);

  /* creation of loop100hzTask */
  loop100hzTaskHandle = osThreadNew(Loop100hzTask, NULL, &loop100hzTask_attributes);

  /* creation of loop50hzTask */
  loop50hzTaskHandle = osThreadNew(Loop50hzTask, NULL, &loop50hzTask_attributes);

  /* creation of heartbeatTask */
  heartbeatTaskHandle = osThreadNew(HeartbeatTask, NULL, &heartbeatTask_attributes);

  /* creation of mavSendTask */
  mavSendTaskHandle = osThreadNew(MavSendTask, NULL, &mavSendTask_attributes);

  /* creation of buzzerTask */
  buzzerTaskHandle = osThreadNew(BuzzerTask, NULL, &buzzerTask_attributes);

  /* creation of sdLogTask */
  sdLogTaskHandle = osThreadNew(SDLogTask, NULL, &sdLogTask_attributes);

  /* creation of gnssTask */
  gnssTaskHandle = osThreadNew(GnssTask, NULL, &gnssTask_attributes);

  /* creation of uwbTask */
  uwbTaskHandle = osThreadNew(UWBTask, NULL, &uwbTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(IMUTask, NULL, &imuTask_attributes);

  /* creation of magTask */
  magTaskHandle = osThreadNew(MagTask, NULL, &magTask_attributes);

  /* creation of offboardTask */
  offboardTaskHandle = osThreadNew(OffboardTask, NULL, &offboardTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /// Define task handler
  /// param1:     task function that step(3) declared;
  /// param2:     pointer that is passed to the thread function as start argument, usually set to NULL;
  /// param3:     task attributes that step(2) defined;
  /// return:	  task handler that step(1) declared;
  idleTaskHandle = osThreadNew(IdleTask, NULL, &idleTask_attributes);

  /* *************************************************
   * ****************Dev code begin*******************/
  // Warning! Developer can add your new code here!

  /* ****************Dev code end*********************
   * *************************************************/

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_InitTask */
/**
  * @brief  Function implementing the initTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_InitTask */
void InitTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN InitTask */
  reset_usb();
  config_comm();
  set_s1_baudrate(COMM_1_BANDRATE);
  set_s2_baudrate(COMM_2_BANDRATE);
  set_s3_baudrate(COMM_3_BANDRATE);
  set_s4_baudrate(COMM_4_BANDRATE);
  usb_printf("\r\nSystem: Mcontroller-V%ld-%ld initializing ...\r\n",VERSION_HARDWARE, VERSION_FIRMWARE);
  adc_init();
  FRAM_Init();
  update_dataflash();
  RC_Input_Init(RC_INPUT_SBUS);
  wifi_init();
  IMU_Init();
  MAG_Init();
  while(BARO_Init());
  vl53lxx_init();
  motors_init();
  attitude_init();
  pos_init();
  if(!uwb_init()){
  	  osThreadTerminate(uwbTaskHandle);
  }
  Buzzer_set_ring_type(BUZZER_INITIALED);
  usb_printf("System initialized succeed!\r\n");
  initialed_task=true;//当初始化未完成时，只运行buzzer task
  set_motors_value();
  set_servos_value();
  SCB_CleanInvalidateDCache();
  osThreadTerminate(initTaskHandle);//终结线程，并回收内存
  /* USER CODE END InitTask */
}

/* USER CODE BEGIN Header_Loop400hzTask */
/**
* @brief Function implementing the loop400hzTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Loop400hzTask */
void Loop400hzTask(void *argument)
{
  /* USER CODE BEGIN Loop400hzTask */
	while(!initialed_task){
		osDelay(1000);
	}
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);
	  ahrs_update();
	  ekf_baro_alt();
	  /***Do not change code above and change or add new code below***/
	  ekf_odom_xy();
	  ekf_gnss_xy();
	  mode_update();
	  set_motors_value();
	  set_servos_value();
  }
  /* USER CODE END Loop400hzTask */
}

/* USER CODE BEGIN Header_Loop200hzTask */
/**
* @brief Function implementing the loop200hzTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Loop200hzTask */
void Loop200hzTask(void *argument)
{
  /* USER CODE BEGIN Loop200hzTask */
	while(!initialed_task){
		osDelay(1000);
	}
	comm_data_flush();
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);
#if COMM_0==CONFIG_COMM
	  config_callback();
#else
	  comm_callback();
#endif
	  adc_update();
  }
  /* USER CODE END Loop200hzTask */
}

/* USER CODE BEGIN Header_Loop100hzTask */
/**
* @brief Function implementing the loop100hzTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Loop100hzTask */
void Loop100hzTask(void *argument)
{
  /* USER CODE BEGIN Loop100hzTask */
	while(!initialed_task){
		osDelay(1000);
	}
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);
	  lock_motors_check();
	  arm_motors_check();
	  throttle_loop();
  }
  /* USER CODE END Loop100hzTask */
}

/* USER CODE BEGIN Header_Loop50hzTask */
/**
* @brief Function implementing the loop50hzTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Loop50hzTask */
void Loop50hzTask(void *argument)
{
  /* USER CODE BEGIN Loop50hzTask */
	while(!initialed_task){
		osDelay(1000);
	}
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);
	  comm_uwb_callback();
	  uwb_send_data();
	  RC_Input_Loop();
	  uwb_position_update();
	  vl53lxx_update();
  }
  /* USER CODE END Loop50hzTask */
}

/* USER CODE BEGIN Header_HeartbeatTask */
/**
* @brief Function implementing the heartbeatTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HeartbeatTask */
void HeartbeatTask(void *argument)
{
  /* USER CODE BEGIN HeartbeatTask */
	while(!initialed_task){
		osDelay(1000);
	}
	const TickType_t TimeIncrement= pdMS_TO_TICKS(1000); // 1s
	TickType_t PreviousWakeTime=xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  FMU_LED1_Control(true);
#if COMM_0!=CONFIG_COMM
	  send_mavlink_heartbeat_data();
#endif
	  osDelay(100);
	  FMU_LED1_Control(false);
	  sdled_update();
	  vTaskDelayUntil(&PreviousWakeTime, TimeIncrement);
  }
  /* USER CODE END HeartbeatTask */
}

/* USER CODE BEGIN Header_MavSendTask */
/**
* @brief Function implementing the mavSendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MavSendTask */
void MavSendTask(void *argument)
{
  /* USER CODE BEGIN MavSendTask */
	while(!initialed_task){
		osDelay(1000);
	}
  /* Infinite loop */
  for(;;)
  {
	  distribute_mavlink_data();
	  osDelay(100);
  }
  /* USER CODE END MavSendTask */
}

/* USER CODE BEGIN Header_BuzzerTask */
/**
* @brief Function implementing the buzzerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BuzzerTask */
void BuzzerTask(void *argument)
{
  /* USER CODE BEGIN BuzzerTask */
  /* Infinite loop */
  for(;;)
  {
	  Buzzer_ring();
  }
  /* USER CODE END BuzzerTask */
}

/* USER CODE BEGIN Header_SDLogTask */
/**
* @brief Function implementing the sdLogTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SDLogTask */
void SDLogTask(void *argument)
{
  /* USER CODE BEGIN SDLogTask */
	while(!initialed_task){
		osDelay(1000);
	}
  /* Infinite loop */
  for(;;)
  {
	Logger_Update();
  }
  /* USER CODE END SDLogTask */
}

/* USER CODE BEGIN Header_GnssTask */
/**
* @brief Function implementing the gnssTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GnssTask */
void GnssTask(void *argument)
{
  /* USER CODE BEGIN GnssTask */
	while(!initialed_task){
		osDelay(1000);
	}
#if !USE_GNSS
	osThreadTerminate(gnssTaskHandle);
#endif
	if(COMM_1==GPS_COMM){
		set_gnss_comm(gnss_comm1);
	}else if(COMM_2==GPS_COMM){
		set_gnss_comm(gnss_comm2);
	}else if(COMM_3==GPS_COMM){
		set_gnss_comm(gnss_comm3);
	}else if(COMM_4==GPS_COMM){
		set_gnss_comm(gnss_comm4);
	}
	Gnss_Init(UM482); // GPS_Init() task will block 10s
    uint8_t state_flag=0;
  /* Infinite loop */
  for(;;)
  {
	  if(state_flag<10){
		  FMU_LED5_Control(get_gnss_stabilize());
	  }else{
		  FMU_LED5_Control(get_gnss_state());// gps state
	  }
	  gnss_update();
	  state_flag++;
	  if(state_flag>=20){
		  state_flag=0;
	  }
	  osDelay(20);
  }
  /* USER CODE END GnssTask */
}

/* USER CODE BEGIN Header_UWBTask */
/**
* @brief Function implementing the uwbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UWBTask */
void UWBTask(void *argument)
{
  /* USER CODE BEGIN UWBTask */
	while(!initialed_task){
		osDelay(1000);
	}
#if USE_UWB==0
	osThreadTerminate(uwbTaskHandle);
#endif
  /* Infinite loop */
  for(;;)
  {
	  uwb_update();
  }
  /* USER CODE END UWBTask */
}

/* USER CODE BEGIN Header_IMUTask */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMUTask */
void IMUTask(void *argument)
{
  /* USER CODE BEGIN IMUTask */
	while(!initialed_task){
		osDelay(1000);
	}
	const TickType_t TimeIncrement= pdMS_TO_TICKS(2); // 2ms
	TickType_t PreviousWakeTime=xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  vTaskDelayUntil(&PreviousWakeTime, TimeIncrement);
	  IMU_Get_Data();
	  BARO_Get_Date();
  }
  /* USER CODE END IMUTask */
}

/* USER CODE BEGIN Header_MagTask */
/**
* @brief Function implementing the magTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MagTask */
void MagTask(void *argument)
{
  /* USER CODE BEGIN MagTask */
	while(!initialed_task){
		osDelay(1000);
	}
	const TickType_t TimeIncrement= pdMS_TO_TICKS(10); // 10ms
	TickType_t PreviousWakeTime=xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	vTaskDelayUntil(&PreviousWakeTime, TimeIncrement);
	MAG_Get_Data();
  }
  /* USER CODE END MagTask */
}

/* USER CODE BEGIN Header_OffboardTask */
/**
* @brief Function implementing the offboardTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OffboardTask */
void OffboardTask(void *argument)
{
  /* USER CODE BEGIN OffboardTask */
	while(!initialed_task){
		osDelay(1000);
	}
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);//200hz
	  comm_send_callback();
  }
  /* USER CODE END OffboardTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/// Add new thread application below

/* *************************************************
 * ****************Dev code begin*******************/
// Warning! Developer can add your new code here!

/* ****************Dev code end*********************
 * *************************************************/
/// (5) define task function that step(3) declared
void IdleTask(void *argument){
	while(!initialed_task){
		osDelay(1000);
	}
//	TaskStatus_t taskstatus;		// NOTED: add codes that don't need to loop
	for(;;)							// NOTED: if codes need to loop, must add into for(;;){} or while(1){} or some other looper.
	{
	  check_usb_reset();
	  debug();
//	  vTaskGetInfo(loop400hzTaskHandle, &taskstatus, pdTRUE, eInvalid);
//	  usb_printf("freeHeapSize:%d, freeStackSize:%d\n", xPortGetFreeHeapSize(),taskstatus.usStackHighWaterMark);
	  osDelay(10);
	}
}

bool get_task_initialed(void){
	return initialed_task;
}

uint32_t get_time_us(void){
	return time_ms*1000+htim16.Instance->CNT;
}

//*************TIM Callback**************//
void TIM_1000HZ_Callback(void){
	time_ms++;
}

void TIM_400HZ_Callback(void){
	osThreadFlagsSet(loop400hzTaskHandle,1);
}

void TIM_200HZ_Callback(void){
	osThreadFlagsSet(offboardTaskHandle,1);
	osThreadFlagsSet(loop200hzTaskHandle,1);
}

void TIM_100HZ_Callback(void){
	osThreadFlagsSet(loop100hzTaskHandle,1);
}

void TIM_50HZ_Callback(void){
	osThreadFlagsSet(loop50hzTaskHandle,1);
}

//************GPIO Callback**************//
void gpio1_interrupt_callback(void){

}

void gpio2_interrupt_callback(void){

}

void gpio3_interrupt_callback(void){

}

void gpio4_interrupt_callback(void){

}

void gpio5_interrupt_callback(void){

}

void gpio6_interrupt_callback(void){

}

void gpio7_interrupt_callback(void){

}

void gpio8_interrupt_callback(void){

}
/* USER CODE END Application */

