//******************************************************************************
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "discoveryf4utils.h"
#include "stdbool.h"
#include "stdlib.h"
//******************************************************************************

//******************************************************************************
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"
#include "main.h"
//******************************************************************************


// sound init*******************************************************************

TIM_TimeBaseInitTypeDef timer_InitStructure;

// use for setup the clock speed for TIM2
// this timer is used for general purpose timing
const uint16_t TIMER_2_PRESCALER = 232;
const uint16_t TIMER_2_PERIOD = 2999;
const uint16_t TIMER_2_FREQUENCY = 120;

unsigned int timer_for_button_hold = 0;
unsigned int timer_for_button_released = 0;

bool is_button_up = true;
bool start = false;

bool within_double_click_period = false;
bool is_single_click = false;
bool is_double_click = false;

typedef enum MODE {FPS, EDF, LLS, custom} mode;
mode curMode = FPS;

const float MIN_PRESS_TIME = 0.05; // the min single press should need 0.05 second.
const float DOUBLE_CLICK_TIME = 0.5; // double press should be with in 0.5 second.
const float SOUND_OUTPUT = 0.3; // output the sound for 1 second
fir_8 filt;
bool output_sound = false;
int timer_for_sound = 0;
bool sound_init = false;

void InitSound() {
	SystemInit();

	//enables GPIO clock for PortD
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	codec_init();
	codec_ctrl_init();

	I2S_Cmd(CODEC_I2S, ENABLE);

	initFilter(&filt);
}


void initSound() {
	SystemInit();

	//enables GPIO clock for PortD
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	codec_init();
	codec_ctrl_init();

	I2S_Cmd(CODEC_I2S, ENABLE);

	initFilter(&filt);
}

// the following code is for sound
// a very crude FIR lowpass filter
float updateFilter(fir_8* filt, float val) {
	uint16_t valIndex;
	uint16_t paramIndex;
	float outval = 0.0;

	valIndex = filt->currIndex;
	filt->tabs[valIndex] = val;

	for (paramIndex=0; paramIndex<8; paramIndex++)
	{
		outval += (filt->params[paramIndex]) * (filt->tabs[(valIndex+paramIndex)&0x07]);
	}

	valIndex++;
	valIndex &= 0x07;

	filt->currIndex = valIndex;

	return outval;
}

void initFilter(fir_8* theFilter) {
	uint8_t i;

	theFilter->currIndex = 0;

	for (i=0; i<8; i++)
		theFilter->tabs[i] = 0.0;

	theFilter->params[0] = 3;
	theFilter->params[1] = 3;
	theFilter->params[2] = 3;
	theFilter->params[3] = 3;
	theFilter->params[4] = 3;
	theFilter->params[5] = 3;
	theFilter->params[6] = 3;
	theFilter->params[7] = 3;
}

void InitTimer_2() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  timer_InitStructure.TIM_Prescaler = TIMER_2_PRESCALER;
  timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timer_InitStructure.TIM_Period = TIMER_2_PERIOD;
  timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timer_InitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, & timer_InitStructure);
  TIM_Cmd(TIM2, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void EnableTimer2Interrupt() {
  NVIC_InitTypeDef nvicStructure;
  nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( & nvicStructure);
}

void TIM2_IRQHandler() {
	// tick = idle time
  //Checks whether the TIM2 interrupt has occurred or not
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    		
		timer_for_button_hold++;
    timer_for_button_released++;
		
		if(output_sound)
		{
			timer_for_sound++;
		}
		if(timer_for_sound > SOUND_OUTPUT * TIMER_2_FREQUENCY)
		{
			timer_for_sound = 0;
			output_sound = false;
		}
		
  }
}

// end sound init***************************************************************

typedef struct task {
	int start_time;
	int priority; // used for fixed priority scheduler maybe?
	int execution_tim_left; // used for laxity scheduler 
	int deadline_tim; // used for earliest deadline first scheduler
	TaskHandle_t handler;
} Task;

int current_time = 0;

void vSoundTask(void *pvParameters);
void vButtonInput(void *pvParameters);
void vSeafood(void *pvParameters);
void vVeggie(void *pvParameters);
void vHawaiianChicken(void *pvParameters);
void vPeperoni(void *pvParameters);
void vScheduler(void *pvParameters); // fixed priority scheduler
//void vEDF(void *pvParameters); // earliest deaslin first scheduler
//void vLLF(void *pvParameters); //least laxity first scheduler

void add_tasks(void);
void update_tasks(void);
void update_task_time(int);
void remove_tasks(void);
void next_task(void);
void next_hightest_priority_task(void);
void next_earliest_deadline_task(void);
void next_least_laxity_task(void);
int get_exe_tim(int);
bool task_hit_deadline(int);
int getTaskPriority(int);
int getTaskDeadline(int);
int getTaskLaxity(int);
void suspendTask(int);
void resumeTask(int);
void suspendAll(void);
Task* taskSetup(TaskHandle_t, int, int, int, int);
//void Delay(uint32_t val);

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/

TaskHandle_t seafood_handler = NULL;
TaskHandle_t veggie_handler = NULL;
TaskHandle_t hawaiian_handler = NULL;
TaskHandle_t peperoni_handler = NULL;

// each task priority
int seafood_priority = 3;
int veggie_priority = 1;
int hawaiian_priority = 2;
int pepperoni_priority = 2;

int scheduler_priority = 4;

// each task deadline
int seafood_deadline = 5;
int veggie_deadline = 10;
int hawaiian_deadline = 10;
int pepperoni_deadline = 15;

// each task duration
int seafood_duration = 3;
int veggie_duration = 4;
int hawaiian_duration = 6;
int pepperoni_duration = 8;

// each task period
int seafood_period = 20;
int veggie_period = 30;
int hawaiian_period = 40;
int pepperoni_period = 40;

#define seafoodLED LED_BLUE;
#define veggieLED LED_GREEN;
#define hawaiianLED LED_ORANGE;
#define pepperoniLED LED_RED;

portBASE_TYPE xStatus;
 
QueueHandle_t xQueue1;
// these are the vaule stored in the queue, represend tasks
int seafood_t = 0;
int veggie_t = 1;
int hawaiian_t = 2;
int peperoni_t = 3;

int cur_t;

Task* seafood_task;
Task* veggie_task;
Task* hawaiian_task;
Task* peperoni_task;
 
//******************************************************************************
int main(void)
{
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	STM_EVAL_LEDInit(LED_BLUE);
	STM_EVAL_LEDInit(LED_GREEN);
	STM_EVAL_LEDInit(LED_ORANGE);
	STM_EVAL_LEDInit(LED_RED);
	
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);
	
	InitTimer_2();
	EnableTimer2Interrupt();
	// init sound******************************************
	initSound();
	initFilter(&filt);
	output_sound = false;
	timer_for_sound = 0;
	 
	xTaskCreate( vSoundTask, (const char*)"Sound Task",
	STACK_SIZE_MIN, NULL, scheduler_priority, NULL);
	// init sound end**************************************
	
	xQueue1 = xQueueCreate( 10, sizeof( int ) );
	
	if(xQueue1 != NULL)
	{
		xTaskCreate( vSeafood, (const char*)"seafood pizza", 
			STACK_SIZE_MIN, NULL, seafood_priority, &seafood_handler );
		xTaskCreate( vVeggie, (const char*)"veggie pizza", 
			STACK_SIZE_MIN, NULL, veggie_priority, &veggie_handler );
		xTaskCreate( vHawaiianChicken, (const char*)"hawaiian chicken pizza", 
			STACK_SIZE_MIN, NULL, hawaiian_priority, &hawaiian_handler );
		xTaskCreate( vPeperoni, (const char*)"pepperoni pizza", 
			STACK_SIZE_MIN, NULL, pepperoni_priority, &peperoni_handler );

		seafood_task  = taskSetup(seafood_handler,  seafood_deadline,   seafood_duration,   seafood_priority,   current_time); 
		veggie_task   = taskSetup(veggie_handler,   veggie_deadline,    veggie_duration,    veggie_priority,    current_time);
		hawaiian_task = taskSetup(hawaiian_handler, hawaiian_deadline,  hawaiian_duration,  hawaiian_priority,  current_time);
		peperoni_task = taskSetup(peperoni_handler, pepperoni_deadline, pepperoni_duration, pepperoni_priority, current_time);
		
		// put the tasks in to the Queue
		xQueueSendToBack(xQueue1, (void *) &seafood_t, ( TickType_t ) 10);
		xQueueSendToBack(xQueue1, (void *) &veggie_t, ( TickType_t ) 10);
		xQueueSendToBack(xQueue1, (void *) &hawaiian_t, ( TickType_t ) 10);
		xQueueSendToBack(xQueue1, (void *) &peperoni_t, ( TickType_t ) 10);

		xTaskCreate( vScheduler, (const char*)"fixed priority task",
			STACK_SIZE_MIN, NULL, scheduler_priority, NULL);
		xTaskCreate( vButtonInput, (const char*)"button input",
			STACK_SIZE_MIN, NULL, scheduler_priority, NULL);
		
		
		vTaskStartScheduler();
		
	}
}

void vSeafood(void *pvParameters)
{
	for(;;)
	{
		Led_TypeDef led = seafoodLED;
		STM_EVAL_LEDToggle(led);
		vTaskDelay( 400 / portTICK_RATE_MS );
	}
}

void vVeggie(void *pvParameters)
{
	for(;;)
	{
		Led_TypeDef led = veggieLED;
		STM_EVAL_LEDToggle(led);
		vTaskDelay( 400 / portTICK_RATE_MS );
	}
}

void vHawaiianChicken(void *pvParameters)
{
	for(;;)
	{
		Led_TypeDef led = hawaiianLED;
		STM_EVAL_LEDToggle(led);
		vTaskDelay( 400 / portTICK_RATE_MS );
	}
}

void vPeperoni(void *pvParameters)
{
	for(;;)
	{
		Led_TypeDef led = pepperoniLED;
		STM_EVAL_LEDToggle(led);
		vTaskDelay( 400 / portTICK_RATE_MS );
	}
}

void vScheduler(void *pvParameters)
{
	for(;;)
	{
		suspendAll();	
		if(start)
		{
			current_time++;
			update_tasks();
			
			if(curMode == FPS)
				next_hightest_priority_task();
			else if(curMode == EDF)
				next_earliest_deadline_task();
			else if(curMode == LLS)
				next_least_laxity_task();
			else if(curMode == custom)
				next_task();
			
			add_tasks();
		}
		vTaskDelay( 800 / portTICK_RATE_MS );
	}
}

void update_tasks()
{
	// update that task execution time
	update_task_time(cur_t);
	
	// remove any tasks are finished, or hit deadline
	remove_tasks();
}

/*
	update the execution time of the given task
*/
void update_task_time(int task)
{
	if(task == seafood_t)
		seafood_task->execution_tim_left = seafood_task->execution_tim_left - 1;
	else if(task == veggie_t)
		veggie_task->execution_tim_left--;
	else if(task == hawaiian_t)
		hawaiian_task->execution_tim_left--;
	else if(task == peperoni_t)
		peperoni_task->execution_tim_left--;
}

/*
	loop through all tasks, if a task execution time is 0, kill it
	if a task is hit deadline, kill it
*/
void remove_tasks()
{
	int receiveTask = -1;
	const portTickType xTicksToWait = 100/portTICK_RATE_MS;
	int i = 0;

	// loop 10 times to find hightest priority
	while(i < 10 && xQueueReceive(xQueue1, &receiveTask, xTicksToWait) == pdPASS)
	{
		if(get_exe_tim(receiveTask) > 0 && !task_hit_deadline(receiveTask))
		{
			while(xQueueSendToBack(xQueue1, &receiveTask, 0) != pdPASS)
			{
			}
		} else
		{
			output_sound = true;
		}
		i++;
	}
}

int get_exe_tim(int task)
{	
	if(task == seafood_t)
		return seafood_task->execution_tim_left;
	else if(task == veggie_t)
		return veggie_task->execution_tim_left;
	else if(task == hawaiian_t)
		return hawaiian_task->execution_tim_left;
	else if(task == peperoni_t)
		return peperoni_task->execution_tim_left;
}

bool task_hit_deadline(int task)
{
	if(task == seafood_t)
		return seafood_task->deadline_tim <= current_time;
	else if(task == veggie_t)
		return veggie_task->deadline_tim <= current_time;
	else if(task == hawaiian_t)
		return hawaiian_task->deadline_tim <= current_time;
	else if(task == peperoni_t)
		return peperoni_task->deadline_tim <= current_time;
}

void next_task()
{
	int receiveTask = -1;
	const portTickType xTicksToWait = 100/portTICK_RATE_MS;
	
	xStatus = xQueueReceive(xQueue1, &receiveTask, xTicksToWait);
	if(xStatus == pdPASS)
	{
		xStatus = xQueueSendToBack(xQueue1, &receiveTask, 0);
		if(xStatus == pdPASS)
		{
			xQueuePeek(xQueue1, &receiveTask, xTicksToWait);
			resumeTask(receiveTask);
		}
	}
}

/*
	check all of the task's priority
	resume that task
*/
void next_hightest_priority_task()
{
	int receiveTask = -1;
	const portTickType xTicksToWait = 100/portTICK_RATE_MS;
	int priority = 0;
	int i = 0;
	
	// loop 10 times to find hightest priority
	while(i < 10 && xQueueReceive(xQueue1, &receiveTask, xTicksToWait) == pdPASS)
	{
		if(getTaskPriority(receiveTask) > priority)
			priority = getTaskPriority(receiveTask);
		xQueueSendToBack(xQueue1, &receiveTask, 0);
		i++;
	}
	
	// find the task with that priority
	i = 0;
	while(i < 10 && xQueuePeek(xQueue1, &receiveTask, xTicksToWait) == pdPASS)
	{
		if(getTaskPriority(receiveTask) == priority)
		{
			cur_t = receiveTask;
			resumeTask(receiveTask);
			break;
		} else 
		{
			while(xQueueReceive(xQueue1, &receiveTask, xTicksToWait) != pdPASS)
			{
			}
			while(xQueueSendToBack(xQueue1, &receiveTask, 0) != pdPASS)
			{
			}
		}
	}
}


/*
	check all of the task's deadline
	resume that task
*/
void next_earliest_deadline_task()
{
	int receiveTask = -1;
	const portTickType xTicksToWait = 100/portTICK_RATE_MS;
	int deadline = 2147483647;
	int i = 0;
	
	// loop 10 times to find hightest priority
	while(i < 10 && xQueueReceive(xQueue1, &receiveTask, xTicksToWait) == pdPASS)
	{
		if(getTaskDeadline(receiveTask) < deadline)
			deadline = getTaskDeadline(receiveTask);
		xQueueSendToBack(xQueue1, &receiveTask, 0);
		i++;
	}
	
	// find the task with that priority
	i = 0;
	while(i < 10 && xQueuePeek(xQueue1, &receiveTask, xTicksToWait) == pdPASS)
	{
		if(getTaskDeadline(receiveTask) == deadline)
		{
			cur_t = receiveTask;
			resumeTask(receiveTask);
			break;
		} else 
		{
			while(xQueueReceive(xQueue1, &receiveTask, xTicksToWait) != pdPASS)
			{
			}
			while(xQueueSendToBack(xQueue1, &receiveTask, 0) != pdPASS)
			{
			}
		}
	}
}

void next_least_laxity_task()
{
	int receiveTask = -1;
	const portTickType xTicksToWait = 100/portTICK_RATE_MS;
	int laxity = 2147483647;
	int i = 0;
	
	// loop 10 times to find hightest priority
	while(i < 10 && xQueueReceive(xQueue1, &receiveTask, xTicksToWait) == pdPASS)
	{
		if(getTaskLaxity(receiveTask) < laxity)
			laxity = getTaskLaxity(receiveTask);
		xQueueSendToBack(xQueue1, &receiveTask, 0);
		i++;
	}
	
	// find the task with that priority
	i = 0;
	while(i < 10 && xQueuePeek(xQueue1, &receiveTask, xTicksToWait) == pdPASS)
	{
		if(getTaskLaxity(receiveTask) == laxity)
		{
			cur_t = receiveTask;
			resumeTask(receiveTask);
			break;
		} else 
		{
			while(xQueueReceive(xQueue1, &receiveTask, xTicksToWait) != pdPASS)
			{
			}
			while(xQueueSendToBack(xQueue1, &receiveTask, 0) != pdPASS)
			{
			}
		}
	}
}

void resumeTask(int task)
{
	if(task == seafood_t)
		vTaskResume(seafood_task->handler);
	else if(task == veggie_t)
		vTaskResume(veggie_task->handler);
	else if(task == hawaiian_t)
		vTaskResume(hawaiian_task->handler);
	else if(task == peperoni_t)
		vTaskResume(peperoni_task->handler);
}

void suspendTask(int task)
{
	if(task == seafood_t)
		vTaskSuspend(seafood_task->handler);
	else if(task == veggie_t)
		vTaskSuspend(veggie_task->handler);
	else if(task == hawaiian_t)
		vTaskSuspend(hawaiian_task->handler);
	else if(task == peperoni_t)
		vTaskSuspend(peperoni_task->handler);
}

void suspendAll()
{
	vTaskSuspend(seafood_task->handler);
	vTaskSuspend(veggie_task->handler);
	vTaskSuspend(peperoni_task->handler);
	vTaskSuspend(hawaiian_task->handler);
}

Task* taskSetup(TaskHandle_t handler, int deadline_time, int execution_time_left, int priority, int start_tim)
{
	Task* task = malloc(sizeof(Task));
	task->start_time = start_tim;
	task->handler = handler;
	task->deadline_tim = deadline_time;
	task->execution_tim_left = execution_time_left;
	task->priority = priority;
	
	return task;
}

int getTaskPriority(int task)
{
	if(task == seafood_t)
		return seafood_priority;
	else if(task == veggie_t)
		return veggie_priority;
	else if(task == hawaiian_t)
		return hawaiian_priority;
	else if(task == peperoni_t)
		return pepperoni_priority;
}

int getTaskDeadline(int task)
{
	if(task == seafood_t)
		return seafood_task->deadline_tim;
	else if(task == veggie_t)
		return veggie_task->deadline_tim;
	else if(task == hawaiian_t)
		return hawaiian_task->deadline_tim;
	else if(task == peperoni_t)
		return peperoni_task->deadline_tim;
}

int getTaskLaxity(int task)
{
	if(task == seafood_t)
		return (seafood_task->deadline_tim - current_time) - seafood_task->execution_tim_left;
	else if(task == veggie_t)
		return (veggie_task->deadline_tim - current_time) - veggie_task->execution_tim_left;
	else if(task == hawaiian_t)
		return (hawaiian_task->deadline_tim - current_time) - hawaiian_task->execution_tim_left; 
	else if(task == peperoni_t)
		return (peperoni_task->deadline_tim - current_time) - peperoni_task->execution_tim_left;
}

void add_tasks()
{
	if(current_time % seafood_period == 0)
	{	
		seafood_task->deadline_tim = seafood_deadline + current_time;
		seafood_task->execution_tim_left = seafood_duration;
		seafood_task->start_time = current_time;
		xQueueSendToBack(xQueue1, (void *) &seafood_t, ( TickType_t ) 10);
	}
	if(current_time % veggie_period == 0)
	{
		veggie_task->deadline_tim = veggie_deadline + current_time;
		veggie_task->execution_tim_left = veggie_duration;
		veggie_task->start_time = current_time;
		xQueueSendToBack(xQueue1, (void *) &veggie_t, ( TickType_t ) 10);
	}
	if(current_time % hawaiian_period == 0)
	{
		hawaiian_task->deadline_tim = hawaiian_deadline + current_time;
		hawaiian_task->execution_tim_left = hawaiian_duration;
		hawaiian_task->start_time = current_time;
		xQueueSendToBack(xQueue1, (void *) &hawaiian_t, ( TickType_t ) 10);
	}
	if(current_time % pepperoni_period == 0 )
	{		
		peperoni_task->deadline_tim = pepperoni_deadline + current_time;
		peperoni_task->execution_tim_left = pepperoni_duration;
		peperoni_task->start_time = current_time;
		xQueueSendToBack(xQueue1, (void *) &peperoni_t, ( TickType_t ) 10);
	}
}
//******************************************************************************

void vSoundTask(void *pvParameters) {
	while(1) {
		if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE)) {
			SPI_I2S_SendData(CODEC_I2S, sample);
			if (sampleCounter & 0x00000001) {
				sawWave += NOTEFREQUENCY;
				if (sawWave > 1.0)
					sawWave -= 2.0;

				filteredSaw = updateFilter(&filt, sawWave);
				sample = (int16_t)(NOTEAMPLITUDE*filteredSaw);
			}
			sampleCounter++;
		}
		if (output_sound) {
			if(!sound_init) {
				sound_init = true;
				initSound();
			}
		} else {
			GPIO_ResetBits(GPIOD, GPIO_Pin_4);
			sound_init = false;
			vTaskDelay(10/portTICK_RATE_MS);
		}
	}
}

bool CanUpdateClickState() {
	return (!is_button_up && 
					timer_for_button_hold >= MIN_PRESS_TIME * TIMER_2_FREQUENCY);
}

void update_mode()
{
	within_double_click_period = (timer_for_button_released <= (DOUBLE_CLICK_TIME * TIMER_2_FREQUENCY));
	if(start)
	{
		if(is_single_click && !within_double_click_period)
			is_single_click = false;
		else if(is_double_click)
		{
			start = false;
			is_double_click = false;
		}
	} else
	{
		if(is_single_click && !within_double_click_period)
		{
			if(curMode == FPS)
				curMode = EDF;
			else if(curMode == EDF)
				curMode = LLS;
			else if (curMode == LLS)
				curMode = custom;
			else if(curMode == custom)
				curMode = FPS;
			
			is_single_click = false;
		} else if(is_double_click)
		{
			start = true;
			STM_EVAL_LEDOff(LED_BLUE);
			STM_EVAL_LEDOff(LED_RED);
			STM_EVAL_LEDOff(LED_GREEN);
			STM_EVAL_LEDOff(LED_ORANGE);
			is_double_click = false;
		}
	}
}

void update_LED()
{
	if(!start)
	{
		if(curMode == FPS)
		{
			STM_EVAL_LEDOff(LED_BLUE);
			STM_EVAL_LEDOff(LED_RED);
			STM_EVAL_LEDOff(LED_GREEN);
			STM_EVAL_LEDOn(LED_ORANGE);
		} else if(curMode == EDF)
		{			
			STM_EVAL_LEDOff(LED_BLUE);
			STM_EVAL_LEDOn(LED_RED);
			STM_EVAL_LEDOff(LED_GREEN);
			STM_EVAL_LEDOff(LED_ORANGE);
		} else if(curMode == LLS)
		{
			STM_EVAL_LEDOn(LED_BLUE);
			STM_EVAL_LEDOff(LED_RED);
			STM_EVAL_LEDOff(LED_GREEN);
			STM_EVAL_LEDOff(LED_ORANGE);			
		} else if(curMode == custom)
		{
			STM_EVAL_LEDOff(LED_BLUE);
			STM_EVAL_LEDOff(LED_RED);
			STM_EVAL_LEDOn(LED_GREEN);
			STM_EVAL_LEDOff(LED_ORANGE);
		}
	}
}

void vButtonInput( void * pvParameter)
{
	while(1)
	{
		if(STM_EVAL_PBGetState(BUTTON_USER))
		{
			if(is_button_up)
				timer_for_button_hold = 0;
			is_button_up = false;
		} else 
		{
			if(CanUpdateClickState())
			{
				if (!is_single_click && !is_double_click) {
					is_single_click = true;
				} else if (is_single_click && !is_double_click) {
					is_single_click = false;
					is_double_click = true;
				}
			}
			if(!is_button_up)
				timer_for_button_released = 0;
			is_button_up = true;
		}
		update_mode();
		update_LED();
		vTaskDelay( 10 / portTICK_RATE_MS );
	}
}
