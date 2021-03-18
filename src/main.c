/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#define TRUE		1
#define FALSE		0

static void prvSetupUart(void);
static void prvSetupGPIO(void);
static void prvSetupHardware(void);
void printmsg(char *msg);
char usr_msg[250] = {0};

void vTask1_menu_display(void *params);
void vTask2_cmd_handling(void *params);
void vTask3_cmd_processing(void *params);
void vTask4_uart_write(void *params);

//Task handles
TaskHandle_t xTaskHandle1 = NULL;
TaskHandle_t xTaskHandle2 = NULL;
TaskHandle_t xTaskHandle3 = NULL;
TaskHandle_t xTaskHandle4 = NULL;

//Queue handle
QueueHandle_t command_queue = NULL;
QueueHandle_t uart_write_queue = NULL;

void rtos_delay(uint32_t delay_in_ms);

uint8_t command_buffer[20];
uint8_t command_len = 0;

uint8_t getCommandCode(uint8_t *buffer);
void getArguments(uint8_t *buffer);

//Menu
char menu[] = {"\
\r\nLED_ON					--->1\
\r\nLED_OFF					--->2\
\r\nLED_TOGGLE				--->3\
\r\nLED_TOGGLE_OFF			--->4\
\r\nLED_READ_STATTUS		--->5\
\r\nLRTC_PRINT_DATETIME		--->6\
\r\nEXIT_APP				--->0\
\r\nType your option here		"};

#define LED_ON_COMMAND		1
#define LED_OFF_COMMAND		2
#define LED_TOGGLE_COMMAND	3
#define LED_TOGGLE_STOP		4
#define LED_READ_STATUS		5
#define RTC_READ_DATE_TIME	6


//command structure
typedef struct APP_CMD{

	uint8_t	COMMAND_NUM;
	uint8_t	COMMAND_ARGS[10];
}APP_CMD_t;


//helper functions
void make_led_on(void);
void make_led_off(void);
void led_toggle_start(uint32_t duration);
void led_toggle_stop(void);
void read_led_status(char *task_msg);
void read_rtc_info(char *task_msg);
void print_error_msg(char *task_msg);

//software timer callback function
void led_toggle(TimerHandle_t xTimer);

//software timer handler
TimerHandle_t led_timer_handle = NULL;

int main(void)
{

	//1. Reset the RCC clock configuration to the default reset state.
	//HSI on, PLL off, HSE off, CPU clock = 16MhZ
	RCC_DeInit();

	//2. Update the SystemCoreClock variable
	SystemCoreClockUpdate();

	prvSetupHardware();

	sprintf(usr_msg, "\r\nDemo of Queue program \r\n");
	printmsg(usr_msg);

	//Create the command queue
	command_queue = xQueueCreate(10, sizeof(APP_CMD_t*));

	//Create the write queue
	uart_write_queue = xQueueCreate(10, sizeof(char*));

	if ( (command_queue != NULL) && (uart_write_queue != NULL)){

		//Create tasks
		xTaskCreate(vTask1_menu_display,
					"Menu display",
					500,
					NULL,
					1,
					&xTaskHandle1);

		xTaskCreate(vTask2_cmd_handling,
					"CMD handling",
					500,
					NULL,
					2,
					&xTaskHandle2);

		xTaskCreate(vTask3_cmd_processing,
					"CMD processing",
					500,
					NULL,
					2,
					&xTaskHandle3);

		xTaskCreate(vTask4_uart_write,
					"UART_write",
					500,
					NULL,
					2,
					&xTaskHandle4);

	}else{

		sprintf(usr_msg, "Queue creation failed\r\n");
		printmsg(usr_msg);
	}

	//Start scheduler
	vTaskStartScheduler();

	for(;;);
}

void vTask1_menu_display(void *params){

	char *pData = menu;

	while(1){

		xQueueSend(uart_write_queue, &pData, portMAX_DELAY);

		//wait until someone notifies
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

	}
}

void vTask2_cmd_handling(void *params){

	uint8_t command_code = 0;
	APP_CMD_t *new_cmd;

	while(1){

		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
		//1. send command to queue
		command_code = getCommandCode(command_buffer);
		new_cmd = (APP_CMD_t*) pvPortMalloc(sizeof(APP_CMD_t));
		new_cmd->COMMAND_NUM = command_code;
		getArguments(new_cmd->COMMAND_ARGS);

		//Send the command to the command queue
		xQueueSend(command_queue, &new_cmd, portMAX_DELAY);

	}
}


void vTask3_cmd_processing(void *params){

	APP_CMD_t *new_cmd;
	uint8_t task_msg[50];

	uint32_t toggle_duration = pdMS_TO_TICKS(500);

	while(1){

		xQueueReceive(command_queue, (void*)&new_cmd, portMAX_DELAY);

		if(new_cmd->COMMAND_NUM == LED_ON_COMMAND){

			make_led_on();

		}else if(new_cmd->COMMAND_NUM == LED_OFF_COMMAND){

			make_led_off();

		}else if(new_cmd->COMMAND_NUM == LED_TOGGLE_COMMAND){

			led_toggle_start(toggle_duration);

		}else if(new_cmd->COMMAND_NUM == LED_TOGGLE_STOP){

			led_toggle_stop();

		}else if(new_cmd->COMMAND_NUM == LED_READ_STATUS){

			read_led_status(task_msg);

		}else if(new_cmd->COMMAND_NUM == RTC_READ_DATE_TIME){

			read_rtc_info(task_msg);

		}else{

			print_error_msg(task_msg);
		}
	}
}

void make_led_on(void){

	GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
}

void make_led_off(void){

	GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
}

void led_toggle(TimerHandle_t xTimer){

	GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
}

void led_toggle_start(uint32_t duration){

	if(led_timer_handle == NULL){

		//create software timer
		led_timer_handle = xTimerCreate("LED-Timer", duration, pdTRUE, NULL, led_toggle);

		//start software timer
		xTimerStart(led_timer_handle, portMAX_DELAY);

	}else{

		//start software timer
		xTimerStart(led_timer_handle, portMAX_DELAY);
	}
}

void led_toggle_stop(void){


	xTimerStop(led_timer_handle, portMAX_DELAY);
}

void read_led_status(char *task_msg){

	sprintf(task_msg, "\r\nLED status is: %d\r\n", GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5));
	xQueueSend(uart_write_queue, &task_msg, portMAX_DELAY);
}

void read_rtc_info(char *task_msg){


}

void print_error_msg(char *task_msg){

	sprintf(task_msg, "\r\nInvalid command\r\n");
	xQueueSend(uart_write_queue, &task_msg, portMAX_DELAY);
}

void vTask4_uart_write(void *params){

	char *pData = NULL;

	while(1){

		xQueueReceive(uart_write_queue, &pData, portMAX_DELAY);
		printmsg(pData);
	}
}

void rtos_delay(uint32_t delay_in_ms){

	uint32_t current_tick_count = xTaskGetTickCount();
	uint32_t delay_in_ticks = (delay_in_ms * configTICK_RATE_HZ) / 1000;

	while(xTaskGetTickCount() < current_tick_count + delay_in_ticks);

}

void USART2_IRQHandler(void){

	uint16_t data_byte;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if( USART_GetFlagStatus(USART2, USART_FLAG_RXNE)){

		//a data byte is received from the user
		data_byte = USART_ReceiveData(USART2);

		command_buffer[command_len++] = data_byte & 0xFF;

		if(data_byte == '\r'){

			// reset the command len
			command_len = 0;

			//notify the command handling task
			xTaskNotifyFromISR(xTaskHandle2, 0, eNoAction, &xHigherPriorityTaskWoken);

			xTaskNotifyFromISR(xTaskHandle1, 0, eNoAction, &xHigherPriorityTaskWoken);
		}
	}

	if(xHigherPriorityTaskWoken){

		taskYIELD();
	}
}

static void prvSetupHardware(void){

	//Setup Button and LED
	prvSetupGPIO();

	//Setup UART
	prvSetupUart();

}

uint8_t getCommandCode(uint8_t *buffer){

	return buffer[0] - 48;
}

void getArguments(uint8_t *buffer){

}

static void prvSetupGPIO(void){

	GPIO_InitTypeDef led_init, button_init;

	//Enable the GPIOA, GPIOC and SYSCFG peripheral clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);


	led_init.GPIO_Mode = GPIO_Mode_OUT;
	led_init.GPIO_OType = GPIO_OType_PP;
	led_init.GPIO_Pin = GPIO_Pin_5; //LED onboard
	led_init.GPIO_Speed = GPIO_Low_Speed;
	led_init.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOA, &led_init);

	button_init.GPIO_Mode = GPIO_Mode_IN;
	button_init.GPIO_OType = GPIO_OType_PP;
	button_init.GPIO_Pin = GPIO_Pin_13; //Button onboard
	button_init.GPIO_Speed = GPIO_Low_Speed;
	button_init.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOC, &button_init);
}

static void prvSetupUart(void){

	GPIO_InitTypeDef gpio_uart_pins;
	USART_InitTypeDef uart2_init;

	//Enable the UART2 and GPIOA peripheral clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//PA2 is Tx. PA3 is Rx
	//Alternate function configuration of MCU to behave as UART2

	memset(&gpio_uart_pins, 0, sizeof(gpio_uart_pins));

	gpio_uart_pins.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	gpio_uart_pins.GPIO_Mode = GPIO_Mode_AF;
	gpio_uart_pins.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &gpio_uart_pins);

	//AF mode setting
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //PA2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); //PA3

	//UART parameter initializations

	memset(&uart2_init, 0, sizeof(uart2_init));

	uart2_init.USART_BaudRate = 115200;
	uart2_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart2_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	uart2_init.USART_Parity = USART_Parity_No;
	uart2_init.USART_StopBits = USART_StopBits_1;
	uart2_init.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &uart2_init);

	//Enable the UART interrupts
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	NVIC_SetPriority(USART2_IRQn, 5);
	NVIC_EnableIRQ(USART2_IRQn);

	//Enable the UART2 peripheral
	USART_Cmd(USART2, ENABLE);

}

void printmsg(char *msg){

	for(uint32_t i = 0; i < strlen(msg); i++){

		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) != SET);
		USART_SendData(USART2, msg[i]);
	}
}
