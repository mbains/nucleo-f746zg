#include <FreeRTOS.h>
#include <task.h>
#include "stm32f7xx_hal.h"

static UART_HandleTypeDef * m_uart_p = NULL;

static void vTask1(void *pvParameters) 
{
    const char * pcTaskName = "Task 1 is running\r\n";
    while(1)
    {
        HAL_UART_Transmit(m_uart_p, pcTaskName, strlen(pcTaskName), 100);
        osDelay(1000);
    }
}

static void vTask2(void *pvParameters)
{
    const char * pcTaskName = "Task 2 is running\r\n";
    while(1) 
    {
        osDelay(1000);
        HAL_UART_Transmit(m_uart_p, pcTaskName, strlen(pcTaskName), 100);
    }
}


int vtask_main(UART_HandleTypeDef * uart) 
{
    m_uart_p = uart;
    xTaskCreate(vTask1, "Task 1", 240/*stack*/, NULL, 1, NULL);
    xTaskCreate(vTask2, "Task 2", 240/*stack*/, NULL, 1, NULL);
    return 0;
}
