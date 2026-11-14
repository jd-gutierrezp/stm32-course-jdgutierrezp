/*
 * stm32f4xx_it.c
 * Interrupt service routines
 * Adaptado para el proyecto de Encoders, LCD y Motores Paso a Paso
 */

#include "stm32f4xx_hal.h"

/* Declaración de las variables externas definidas en main.c */
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

/* SysTick handler — requerido por la HAL para HAL_Delay() y timeouts */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/******************************************************************************/

/* Manejador de interrupción del Timer 4 (250ms Blinky y Refresco) */
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4); // Redirige al callback de HAL (HAL_TIM_PeriodElapsedCallback en main.c)
}

/* Manejador de interrupción del USART2 (Recepción de comandos seriales) */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2); // Redirige al callback de HAL (HAL_UART_RxCpltCallback en main.c)
}

void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
}

