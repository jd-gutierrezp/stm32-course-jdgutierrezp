/*
 * main_b.c
 *
 *  Created on: Jul 24, 2026
 *      Author: juand
 */


#include "stm32f4xx_hal.h"

#include <stdio.h>

#include <stdlib.h>

#include <string.h>



I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;



uint8_t msg_buffer[64] = {0};



uint16_t dirMatch = 0x08;

uint8_t flag_refresh_250ms =0;




void SystemClock_Config(void);

void MX_GPIO_Init(void);

void MX_I2C1_Init(void);

void MX_USART2_UART_Init(void);

void Error_Handler(void);

void deviceReady(void);

void tim4_blinky_Init(void);





int main(void) {

	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();

	MX_I2C1_Init();

	MX_USART2_UART_Init();

	deviceReady();

	tim4_blinky_Init();

	while (1) {

		// Aquí puedes agregar la lógica para enviar comandos y datos

		// a tu pantalla usando las funciones HAL_I2C_Master_Transmit



		HAL_Delay(1000);

	}

}


void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();   // <-- faltaba

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);   // <-- faltaba
}


void MX_I2C1_Init(void) {

	// 1. Habilitar el reloj del periférico I2C1

	__HAL_RCC_I2C1_CLK_ENABLE();



	GPIO_InitTypeDef GPIO_InitStruct = {0};



	// 2. Configurar los pines GPIO para I2C1 (PB6 -> SCL, PB7 -> SDA)

	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;

	// Modo Alternativo Open-Drain (Drenador Abierto): requerido para el bus I2C

	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;

	// Activar pull-up interno para asegurar niveles lógicos estables

	GPIO_InitStruct.Pull = GPIO_PULLUP;

	// Velocidad de respuesta muy alta para flancos de subida y bajada limpios

	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	// Configurar función alternativa AF4 para habilitar el periférico I2C1 en estos pines

	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);



	// 3. Configuración de los parámetros del periférico I2C1

	hi2c1.Instance = I2C1;

	// Velocidad de transmisión estándar a 100 kHz (Standard Mode)

	hi2c1.Init.ClockSpeed = 100000;

	// Relación de ciclo de trabajo al 50%

	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;

	// Dirección propia del microcontrolador (no aplica actuando como maestro, se deja en 0)

	hi2c1.Init.OwnAddress1 = 0;

	// Direcciones del bus de 7 bits

	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;

	// Desactivar direccionamiento dual

	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;

	hi2c1.Init.OwnAddress2 = 0;

	// Desactivar el broadcast (llamada general)

	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;

	// Habilitar Clock Stretching (estiramiento del reloj)

	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;



	// Inicializar el periférico con la estructura configurada

	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {

		Error_Handler();

	}

}



void MX_USART2_UART_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct = {0};



	__HAL_RCC_GPIOA_CLK_ENABLE();

	__HAL_RCC_USART2_CLK_ENABLE();



	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;

	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

	GPIO_InitStruct.Pull = GPIO_NOPULL;

	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



	huart2.Instance = USART2;

	huart2.Init.BaudRate = 19200;

	huart2.Init.Mode = UART_MODE_TX_RX;

	huart2.Init.Parity = UART_PARITY_NONE;

	huart2.Init.StopBits = UART_STOPBITS_1;

	huart2.Init.WordLength = UART_WORDLENGTH_8B;

	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;

	huart2.Init.OverSampling = UART_OVERSAMPLING_16;



	if (HAL_UART_Init(&huart2) != HAL_OK) {

		Error_Handler();

	}

}



void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct = {0};

	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};



	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;

	RCC_OscInitStruct.HSIState = RCC_HSI_ON;

	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

	HAL_RCC_OscConfig(&RCC_OscInitStruct);



	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK |

			RCC_CLOCKTYPE_HCLK |

			RCC_CLOCKTYPE_PCLK1 |

			RCC_CLOCKTYPE_PCLK2;

	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;

	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;



	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

}



//void datos(uint8_t dato){

// uint8_t bajo = (dato << 4);

// uint8_t alto = (dato >> 4);

//

// //mandamos mitad alta con 1

// alto = (alto << 4) |= (0b1 << 2); //Bit alto con E

//

// HAL_I2C_Master_Transmit();

//

//

//

//

//

//}

/* ID de los dispositivos I2C */

void deviceReady(void){

	sprintf((char*)msg_buffer, "Scanner iniciado \r\n"); //Creamos el mensaje a enviar

	HAL_UART_Transmit(&huart2, msg_buffer, strlen((char *)msg_buffer), 100); //Transmitimos el mensaje

	while(dirMatch <= 0x7F){

		if(HAL_I2C_IsDeviceReady(&hi2c1, dirMatch << 1, 1, 100) == HAL_OK){

			sprintf((char*)msg_buffer, "The OLED is at %02X \r\n", dirMatch); //Creamos el mensaje a enviar

			HAL_UART_Transmit(&huart2, msg_buffer, strlen((char *)msg_buffer), 100); //Transmitimos el mensaje

		}

		dirMatch++;

	}

}

void tim4_blinky_Init(void) {
	__HAL_RCC_TIM4_CLK_ENABLE();				        // Otorga reloj al periférico TIM4

	htim4.Instance = TIM4;						        // Asigna el periférico correspondiente
	htim4.Init.Prescaler = 15999;     			        // Ajusta el prescaler para obtener pulsos de 1 milisegundo
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;        // Establece conteo incremental
	htim4.Init.Period = 249;       				        // Pide que resbale o se desborde al llegar a 250 milisegundos
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  // Anula divisiones de reloj auxiliares
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // Permite amortiguación de parámetros en caliente
	HAL_TIM_Base_Init(&htim4);                          // Finaliza la configuración en hardware

	HAL_NVIC_EnableIRQ(TIM4_IRQn);                      //matricula la interrupcion en el NVIC

	HAL_TIM_Base_Start_IT(&htim4);                      // Ordena al temporizador iniciar y emitir alertas cada fin de ciclo
	__NOP();
}
/* Se ejecuta automáticamente cada que un Timer finaliza su cuenta máxima */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) {                       // Verifica que la llamada proviene exclusivamente del TIM4 (250ms)
		HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);          // Cambia el estado actual del LED de placa (Blinky)
		flag_refresh_250ms = 1;                         // Informa a la Máquina Principal que es hora de actualizar la informaion en el serial
	}
}

void Error_Handler(void) {

	while (1) {

	}

}

