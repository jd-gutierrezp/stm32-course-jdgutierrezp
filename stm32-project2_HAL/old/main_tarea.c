/*
 * main_tarea.c
 *
 *  Created on: Jul 21, 2026
 *      Author: juand
 */


/* DISTRIBUCION DEL CODIGO*/

/*
 * #INCLUDES
 * DEFINICION DE VARIABLES
 * CABECERA DE FUNCIONES
 * FUNCION MAIN
 * FUNCIONES
 * WHILE
 * FUNCIONES
 * CALLBACKS
 */


/*INCLUDES*/

#include "stm32f4xx_hal.h"  // Incluye la librería principal de abstracción de hardware (HAL)
#include "stdio.h"          // Incluye funciones estándar de entrada/salida (como sprintf)
#include "string.h"         // Incluye funciones para manejo de cadenas de texto (como strlen)
#include <stdlib.h>

/*DEFINICION DE VARIABLES (PERIFÉRICOS)*/

#define LCD_ADDR (0x22 << 1) // Cambia 0x27 por 0x3F si el escáner detecta esa dirección

TIM_HandleTypeDef htim4 = { 0 }; 	// Administra el Timer 4 (Genera la interrupción del Blinky cada 250ms)
UART_HandleTypeDef huart2 = { 0 }; 	// Administra la comunicación serial USART2
I2C_HandleTypeDef hi2c1 = {0};
RTC_HandleTypeDef hrtc1= {0};
ADC_HandleTypeDef hadc1 = {0};
DMA_HandleTypeDef hdma1 = {0};


RTC_TimeTypeDef time ={0};
RTC_DateTypeDef date ={0};

/* VARIABLES DE USUARIO*/

/*pantalla*/
char lcd_buffer[34];            // Buffer para formatear el texto (16 caracteres + fin de línea)
uint32_t last_lcd_update = 0;   // Controla el refresco sin usar delays

/*joystick*/
volatile uint16_t joy_x = 0;
volatile uint16_t joy_y = 0;
uint16_t dma_buffer[2] = {0, 0};


/*escaner bus I2C*/
uint8_t msg_buffer[64] = {0};

/* Uso UART */
volatile uint8_t rx_data = 0;       	// Actúa como buffer temporal para recibir 1 solo byte por hardware
volatile uint8_t flag_rx = 0;       	// Indica a la FSM que llegó un nuevo carácter válido
volatile uint8_t rx_char = 0;			// Guarda el carácter recibido de forma segura para procesarlo
volatile uint32_t serial_pwm_width = 0; // Almacena el valor del ciclo de trabajo (PWM) calculado por comandos seriales
volatile int32_t serial_pwm_percent = 0;// Almacena el porcentaje del PWM (0-100) ingresado por puerto serial


/* FSM (Máquinas de Estados) */
volatile uint8_t general_state = 0; 	// Controla la FSM Principal: 0=ENCODER, 1=UART, 2=ADC, 3=REFRESCO SERIAL
volatile uint8_t uart_state = 0; 		// Controla la Sub-FSM UART: 0=IDLE, 1=PROCESAR, 2=MODIFICAR_PWM, 3=ACCION_EXTRA



/*CABECERA DE FUNCIONES (PROTOTIPOS)*/

void SystemClock_Config(void);   // Configura la señal de reloj principal del sistema a 16 MHz
void gpio_Init(void);			// Configura el pin PH1 como salida para el LED Blinky
void tim2_Init(void);			// Configura el Timer 2 para llevar la base de tiempo de refresco de 20 ms
void tim4_blinky_Init(void);		// Configura el Timer 4 para interrumpir cada 250 ms
void uart2_Init(void);			// Configura los pines y parámetros de la comunicación serial
void MCO1_GPIO_Init(void);				// Configura el pin PA8 para sacar la señal de reloj del sistema (MCO1)
void I2C_Init(void);
void escanear_bus_i2c_uart(void);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_init(void);
void lcd_send_string(char *str);
void RTC_init(void);
void joystick_init(void);
void Joystick_Start(void);

/*FUNCION PRINCIPAL (MAIN)*/

int main(void) {
	HAL_Init();                         // Inicializa la librería HAL y los servicios básicos del microcontrolador
	SystemClock_Config();               // Ajusta el reloj interno a la velocidad deseada

    // Inicialización de Periféricos
    gpio_Init();						// Inicializa el pin del LED (Blinky)
	tim4_blinky_Init(); 				// Arranca el temporizador del LED Blinky a 250 ms
	uart2_Init(); 						// Abre el puerto serial para transmisión y recepción
	MCO1_GPIO_Init(); 					// Habilita la salida del reloj de prueba
	I2C_Init();
	escanear_bus_i2c_uart();
	RTC_init();
	joystick_init();
	Joystick_Start();




	/* ACTIVAR RECEPCIÓN POR INTERRUPCIÓN */
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx_data, 1); // Pone al hardware a escuchar el primer carácter serial


	/* === INICIO DE LA PANTALLA LCD === */
	lcd_init();

		// Escribir en la primera línea (Comando 0x80)
	lcd_send_cmd(0x80);
	lcd_send_string("Hola Mundo!");

		// Escribir en la segunda línea (Comando 0xC0)
	lcd_send_cmd(0xC0);
	lcd_send_string("STM32 Lab F411");
		/* ================================= */

	while (1) {
	        // Tu máquina de estados (FSM) irá aquí
		// Refrescar el LCD cada 1000 milisegundos (1 segundo)


		            /*
		             * Siempre debes leer el Date DESPUÉS del Time.
		             * Al leer 'GetTime', los registros se bloquean para evitar desfases,
		             * y solo se desbloquean al leer 'GetDate'.
		             */
		            HAL_RTC_GetTime(&hrtc1, &time, RTC_FORMAT_BIN);
		            HAL_RTC_GetDate(&hrtc1, &date, RTC_FORMAT_BIN);

		            // --- Fila 1: Mostrar la Hora ---
		            lcd_send_cmd(0x80);
		            // %02d asegura que siempre haya 2 dígitos (ej: 05 en vez de 5)
		            sprintf(lcd_buffer, "Hora: %02d:%02d:%02d", time.Hours, time.Minutes, time.Seconds);
		            lcd_send_string(lcd_buffer);

		            // --- Fila 2: Mostrar la Fecha ---
		            lcd_send_cmd(0xC0);
		            sprintf(lcd_buffer, "Fecha: %02d/%02d/%02d", date.Date, date.Month, date.Year);
		            lcd_send_string(lcd_buffer);


	}
}


/*CONFIGURACION DE RELOJ*/					// SE DEJA CONFIGURADO POR DEFECTO COMO SE HIZO EN CLASE

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };       // Crea estructura para el oscilador
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };       // Crea estructura para la distribución de reloj

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // Selecciona el oscilador interno (HSI)
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;            // Enciende el oscilador HSI
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Usa la calibración de fábrica
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 400;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);              // Aplica los parámetros al hardware

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2; // Selecciona todos los buses
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Pasa el oscilador interno al reloj principal del sistema
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  // Ajusta divisor principal (HCLK) a 100 MHz
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   // Ajusta bus periférico 1 a 50 MHz
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   // Ajusta bus periférico 2 a 100 MHz

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3); // Carga la configuración con 0 estados de espera para la Flash

}

/*CONFIGURACION DE PINES (GPIO)*/

void gpio_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 }; 	        // Crea estructura para administrar los pines

	__HAL_RCC_GPIOH_CLK_ENABLE();				        // Enciende la señal de reloj para el puerto H

	GPIO_InitStruct.Pin = GPIO_PIN_1;			        // Selecciona el Pin 1
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;	        // Configura el pin como Salida Digital (Push-Pull)
	GPIO_InitStruct.Pull = GPIO_NOPULL;			        // Desactiva resistencias internas (Ni Pull-up, Ni Pull-down)
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;        // Ajusta la velocidad de conmutación en "Baja"
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);		        // Carga y aplica la configuración a los registros físicos FSR
	__NOP();
}



/*CONFIGURACION TIMER 4 (Blinky / 250ms)*/

void tim4_blinky_Init(void) {
	__HAL_RCC_TIM4_CLK_ENABLE();				        // Otorga reloj al periférico TIM4

	htim4.Instance = TIM4;						        // Asigna el periférico correspondiente
	htim4.Init.Prescaler = 99999;     			        // Ajusta el prescaler para obtener pulsos de 1 milisegundo
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;        // Establece conteo incremental
	htim4.Init.Period = 249;       				        // Pide que resbale o se desborde al llegar a 250 milisegundos
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  // Anula divisiones de reloj auxiliares
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // Permite amortiguación de parámetros en caliente
	HAL_TIM_Base_Init(&htim4);                          // Finaliza la configuración en hardware

	HAL_NVIC_EnableIRQ(TIM4_IRQn);                      //matricula la interrupcion en el NVIC

	HAL_TIM_Base_Start_IT(&htim4);                      // Ordena al temporizador iniciar y emitir alertas cada fin de ciclo
	__NOP();
}


/*CONFIGURACION SERIAL (USART2)*/

void uart2_Init(void) {
	__HAL_RCC_USART2_CLK_ENABLE();                      // Autoriza suministro de reloj al módulo USART2
	__HAL_RCC_GPIOA_CLK_ENABLE();                       // Autoriza suministro de reloj al banco de pines A

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };           // Genera matriz de ajustes de Pin

	GPIO_InitStruct.Pin = GPIO_PIN_2;                   // Configura el Pin físico TX (Transmisor)
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;             // Otorga propiedad del pin a la comunicación Alternativa
	GPIO_InitStruct.Pull = GPIO_PULLUP;                 // Añade resistencia elevadora de seguridad
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  // Escala su tiempo de respuesta al máximo
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;        // Vincula el Pin al canal Serial número 2
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);             // Fija cambios

	GPIO_InitStruct.Pin = GPIO_PIN_3;                   // Configura el Pin físico RX (Receptor)
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;             // Otorga propiedad de lectura alterna
	GPIO_InitStruct.Pull = GPIO_PULLUP;                 // Añade resistencia elevadora de seguridad
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  // Escala tiempo de respuesta de recepción
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;        // Lo conecta al túnel del USART2
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);             // Fija cambios

	huart2.Instance = USART2;                           // Selecciona unidad USART2
	huart2.Init.BaudRate = 19200;                       // Establece la velocidad de bits por segundo
	huart2.Init.WordLength = UART_WORDLENGTH_8B;        // Estipula que cada letra ocupará 8 bits
	huart2.Init.StopBits = UART_STOPBITS_1;             // Indica cuántos bits avisan que la letra finalizó
	huart2.Init.Parity = UART_PARITY_NONE;              // Apaga la verificación de seguridad por paridad
	huart2.Init.Mode = UART_MODE_TX_RX;                 // Decreta que funcionará bidireccionalmente
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;        // Anula el control de flujo físico
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;    // Muestrea cada bit 16 veces para rechazar ruidos
	HAL_UART_Init(&huart2);                             // Activa el subsistema y sella la configuración

	HAL_NVIC_EnableIRQ(USART2_IRQn);                    //matricula la interrupcion en el NVIC
}


void I2C_Init(void){

	__HAL_RCC_I2C1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	HAL_I2C_Init(&hi2c1);


}
/*CONFIGURACION SALIDA RELOJ DE PRUEBA*/

void MCO1_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };           // Constructor del pin de salida

	__HAL_RCC_GPIOA_CLK_ENABLE();                       // Despierta Puerto A

	GPIO_InitStruct.Pin = GPIO_PIN_8;                   // Focaliza pin 8
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;             // Adjudica labor al oscilador maestro interno
	GPIO_InitStruct.Pull = GPIO_NOPULL;                 // Limpia configuración de resistencia
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  // Garantiza que es capaz de encender y apagar a 16 millones de veces por segundo
	GPIO_InitStruct.Alternate = GPIO_AF0_MCO;           // Apunta la salida del sistema directo al Pin 8
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);             // Completa trámite

	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1); // Conecta físicamente el reloj interno al pin sin usar divisiones
}

/* FUNCION PARA ESCANEAR EL BUS I2C Y ENVIAR RESULTADOS POR UART */
void escanear_bus_i2c_uart(void) {
	HAL_StatusTypeDef resultado;

	// Imprimir encabezado usando el buffer global
	sprintf((char*)msg_buffer, "\r\n--- Iniciando Escaner I2C ---\r\n");
	HAL_UART_Transmit(&huart2, msg_buffer, strlen((char*)msg_buffer), 100);

	// Ciclo para buscar en todas las direcciones posibles (1 a 127)
	for (uint8_t i = 1; i < 128; i++) {
		// HAL requiere la dirección desplazada 1 bit a la izquierda
		resultado = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 2, 2);

		if (resultado == HAL_OK) {
			// Si el dispositivo responde, armamos el mensaje con la dirección en Hexadecimal
			sprintf((char*)msg_buffer, "Dispositivo encontrado en: 0x%02X\r\n", i);
			HAL_UART_Transmit(&huart2, msg_buffer, strlen((char*)msg_buffer), 100);
		}
	}

	// Mensaje de finalización
	sprintf((char*)msg_buffer, "--- Escaneo Terminado ---\r\n\r\n");
	HAL_UART_Transmit(&huart2, msg_buffer, strlen((char*)msg_buffer), 100);
}

void lcd_send_cmd(char cmd) {
    char data_u, data_l;
    uint8_t data_t[4];

    // Extraemos la mitad superior e inferior del comando
    data_u = (cmd & 0xF0);
    data_l = ((cmd << 4) & 0xF0);

    // Empaquetamos: Datos + Luz (0x08) + Enable (0x04) + RS (0x00 para comandos)
    data_t[0] = data_u | 0x0C;  // EN=1, RS=0
    data_t[1] = data_u | 0x08;  // EN=0, RS=0 (Pulso de bajada)
    data_t[2] = data_l | 0x0C;  // EN=1, RS=0
    data_t[3] = data_l | 0x08;  // EN=0, RS=0

    // Transmitimos los 4 paquetes de golpe
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_t, 4, 100);
}

void lcd_send_data(char data) {
    char data_u, data_l;
    uint8_t data_t[4];

    // Extraemos la mitad superior e inferior de la letra
    data_u = (data & 0xF0);
    data_l = ((data << 4) & 0xF0);

    // Empaquetamos: Datos + Luz (0x08) + Enable (0x04) + RS (0x01 para texto)
    data_t[0] = data_u | 0x0D;  // EN=1, RS=1
    data_t[1] = data_u | 0x09;  // EN=0, RS=1 (Pulso de bajada)
    data_t[2] = data_l | 0x0D;  // EN=1, RS=1
    data_t[3] = data_l | 0x09;  // EN=0, RS=1

    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_t, 4, 100);
}

void lcd_init(void) {
    HAL_Delay(50); // Esperar que el voltaje se estabilice

    // Secuencia de inicialización obligatoria
    lcd_send_cmd(0x30); HAL_Delay(5);
    lcd_send_cmd(0x30); HAL_Delay(1);
    lcd_send_cmd(0x30); HAL_Delay(10);

    // Cambiar a modo 4 bits
    lcd_send_cmd(0x20); HAL_Delay(10);

    // Ajustes de pantalla
    lcd_send_cmd(0x28); HAL_Delay(1); // Modo 4 bits, multiples lineas, fuente 5x8
    lcd_send_cmd(0x08); HAL_Delay(1); // Apagar display
    lcd_send_cmd(0x01); HAL_Delay(2); // Limpiar pantalla
    lcd_send_cmd(0x06); HAL_Delay(1); // Cursor de izq a der
    lcd_send_cmd(0x0C); HAL_Delay(1); // Encender display sin cursor
}

void lcd_send_string(char *str) {
    // Mientras el carácter no sea el final de la cadena ('\0')
    while (*str) {
        lcd_send_data(*str++); // Envía la letra y avanza al siguiente espacio
    }
}

void RTC_init(void){
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };


	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE; // Selecciona el oscilador interno (HSI)
	RCC_OscInitStruct.LSEState= RCC_LSI_ON;            // Enciende el oscilador HSI
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;      // Desactiva el multiplicador (PLL)
	HAL_RCC_OscConfig(&RCC_OscInitStruct);              // Aplica los parámetros al hardware


	__HAL_RCC_PWR_CLK_ENABLE();

	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSE_CONFIG(RCC_LSE_ON);

	while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET) {

	}

	__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);
	__HAL_RCC_RTC_ENABLE();

	hrtc1.Instance = RTC;
	hrtc1.Init.HourFormat= RTC_HOURFORMAT_24;
	hrtc1.Init.AsynchPrediv = 127;
	hrtc1.Init.SynchPrediv = 255;
	hrtc1.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc1.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_LOW;
	hrtc1.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

	HAL_RTC_Init(&hrtc1);

	if(HAL_RTCEx_BKUPRead(&hrtc1, RTC_BKP_DR0) !=0x0001){

		RTC_TimeTypeDef time ={0};
		RTC_DateTypeDef date ={0};

		//HORA INICIAL

		time.Hours=12;
		time.Minutes=0;
		time.Seconds=0;

		HAL_RTC_SetTime(&hrtc1, &time, RTC_FORMAT_BIN);

		date.Month= RTC_MONTH_JULY;
		date.Date=23;
		date.Year = 26;

		HAL_RTC_SetDate(&hrtc1, &date, RTC_FORMAT_BIN);

	}

//firma
	HAL_RTCEx_BKUPWrite(&hrtc1, RTC_BKP_DR0, 0x0001);//REGISTRO DE BACKUP NO SE BORRAN

}
void joystick_init(void){

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Analog pins for X and Y axes
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	hdma1.Instance = DMA2_Stream0;
	hdma1.Init.Channel = DMA_CHANNEL_0;
	hdma1.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma1.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma1.Init.MemInc = DMA_MINC_ENABLE;
	hdma1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma1.Init.Mode = DMA_CIRCULAR;
	hdma1.Init.Priority = DMA_PRIORITY_LOW;

	HAL_DMA_Init(&hdma1);


	__HAL_LINKDMA(&hadc1, DMA_Handle, hdma1);

	//CONFIGURACION ADC1
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;//REVISAR SI ES NECESARIO
	hadc1.Init.DMAContinuousRequests = ENABLE;

	HAL_ADC_Init(&hadc1);

	// Configure ADC Channel 0 (PA0 - X Axis)
	ADC_ChannelConfTypeDef adc_config = {0};
	adc_config.Channel = ADC_CHANNEL_0;
	adc_config.Rank = 1;
	adc_config.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &adc_config);

	    // Configure ADC Channel 1 (PA1 - Y Axis)
	adc_config.Channel = ADC_CHANNEL_1;
	adc_config.Rank = 2;
	adc_config.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &adc_config);

	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

		__NOP();
}
void Joystick_Start(void) {
    // Point the DMA to the hidden array
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dma_buffer, 2);
}

/*RUTINAS DE INTERRUPCIÓN (CALLBACKS)*/


/* Se ejecuta automáticamente cada que un Timer finaliza su cuenta máxima */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) {                       // Verifica que la llamada proviene exclusivamente del TIM4 (250ms)
		HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);          // Cambia el estado actual del LED de placa (Blinky)

	}
}


/* Se ejecuta automáticamente al culminar la recepción satisfactoria de bytes por puerto serial */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {                    // Revisa que sea el cable serial del PC
		rx_char = rx_data;                              // Transfiere el carácter hacia rx_data
		flag_rx = 1;                                    // Dictamina dictamina el evento para ser procesado

		HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx_data, 1); // Carga la recámara vacía para escuchar al siguiente comando
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        // The DMA just finished filling dma_buffer.
        // Update your individual global variables instantly!
        joy_x = dma_buffer[0];
        joy_y = dma_buffer[1];
    }
}

