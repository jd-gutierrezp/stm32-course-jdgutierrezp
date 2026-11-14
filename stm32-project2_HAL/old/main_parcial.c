//main.c 			parcial
//Author: juand

/*INCLUDES*/

#include "stm32f4xx_hal.h"  // Incluye la librería principal de abstracción de hardware (HAL)
#include "stdio.h"          // Incluye funciones estándar de entrada/salida (como sprintf)
#include "string.h"         // Incluye funciones para manejo de cadenas de texto (como strlen)
#include <stdlib.h>			// biblioteca estandar de proposito general
#define LCD_ADDR (0x22 << 1) // Dirección I2C del LCD desplazada 1 bit a la izq (formato HAL)

/* HANDLERS DE PERIFÉRICOS (Estructuras de control de la HAL) */
TIM_HandleTypeDef htim4 = { 0 }; // Timer 4: Interrupción base de tiempo (250ms)
UART_HandleTypeDef huart2 = { 0 }; // USART 2: Comunicación serial al PC
RTC_HandleTypeDef hrtc1 = { 0 };   // RTC: Reloj en tiempo real
RTC_TimeTypeDef time = { 0 };      // Estructura para almacenar la hora
RTC_DateTypeDef date = { 0 };      // Estructura para almacenar la fecha
I2C_HandleTypeDef hi2c1 = { 0 };   // I2C 1: Comunicación con la pantalla LCD
ADC_HandleTypeDef hadc1 = { 0 };   // ADC 1: Lectura de voltajes del joystick
DMA_HandleTypeDef hdma1 = { 0 }; // DMA 2 (Stream 0): Traspaso automático ADC -> RAM

/* VARIABLES DE MÁQUINA DE ESTADOS Y CONTROL */
volatile uint8_t rx_data = 0; // Buffer de 1 byte para recibir un caracter por hardware (UART)
volatile uint8_t flag_rx = 0; // Bandera: 1 = Nuevo caracter recibido listo para procesar
volatile uint8_t rx_char = 0; // Almacena el caracter validado para el switch del UART
volatile uint8_t general_state = 0; // FSM Principal: 0=UART, 1=LCD, 2=JOYSTICK, 3=REFRESCO
volatile uint8_t uart_state = 0; // FSM UART: 0=IDLE, 1=EVALUAR, 2=SET CLK, 3=SET PCLR
volatile uint8_t flag_refresh_250ms = 0; // Bandera de tiempo activada por el TIM4

/* VARIABLES MCO1 Y RELOJ */
uint32_t mco1_source = RCC_MCO1SOURCE_HSI; // Fuente por defecto para el pin MCO1
volatile uint8_t pclr_mco1 = 0;            // Índice del divisor del MCO1
uint32_t mco1_div = RCC_MCODIV_1;   // Divisor físico por defecto (Sin división)
char *clck_name = "HSI";               // Etiqueta para imprimir el reloj actual
char lcd_buffer[34];  // Cadena de texto para dar formato antes de enviar al LCD

/* VARIABLES JOYSTICK */
volatile uint16_t joy_x = 0;       // Eje X crudo (0-4095)
volatile uint16_t joy_y = 0;       // Eje Y crudo (0-4095)
uint16_t dma_buffer[2] = { 0, 0 }; // Arreglo oculto donde el DMA escribe los 2 canales del ADC
int32_t percent_x = 0;             // Porcentaje calculado eje X (-100 a +100)
int32_t percent_y = 0;             // Porcentaje calculado eje Y (-100 a +100)

//cabecera de funciones
void SystemClock_Config(void); // Configura la señal de reloj principal del sistema a 100 MHz
void gpio_Init(void);	// Configura el pin PH1 como salida para el LED Blinky
void tim4_blinky_Init(void);// Configura el Timer 4 para interrumpir cada 250 ms
void uart2_Init(void);// Configura los pines y parámetros de la comunicación serial
void RTC_init(void);
void I2C_Init(void);
void joystick_init(void);
void Joystick_Start(void);
void MCO1_GPIO_Init(void);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_init(void);
void lcd_send_string(char *str);
void format_joystick_center_bar(char axis_label, uint16_t raw_adc_value,
		char *buffer);

int main(void) {
	HAL_Init(); // Inicializa el Systick y el NVIC (controlador de interrupciones)
	SystemClock_Config();     // Configura el PLL para correr el núcleo a 100MHz
	gpio_Init();                // Configura el pin PH1 (LED)
	tim4_blinky_Init();         // Inicia el contador de hardware del TIM4
	uart2_Init();               // Configura pines PA2/PA3 a 19200 baudios
	RTC_init();           // Enciende el cristal LSE (32.768kHz) y configura RTC
	I2C_Init();                 // Configura pines PB6/PB7 a 100kHz para el LCD
	MCO1_GPIO_Init(); // Configura PA8 en Alternate Function 0 (Señal de reloj de salida)
	joystick_init(); // Configura PA0/PA1 en modo Analógico y enlaza el DMA al ADC
	Joystick_Start(); // Dispara el ADC por software y activa el flujo continuo del DMA

	HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx_data, 1); // Arma la interrupción UART para recibir exactamente 1 byte de forma asíncrona

	lcd_init();    // Envía los comandos de inicialización HD44780 a la pantalla

	while (1) {
		switch (general_state) { // Evalúa en qué estado de la máquina principal se encuentra

		case 0: 					// ESTADO GENERAL 0:UART

			switch (uart_state) { // Evalúa en qué sub-estado de recepción se encuentra

			case 0: // Sub-Estado 0: IDLE (Reposo)
				if (flag_rx == 1) { // Verifica si llegó una nueva letra desde el PC
					flag_rx = 0;    // Apaga la bandera de aviso
					uart_state = 1; // Pasa al estado de procesar la letra
				}
				break;

			case 1: // Sub-Estado 1: PROCESAR COMANDOS
				switch (rx_char) {
				// Si es H, L o P (mayúscula o minúscula) -> Va al Estado 2
				case 'H':
				case 'h':
				case 'L':
				case 'l':
				case 'P':
				case 'p':
					uart_state = 2;
					break;

					// Si es '+' o '-' -> Va al Estado 3
				case '+':
				case '-':
					uart_state = 3;
					break;
				default:       // Si el usuario presiona una tecla no registrada
					char serial_invalid_msg[] = "Comando invalido\r\n";
					HAL_UART_Transmit(&huart2, (uint8_t*) serial_invalid_msg,
							strlen(serial_invalid_msg) - 1, 100); // Envía mensaje de error
					uart_state = 0; // Regresa al reposo a esperar una nueva tecla
					break;
				}

				break;

			case 2: // Sub-Estado 2: CAMBIAR CLCK
			{
				switch (rx_char) {
				case 'H':
				case 'h':
					mco1_source = RCC_MCO1SOURCE_HSI;     // pone le CLCK en HSI
					clck_name = "HSI";
					break;
				case 'L':
				case 'l':
					mco1_source = RCC_MCO1SOURCE_LSE; // pone le CLCK en LSE
					clck_name = "LSE";
					break;

				case 'P':
				case 'p':
					mco1_source = RCC_MCO1SOURCE_PLLCLK; // pone le CLCK en PLL
					clck_name = "PLL";
					break;
				}

				HAL_RCC_MCOConfig(RCC_MCO1, mco1_source, mco1_div);

				char source_msg[50];
				sprintf(source_msg, "CLCK %s\r\n", clck_name); // envia un texto de confirmación
				HAL_UART_Transmit(&huart2, (uint8_t*) source_msg,
						strlen(source_msg) - 1, 100); // Envía la confirmación al serial

				uart_state = 0; // Regresa al reposo
				break;
			}

			case 3: // Sub-Estado 3: CAMBIAR PCLR--otros dos cararcteres que modigican el PLCR del CLCK
			{
				switch (rx_char) {
				case '+':
					if (pclr_mco1 < 4) {
						pclr_mco1++; // Solo suma si es menor que el máximo
					}
					break;

				case '-':
					if (pclr_mco1 > 0) {
						pclr_mco1--; // Solo resta si es mayor que el mínimo
					}
					break;
				}

				if (pclr_mco1 == 0)	//aqui se hace lamodificacion del precales deacuerso a los comandos ateriores
					mco1_div = RCC_MCODIV_1;
				else if (pclr_mco1 == 1)
					mco1_div = RCC_MCODIV_2;
				else if (pclr_mco1 == 2)
					mco1_div = RCC_MCODIV_3;
				else if (pclr_mco1 == 3)
					mco1_div = RCC_MCODIV_4;
				else if (pclr_mco1 == 4)
					mco1_div = RCC_MCODIV_5;

				HAL_RCC_MCOConfig(RCC_MCO1, mco1_source, mco1_div);	//aqui actiliza la salida del MCO, segun el CLCK y el prescaler

				uart_state = 0; // Regresa al reposo
				break;
			}

				general_state = 1; //Cede el turno al estado encargado de la pnatalla LCD
				break;
			}
		case 1:				// ESTADO GENERAL 0:LCD
			//recoge los datos de la hora y fecha del rtc
			HAL_RTC_GetTime(&hrtc1, &time, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc1, &date, RTC_FORMAT_BIN);

			//Fila 1: Mostrar la Hora
			lcd_send_cmd(0x80);	// 0x80: Mueve el cursor a la Fila 1, Columna 0
			sprintf(lcd_buffer, "Hora: %02d:%02d:%02d", time.Hours,
					time.Minutes, time.Seconds);
			lcd_send_string(lcd_buffer);

			//Fila 2: Mostrar la Fecha
			lcd_send_cmd(0xC0);	// 0xC0: Mueve el cursor a la Fila 2, Columna 0
			sprintf(lcd_buffer, "Fecha: %02d/%02d/%02d", date.Date, date.Month,
					date.Year);
			lcd_send_string(lcd_buffer);

			//Fila 3: Mostrar Joystick X
			lcd_send_cmd(0x94); // Dirección DDRAM Fila 3 (0x80 | 0x14)
			sprintf(lcd_buffer, "X Percent: %ld%%   ", percent_x);
			lcd_send_string(lcd_buffer);

			// Fila 4: Mostrar Joystick Y
			lcd_send_cmd(0xD4); // Dirección DDRAM Fila 4 (0x80 | 0x54)
			sprintf(lcd_buffer, "Y Percent: %ld%%   ", percent_y);
			lcd_send_string(lcd_buffer);
			general_state = 2; //Cede el turno al estado encargado del joystick
			break;

			general_state = 2; // Cede el turno al estado encargado del joystick
			break;

		case 2: //// ESTADO GENERAL 0:JOYSTICK

			percent_x = ((joy_x * 200) / 4095) - 100;
			percent_y = ((joy_y * 200) / 4095) - 100;
			if (percent_x <= 1 && percent_x >= -1) { //condicion para evitar pardadeo en 0
				percent_x = 0;
				percent_x = 0;
			}
			if (percent_y <= 1 && percent_y >= -1) {
				percent_y = 0;
				percent_y = 0;
			}
			general_state = 3; //Cede el turno al estado encargado del refresco
			break;

		case 3:				////ESTADO GENERAL 3: REFRESCO

			if (flag_refresh_250ms == 1) { // Comprueba si el Timer 4 levantó la bandera de los 250ms
				flag_refresh_250ms = 0; // Baja la bandera inmediatamente para evitar reingresos
				HAL_RTC_GetTime(&hrtc1, &time, RTC_FORMAT_BIN); //se recoge denuevo los datos de la hora y fecha actulizada
				HAL_RTC_GetDate(&hrtc1, &date, RTC_FORMAT_BIN);
				char serial_refresh_msg[120]; // Reserva memoria para el mensaje final
				sprintf(serial_refresh_msg,
						"Hora: %02u:%02u | Date: %02u/%02u/%02u | CLCK: %s | PCLR: %d| X:%ld%% | Y:%ld%% \r\n",
						time.Hours, time.Minutes, date.Date, date.Month,
						date.Year, clck_name, pclr_mco1, percent_x, percent_y); // Fusiona los valores de las variables en una sola frase de texto

				HAL_UART_Transmit(&huart2, (uint8_t*) serial_refresh_msg,
						strlen(serial_refresh_msg) - 1, 100); // Envía el reporte a la terminal
			}
			general_state = 0; //Cede el turno al estado encargado del refresco
			break;

		}
	}

}
/*CONFIGURACION DE RELOJ*/
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 }; // Crea estructura para el oscilador
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 }; // Crea estructura para la distribución de reloj

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // Habilitamos HSI para alimentar el PLL
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;                // Enciende RC 16MHz
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;   // Enciende Phase-Locked Loop
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; // HSI es la entrada al PLL
	RCC_OscInitStruct.PLL.PLLM = 16; // M=16 -> 16MHz / 16 = 1MHz (entrada al VCO)
	RCC_OscInitStruct.PLL.PLLN = 400; // N=400 -> 1MHz * 400 = 400MHz (frecuencia VCO)
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; // P=4 -> 400MHz / 4 = 100MHz (Salida final SYSCLK)

	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2; // Selecciona todos los buses
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Pasa el oscilador interno al reloj principal del sistema
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // Ajusta divisor principal en 1 para que (HCLK) a 100 MHz
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; // Ajusta bus periférico 1 a 50 MHz
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; // Ajusta bus periférico 2 a 100 MHz

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3); // segun el manula si corre entre 84 y 100MHz se debe poder 3 de latencia.

}
void gpio_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 }; // Crea estructura para administrar los pines

	__HAL_RCC_GPIOH_CLK_ENABLE(); // Enciende la señal de reloj para el puerto H

	GPIO_InitStruct.Pin = GPIO_PIN_1;			        // Selecciona el Pin 1
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;	// Configura el pin como Salida Digital (Push-Pull)
	GPIO_InitStruct.Pull = GPIO_NOPULL;	// Desactiva resistencias internas (Ni Pull-up, Ni Pull-down)
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Ajusta la velocidad de conmutación en "Baja"
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);	// Carga y aplica la configuración a los registros físicos FSR
	__NOP();
}
void tim4_blinky_Init(void) {
	__HAL_RCC_TIM4_CLK_ENABLE();			// Otorga reloj al periférico TIM4

	htim4.Instance = TIM4;				// Asigna el periférico correspondiente
	htim4.Init.Prescaler = 9999; // Ajusta el prescaler para obtener pulsos de 1 milisegundo
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP; // Establece conteo incremental
	htim4.Init.Period = 2499; // Pide que resbale o se desborde al llegar a 250 milisegundos
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // Anula divisiones de reloj auxiliares
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // Permite amortiguación de parámetros en caliente
	HAL_TIM_Base_Init(&htim4);          // Finaliza la configuración en hardware

	HAL_NVIC_EnableIRQ(TIM4_IRQn);        //matricula la interrupcion en el NVIC

	HAL_TIM_Base_Start_IT(&htim4); // Ordena al temporizador iniciar y emitir alertas cada fin de ciclo
	__NOP();
}
void uart2_Init(void) {
	__HAL_RCC_USART2_CLK_ENABLE(); // Autoriza suministro de reloj al módulo USART2
	__HAL_RCC_GPIOA_CLK_ENABLE(); // Autoriza suministro de reloj al banco de pines A

	GPIO_InitTypeDef GPIO_InitStruct = { 0 }; // Genera estructura de ajustes de Pin

	GPIO_InitStruct.Pin = GPIO_PIN_2; // Configura el Pin físico TX (Transmisor)
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Otorga propiedad del pin a la comunicación Alternativa
	GPIO_InitStruct.Pull = GPIO_PULLUP; // Añade resistencia elevadora de seguridad
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // Escala su tiempo de respuesta al máximo
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2; // Vincula el Pin al canal Serial número 2
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);             // Fija cambios

	GPIO_InitStruct.Pin = GPIO_PIN_3;   // Configura el Pin físico RX (Receptor)
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Otorga propiedad de lectura alterna
	GPIO_InitStruct.Pull = GPIO_PULLUP; // Añade resistencia elevadora de seguridad
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // Escala tiempo de respuesta de recepción
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2; // Lo conecta al túnel del USART2
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);             // Fija cambios

	huart2.Instance = USART2;                        // Selecciona unidad USART2
	huart2.Init.BaudRate = 19200;  // Establece la velocidad de bits por segundo
	huart2.Init.WordLength = UART_WORDLENGTH_8B; // Estipula que cada letra ocupará 8 bits
	huart2.Init.StopBits = UART_STOPBITS_1; // Indica cuántos bits avisan que la letra finalizó
	huart2.Init.Parity = UART_PARITY_NONE; // Apaga la verificación de seguridad por paridad
	huart2.Init.Mode = UART_MODE_TX_RX; // Decreta que funcionará bidireccionalmente
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; // Anula el control de flujo físico
	huart2.Init.OverSampling = UART_OVERSAMPLING_16; // Muestrea cada bit 16 veces para rechazar ruidos
	HAL_UART_Init(&huart2);     // Activa el subsistema y sella la configuración

	HAL_NVIC_EnableIRQ(USART2_IRQn);      //matricula la interrupcion en el NVIC
}
void RTC_init(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	__HAL_RCC_PWR_CLK_ENABLE();	// 1. Necesario para acceder a registros de energía (RM Cap 5)

	HAL_PWR_EnableBkUpAccess(); //2. Desbloquea la escritura en el dominio de Backup (RTC y registros BKP)

	__HAL_RCC_LSE_CONFIG(RCC_LSE_ON);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE; // Selecciona el oscilador interno (LSE)
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;        // Enciende el oscilador LSE
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE; // Desactiva el multiplicador (PLL)
	HAL_RCC_OscConfig(&RCC_OscInitStruct);  // Aplica los parámetros al hardware

	while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET) {

	}

	__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);
	__HAL_RCC_RTC_ENABLE();

	hrtc1.Instance = RTC;
	hrtc1.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc1.Init.AsynchPrediv = 127;              // Divisor asíncrono (128 - 1)
	hrtc1.Init.SynchPrediv = 255; // Divisor síncrono (256 - 1). 32768 / (128*256) = 1Hz exacto
	hrtc1.Init.OutPut = RTC_OUTPUT_DISABLE; // No queremos emitir señal de alarma por pin
	hrtc1.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_LOW;
	hrtc1.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	HAL_RTC_Init(&hrtc1);

	// Lee el registro 0 de backup. Si no es 0x0001, significa que hubo un corte total de energía (VBAT murió)
	if (HAL_RTCEx_BKUPRead(&hrtc1, RTC_BKP_DR0) != 0x0001) {

		RTC_TimeTypeDef time = { 0 };
		RTC_DateTypeDef date = { 0 };

		//HORA INICIAL

		time.Hours = 17;
		time.Minutes = 10;
		time.Seconds = 0;

		HAL_RTC_SetTime(&hrtc1, &time, RTC_FORMAT_BIN);

		date.Month = RTC_MONTH_JULY;
		date.Date = 26;
		date.Year = 26;

		HAL_RTC_SetDate(&hrtc1, &date, RTC_FORMAT_BIN);
		//firma
	}
	//firma
	HAL_RTCEx_BKUPWrite(&hrtc1, RTC_BKP_DR0, 0x0001); //REGISTRO DE BACKUP NO SE BORRAN
}
void I2C_Init(void) {
	__HAL_RCC_I2C1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7; // PB6=SCL, PB7=SDA
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; // I2C obliga a usar Open-Drain (colector abierto)
	GPIO_InitStruct.Pull = GPIO_PULLUP;            // Requiere pull-ups de bus
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;  // Mapeado a AF4 según datasheet
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;                // 100 kHz (I2C Estándar)
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;      // Relación de trabajo estándar
	hi2c1.Init.OwnAddress1 = 0; // MCU actúa como Maestro, no requiere dirección
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; // Direccionamiento clásico de 7 bits
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; // Permite al esclavo alargar el ciclo de reloj (Clock Stretching)
	HAL_I2C_Init(&hi2c1);

}

void joystick_init(void) {

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE(); // El ADC1 en el F411 está cableado obligatoriamente al DMA2

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1; // PA0=Canal0 (Y), PA1=Canal1 (X)
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG; // Modo netamente analógico (apaga el buffer digital del pin)
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	hdma1.Instance = DMA2_Stream0;     // Stream 0 es el único que sirve al ADC1
	hdma1.Init.Channel = DMA_CHANNEL_0;            // Canal de hardware del DMA
	hdma1.Init.Direction = DMA_PERIPH_TO_MEMORY; // Lee del Periférico (ADC) hacia la Memoria (RAM)
	hdma1.Init.PeriphInc = DMA_PINC_DISABLE; // La dir. del ADC siempre es fija (registro DR)
	hdma1.Init.MemInc = DMA_MINC_ENABLE; // SÍ incrementa la dirección en RAM para llenar el array dma_buffer
	hdma1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // Tamaño transferido: 16 bits (HalfWord)
	hdma1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD; // Tamaño almacenado: 16 bits
	hdma1.Init.Mode = DMA_CIRCULAR; // Al llegar al final del array, vuelve a empezar infinitamente
	hdma1.Init.Priority = DMA_PRIORITY_LOW;
	HAL_DMA_Init(&hdma1);
	__HAL_LINKDMA(&hadc1, DMA_Handle, hdma1); // Enlaza estructuralmente el ADC con este DMA

	//CONFIGURACION ADC1
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // 100MHz(APB2) / 4 = 25MHz reloj ADC
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;           // 0 a 4095
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT; // Bits hacia la derecha en la variable
	hadc1.Init.ScanConvMode = ENABLE; // Escaneará múltiples canales (CH0 y CH1)
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.ContinuousConvMode = ENABLE; // Al terminar CH1, vuelve a empezar en CH0 solo
	hadc1.Init.NbrOfConversion = 2;                 // Secuencia de 2 mediciones
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START; // Se dispara por código (HAL_ADC_Start)
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; // Como es por software, ignorar hardware externo
	hadc1.Init.DMAContinuousRequests = ENABLE; // ADC avisa al DMA en cada conversión terminada
	HAL_ADC_Init(&hadc1);

	// Configure ADC Channel 0 (PA0 - Y Axis)
	ADC_ChannelConfTypeDef adc_config = { 0 };
	adc_config.Channel = ADC_CHANNEL_0;
	adc_config.Rank = 1;	// Primer canal a convertir en la secuencia
	adc_config.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &adc_config);

	// Configure ADC Channel 1 (PA1 - X Axis)
	adc_config.Channel = ADC_CHANNEL_1;
	adc_config.Rank = 2;	// Segundo canal a convertir
	adc_config.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &adc_config);

	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

	__NOP();
}
void Joystick_Start(void) { //REVISION TRASLADAR A INICION SOLO LLAMAR UNA VEZ
	// Ordena al ADC arrancar, y al DMA guardar resultados en nuestro dma_buffer. Se llama 1 sola vez por ser CIRCULAR
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) dma_buffer, 2);
}
void MCO1_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };   // Constructor del pin de salida

	__HAL_RCC_GPIOA_CLK_ENABLE();                       // Despierta Puerto A

	GPIO_InitStruct.Pin = GPIO_PIN_8;                   // Focaliza pin 8
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Adjudica labor al oscilador maestro interno
	GPIO_InitStruct.Pull = GPIO_NOPULL;   // Limpia configuración de resistencia
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // Garantiza que es capaz de encender y apagar a 16 millones de veces por segundo
	GPIO_InitStruct.Alternate = GPIO_AF0_MCO; // Apunta la salida del sistema directo al Pin 8
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);             // Completa trámite

	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1); // Conecta físicamente el reloj interno al pin sin usar divisiones
}

void lcd_send_cmd(char cmd) {
	char data_u, data_l;
	uint8_t data_t[4];

	// Extraemos la mitad superior e inferior del comando
	data_u = (cmd & 0xF0);         // Máscara para la parte alta
	data_l = ((cmd << 4) & 0xF0);  // Máscara para la parte baja

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
	HAL_Delay(50); // El controlador del LCD tarda en estabilizar voltajes tras un encendido
	// Secuencia rígida de Datasheet HD44780 para pasar de modo 8-bits a 4-bits por I2C
	lcd_send_cmd(0x30);
	HAL_Delay(5);
	lcd_send_cmd(0x30);
	HAL_Delay(1);
	lcd_send_cmd(0x30);
	HAL_Delay(10);
	lcd_send_cmd(0x20);
	HAL_Delay(10); // Transición oficial a 4-bits
	lcd_send_cmd(0x28);
	HAL_Delay(1);  // 4 bits, 2 líneas, 5x8 font
	lcd_send_cmd(0x08);
	HAL_Delay(1);  // Display off
	lcd_send_cmd(0x01);
	HAL_Delay(2);  // Limpia pantalla
	lcd_send_cmd(0x06);
	HAL_Delay(1);  // Cursor mueve derecha, no hace scroll
	lcd_send_cmd(0x0C);
	HAL_Delay(1);  // Display ON, Cursor OFF
}

void lcd_send_string(char *str) {
	// Mientras el carácter no sea el final de la cadena ('\0')
	while (*str) {
		lcd_send_data(*str++); // Envía la letra y avanza al siguiente espacio
	}
}

//Callbacks
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) { // Verifica que la llamada proviene exclusivamente del TIM4 (250ms)
		HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1); // Cambia el estado actual del LED de placa (Blinky)
		flag_refresh_250ms = 1; // Informa a la Máquina Principal que es hora de actualizar la informaion en el serial
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {    // Revisa que sea el cable serial del PC
		rx_char = rx_data;               // Transfiere el carácter hacia rx_data
		flag_rx = 1;         // Dictamina dictamina el evento para ser procesado

		HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx_data, 1); // Carga la recámara vacía para escuchar al siguiente comando
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		// The DMA just finished filling dma_buffer.
		// Update your individual global variables instantly!
		joy_x = dma_buffer[0];
		joy_y = dma_buffer[1];
	}
}
//usando esta funcion se permite encontrar la direccion de la pantalla
/* FUNCION PARA ESCANEAR EL BUS I2C Y ENVIAR RESULTADOS POR UART */
/*void escanear_bus_i2c_uart(void) {
 HAL_StatusTypeDef resultado;

 // Imprimir encabezado usando el buffer global
 sprintf((char*)msg_buffer, "\r\n--- Iniciando Escaner I2C ---\r\n");
 HAL_UART_Transmit(&huart2, msg_buffer, strlen((char*)msg_buffer), 100);// revisar erro en el uart

 // Ciclo para buscar en todas las direcciones posibles (1 a 127)
 for (uint8_t i = 1; i < 128; i++) {
 // HAL requiere la dirección desplazada 1 bit a la izquierda
 resultado = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 2, 2);

 if (resultado == HAL_OK) {
 // Si el dispositivo responde, armamos el mensaje con la dirección en Hexadecimal
 sprintf((char*)msg_buffer, "Dispositivo encontrado en: 0x%02X\r\n", i);
 HAL_UART_Transmit(&huart2, msg_buffer, strlen((char*)msg_buffer)-1, 100);
 }
 }

 // Mensaje de finalización por medio de uart
 sprintf((char*)msg_buffer, "--- Escaneo Terminado ---\r\n\r\n");
 HAL_UART_Transmit(&huart2, msg_buffer, strlen((char*)msg_buffer)-1, 100);
 }*/
