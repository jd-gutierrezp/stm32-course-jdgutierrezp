//main.c
//Author: juand (Adaptado para Encoders y A4988)

/*INCLUDES*/
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#define LCD_ADDR (0x22 << 1) // Dirección I2C del LCD (formato HAL)
#include <math.h> // Necesario para cálculos con float y redondeo

/* HANDLERS DE PERIFÉRICOS */
TIM_HandleTypeDef htim4 = { 0 }; // Timer 4: Interrupción base de tiempo (250ms)
TIM_HandleTypeDef htim2 = { 0 };   // Timer 2: Encoder 1 (ANT)
TIM_HandleTypeDef htim3 = { 0 };   // Timer 3: Encoder 2 (CAP)
UART_HandleTypeDef huart2 = { 0 }; // USART 2: Comunicación serial al PC
I2C_HandleTypeDef hi2c1 = { 0 };   // I2C 1: Comunicación con la pantalla LCD
/* HANDLERS DEL ADC Y DMA */
ADC_HandleTypeDef hadc1 = { 0 };
DMA_HandleTypeDef hdma_adc1 = { 0 };
/* HANDLER DEL TIMER 1 */
TIM_HandleTypeDef htim1 = { 0 };   // Timer 1: Encoder 3 (Frecuencia)

/* PINES PARA LOS A4988 */
#define DIR_ANT_PIN   GPIO_PIN_0
#define STEP_ANT_PIN  GPIO_PIN_1
#define DIR_CAP_PIN   GPIO_PIN_2
#define STEP_CAP_PIN  GPIO_PIN_10
#define MOTOR_PORT    GPIOB
/* VARIABLES DEL SENSOR DE RF (Búfer DMA) */
#define ADC_BUFFER_SIZE 20

/* VARIABLES DE MÁQUINA DE ESTADOS Y CONTROL */
volatile uint8_t rx_data = 0;
volatile uint8_t flag_rx = 0;
volatile uint8_t rx_char = 0;
volatile uint8_t general_state = 0; // FSM: 0=UART, 1=LCD, 2=MOTORES, 3=REFRESCO
volatile uint8_t uart_state = 0;
volatile uint8_t flag_refresh_250ms = 0;

/* --- NUEVO: VARIABLES PARA BARRIDO AUTOMÁTICO DE ANTENA --- */
volatile uint8_t sweep_active = 0;
volatile uint8_t sweep_step = 0;

/* VARIABLES DE MOTORES Y ENCODERS */
int16_t target_ant = 0; // Posición deseada leída del encoder 1
int16_t target_cap = 0; // Posición deseada leída del encoder 2
int16_t pos_ant = 0;    // Posición física actual del motor 1
int16_t pos_cap = 0;    // Posición física actual del motor 2
int16_t step_size = 1;  // <--- NUEVA VARIABLE (Inicia en 1 por defecto)

char lcd_buffer[34];

uint16_t adc_buffer[ADC_BUFFER_SIZE]; // El DMA escribirá los datos aquí en segundo plano
uint32_t rf_level = 0; // Promedio calculado

/* VARIABLES DEL ENCODER DE FRECUENCIA */
int16_t last_freq_count = 0; // Para saber si el encoder se movió
uint32_t target_frequency = 7000; // Frecuencia inicial en kHz (Ej. 7.000 MHz)
volatile uint8_t flag_boton_sw = 0; // Bandera global (arriba en tu main.c)

// Cabecera de funciones
void SystemClock_Config(void);
void gpio_Init(void);
void tim4_blinky_Init(void);
void uart2_Init(void);
void I2C_Init(void);
void encoders_Init(void);
void steppers_Init(void);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_init(void);
void lcd_send_string(char *str);
void pulse_step(uint16_t pin);
void adc_dma_Init(void);
void autotune_calculate_position(void);


int main(void) {

	HAL_Init();
	SystemClock_Config();
	gpio_Init();          // Configura pin PH1 (LED)
	tim4_blinky_Init();   // TIM4 a 250ms
	uart2_Init();			//configuracion uart
	I2C_Init();				//configuracion i2c para la pantalla
	steppers_Init();      // Inicializa pines DIR y STEP para A4988
	encoders_Init();      // Inicializa TIM2 y TIM3 en modo Encoder
	adc_dma_Init();			// configuracion del dma para la toma de datos del rf
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx_data, 1);
	lcd_init();				//configuracion de la pnatallarr
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, ADC_BUFFER_SIZE);

	while (1) {
		// --- DETECCIÓN DEL BOTÓN SW PARA AUTO-CÁLCULO ---
		if (flag_boton_sw == 1) {
			flag_boton_sw = 0; // Limpiamos la bandera

			// Realizamos el cálculo y actualizamos el objetivo del motor
			autotune_calculate_position();
		}

		switch (general_state) {

		case 0: // ESTADO GENERAL 0: UART
			switch (uart_state) {

			case 0: // Sub-Estado 0: Menú Principal
				if (flag_rx == 1) {
					flag_rx = 0;

					// COMANDO '0': Fijar la posición actual como el nuevo CERO (Home)
					if (rx_char == '0') {
						__HAL_TIM_SET_COUNTER(&htim2, 0);
						__HAL_TIM_SET_COUNTER(&htim3, 0);
						pos_ant = 0; // Le decimos al sistema que físicamente ya está en 0
						pos_cap = 0;
						char msg[] = "Cero fijado en la posicion actual\r\n";
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
								100);
					}
					// COMANDO 'R' o 'r': Regresar los motores a la posición CERO guardada
					else if (rx_char == 'R' || rx_char == 'r') {
						__HAL_TIM_SET_COUNTER(&htim2, 0);
						__HAL_TIM_SET_COUNTER(&htim3, 0);
						// OJO: No alteramos pos_ant ni pos_cap.
						// Así el Estado 2 se encargará de moverlos de regreso hasta 0.
						char msg[] = "Regresando a la posicion 0...\r\n";
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
								100);
					}
					// COMANDO 'A' o 'a': Control Antena
					else if (rx_char == 'A' || rx_char == 'a') {
						uart_state = 1;
						char msg[] =
								"Control ANT: Envia '+' o '-'. Envia 'S' para salir.\r\n";
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
								100);
					}
					// COMANDO 'C' o 'c': Control Capacitancia
					else if (rx_char == 'C' || rx_char == 'c') {
						uart_state = 2;
						char msg[] =
								"Control CAP: Envia '+' o '-'. Envia 'S' para salir.\r\n";
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
								100);
					}
				}
				break;

			case 1: // Sub-Estado 1: Mover Antena (ANT)
			    if (flag_rx == 1) {
			        flag_rx = 0;

			        if (rx_char == '+') {
			            int16_t temp_ant = (int16_t) __HAL_TIM_GET_COUNTER(&htim2);
			            __HAL_TIM_SET_COUNTER(&htim2, temp_ant + step_size * 4);
			        } else if (rx_char == '-') {
			            int16_t temp_ant = (int16_t) __HAL_TIM_GET_COUNTER(&htim2);
			            __HAL_TIM_SET_COUNTER(&htim2, temp_ant - step_size * 4);
			        }
			        // Cambiar a paso RÁPIDO (16 pasos)
			        else if (rx_char == 'H' || rx_char == 'h') {
			            step_size = 16;
			            char msg[] = "Modo RAPIDO: Saltos de 16 pasos\r\n";
			            HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
			        }
			        // Cambiar a paso LENTO (1 paso)
			        else if (rx_char == 'L' || rx_char == 'l') {
			            step_size = 1;
			            char msg[] = "Modo LENTO: Saltos de 1 paso\r\n";
			            HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
			        } else if (rx_char == 'S' || rx_char == 's') {
			            uart_state = 0;
			            char msg[] = "Regreso al Menu Principal\r\n";
			            HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
			        }
			    }
			    break;

			case 2: // Sub-Estado 2: Mover Capacitancia (CAP)
				if (flag_rx == 1) {
					flag_rx = 0;

					if (rx_char == '+') {
						int16_t temp_cap = (int16_t) __HAL_TIM_GET_COUNTER(
								&htim3);
						__HAL_TIM_SET_COUNTER(&htim3, temp_cap + step_size * 4);
					} else if (rx_char == '-') {
						int16_t temp_cap = (int16_t) __HAL_TIM_GET_COUNTER(
								&htim3);
						__HAL_TIM_SET_COUNTER(&htim3, temp_cap - step_size * 4);
					}
					// Cambiar a paso RÁPIDO (16 pasos)
					else if (rx_char == 'H' || rx_char == 'h') {
						step_size = 16;
						char msg[] = "Modo RAPIDO: Saltos de 16 pasos\r\n";
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
								100);
					}
					// Cambiar a paso LENTO (1 paso)
					else if (rx_char == 'L' || rx_char == 'l') {
						step_size = 1;
						char msg[] = "Modo LENTO: Saltos de 1 paso\r\n";
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
								100);
					} else if (rx_char == 'S' || rx_char == 's') {
						uart_state = 0;
						char msg[] = "Regreso al Menu Principal\r\n";
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
								100);
					}
				}
				break;
			}

			general_state = 1; // Cede el turno al estado 1 (LCD)
			break;

		case 1: // ESTADO 1: ACTUALIZACIÓN LCD
				// Fila 1: Mostrar FREQ (Dirección 0x80)
			lcd_send_cmd(0x80);
			// "FREQ: " (6 chars) + 10 dígitos + " kHz" (4 chars) = 20 caracteres
			sprintf(lcd_buffer, "FREQ: %-10lu kHz", target_frequency);
			lcd_send_string(lcd_buffer);

			// Fila 2: Mostrar ANT (Dirección 0xC0)
			lcd_send_cmd(0xC0);
			// "ANT : " (6 chars) + 14 espacios = 20 caracteres
			sprintf(lcd_buffer, "ANT : %-14d", pos_ant);
			lcd_send_string(lcd_buffer);

			// Fila 3: Mostrar CAP (Dirección 0x94)
			lcd_send_cmd(0x94);
			// "CAP : " (6 chars) + 14 espacios = 20 caracteres
			sprintf(lcd_buffer, "CAP : %-14d", pos_cap);
			lcd_send_string(lcd_buffer);

			// Fila 4: Mostrar RF (Dirección 0xD4)
			lcd_send_cmd(0xD4);
			// "RF  : " (6 chars) + 14 espacios = 20 caracteres
			sprintf(lcd_buffer, "RF  : %-14lu", rf_level);
			lcd_send_string(lcd_buffer);

			general_state = 2;
			break;
		case 2: // ESTADO 2: LECTURA ENCODERS Y CONTROL DE MOTORES
			// --- LECTURA DEL ENCODER DE FRECUENCIA ---
			int16_t current_freq_count = (int16_t) __HAL_TIM_GET_COUNTER(&htim1)
					/ 4;

			if (current_freq_count != last_freq_count) {
				// Calculamos la diferencia (si giró a la derecha o a la izquierda)
				int16_t diff = current_freq_count - last_freq_count;

				// Aumentamos o disminuimos la frecuencia (ej. de 10 en 10 kHz)
				target_frequency += (diff * 10);

				// Guardamos el nuevo estado
				last_freq_count = current_freq_count;
			}

			// Límites de seguridad para la frecuencia (ej. 3.5 MHz a 30 MHz)
			if (target_frequency < 3500)
				target_frequency = 3500;
			if (target_frequency > 30000)
				target_frequency = 30000;

			// 1. Leer valores de los Timers
			target_ant = (int16_t) __HAL_TIM_GET_COUNTER(&htim2) / 4;
			target_cap = (int16_t) __HAL_TIM_GET_COUNTER(&htim3) / 4;

			// --- NUEVO: LÓGICA DE BARRIDO AUTOMÁTICO DE ANTENA ---
			if (sweep_active) {
				switch (sweep_step) {
					case 0: // Espera a que el capacitor termine de llegar a su nueva posición
						if (pos_cap == target_cap) {
							sweep_step = 1;
							target_ant = 0; // Vamos al punto de partida
							__HAL_TIM_SET_COUNTER(&htim2, target_ant * 4); // Sincroniza encoder

							char msg[] = "CAP posicionado. Llevando ANT a 0...\r\n";
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
						}
						break;
					case 1: // Espera a llegar a 0 para bajar a -100
						if (pos_ant == 0) {
							sweep_step = 2;
							target_ant = -100;
							__HAL_TIM_SET_COUNTER(&htim2, (uint16_t)(-100 * 4));

							char msg[] = "ANT en 0. Iniciando barrido hacia -100...\r\n";
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
						}
						break;
					case 2: // Espera a llegar a -100 para subir a 100
						if (pos_ant == -100) {
							sweep_step = 3;
							target_ant = 100;
							__HAL_TIM_SET_COUNTER(&htim2, 100 * 4);

							char msg[] = "ANT en -100. Barriendo hasta 100...\r\n";
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
						}
						break;
					case 3: // Espera a llegar a 100 para finalizar
						if (pos_ant == 100) {
							sweep_active = 0; // Barrido Terminado

							char msg[] = "Barrido ANT completado.\r\n";
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
						}
						break;
				}
			}

			// 2. Limitar Encoder ANT entre -100 y 100
			if (target_ant > 100) {
				target_ant = 100;
				__HAL_TIM_SET_COUNTER(&htim2, 100 * 4);
			} else if (target_ant < -100) {
				target_ant = -100;
				__HAL_TIM_SET_COUNTER(&htim2, (uint16_t) -100 * 4);
			}

			// 3. Limitar Encoder CAP entre 0 y 1600 pasos
			if (target_cap > 1600) {
				target_cap = 1600;
				// Le decimos al hardware del Timer que se fije al máximo permitido (1600 * 4)
				__HAL_TIM_SET_COUNTER(&htim3, 1600 * 4);
			} else if (target_cap < 0) {
				target_cap = 0;
				// Si intentas girar a la izquierda de 0, forzamos el Timer a quedarse en 0
				__HAL_TIM_SET_COUNTER(&htim3, 0);
			}

			// 4. Mover Motor ANT
			if (pos_ant < target_ant) {
				HAL_GPIO_WritePin(MOTOR_PORT, DIR_ANT_PIN, GPIO_PIN_SET); // Derecha
				pulse_step(STEP_ANT_PIN);
				pos_ant++;
			} else if (pos_ant > target_ant) {
				HAL_GPIO_WritePin(MOTOR_PORT, DIR_ANT_PIN, GPIO_PIN_RESET); // Izquierda
				pulse_step(STEP_ANT_PIN);
				pos_ant--;
			}

			// 5. Mover Motor CAP (Lógica de dirección invertida)
			if (pos_cap < target_cap) {
			    HAL_GPIO_WritePin(MOTOR_PORT, DIR_CAP_PIN, GPIO_PIN_RESET); // Cambiado de SET a RESET
			    pulse_step(STEP_CAP_PIN);
			    pos_cap++;
			} else if (pos_cap > target_cap) {
			    HAL_GPIO_WritePin(MOTOR_PORT, DIR_CAP_PIN, GPIO_PIN_SET);   // Cambiado de RESET a SET
			    pulse_step(STEP_CAP_PIN);
			    pos_cap--;
			}

			general_state = 3;
			break;

		case 3: // ESTADO 3: REFRESCO SERIAL Y LECTURA
			if (flag_refresh_250ms == 1) {
				// 1. Sumar los 20 valores frescos del arreglo DMA
				uint32_t sum = 0;
				for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
					sum += adc_buffer[i];
				}
				// 2. Obtener el promedio
				rf_level = sum / ADC_BUFFER_SIZE;

				// 3. Imprimir por consola
				char serial_msg[80];
				sprintf(serial_msg,
						"FREQ: %lu kHz | ANT: %d | CAP: %d | RF: %lu \r\n",
						target_frequency, pos_ant, pos_cap, rf_level);
				HAL_UART_Transmit(&huart2, (uint8_t*) serial_msg,
						strlen(serial_msg), 100);

				flag_refresh_250ms = 0; // <--- SE BAJA LA BANDERA AL FINAL DEL TODO
			}
			general_state = 0;
			break;
		}
	}
}

/* ==================================================================== */
/*                       FUNCIONES DE HARDWARE                          */
/* ==================================================================== */

// Función para generar un pulso de >1us requerido por el A4988
void pulse_step(uint16_t pin) {
	HAL_GPIO_WritePin(MOTOR_PORT, pin, GPIO_PIN_SET);
	for (volatile int i = 0; i < 200; i++)
		__NOP(); // Tiempo en ALTO (microscópico)
	HAL_GPIO_WritePin(MOTOR_PORT, pin, GPIO_PIN_RESET);

	HAL_Delay(1); // <--- Tiempo en BAJO. Le da fuerza al motor y regula la velocidad (1000 pasos/segundo)
}

void steppers_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// Configura pines DIR y STEP como salidas
	GPIO_InitStruct.Pin = DIR_ANT_PIN | STEP_ANT_PIN | DIR_CAP_PIN
			| STEP_CAP_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(MOTOR_PORT, &GPIO_InitStruct);
}

void encoders_Init(void) {
	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	__HAL_RCC_TIM1_CLK_ENABLE(); // <--- Reloj para Timer 1 (Frecuencia)
	__HAL_RCC_TIM2_CLK_ENABLE(); // Reloj para Timer 2 (ANT)
	__HAL_RCC_TIM3_CLK_ENABLE(); // Reloj para Timer 3 (CAP)
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	// 1. Configuración Pines TIM1 (PA8, PA9) - Encoder FRECUENCIA
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1; // Alternate function del TIM1
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// 2. Configuración Pines TIM2 (PA0, PA1) - Encoder ANT
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// 3. Configuración Pines TIM3 (PA6, PA7) - Encoder CAP
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// --- CONFIGURACIÓN BASE DE LOS TIMERS ENCODER ---
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;

	// Iniciar TIM1 (Frecuencia)
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0xFFFF;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Encoder_Init(&htim1, &sConfig);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	// Iniciar TIM2 (Antena)
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 0xFFFF;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Encoder_Init(&htim2, &sConfig);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	// Iniciar TIM3 (Capacitor)
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 0xFFFF;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Encoder_Init(&htim3, &sConfig);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 400;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
}

void gpio_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	// --- NUEVO: Configuración del botón SW (PA10) desde el 74HC14 ---
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Detecta la subida de voltaje
	GPIO_InitStruct.Pull = GPIO_NOPULL;         // No necesita pull interno
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Habilitar la interrupción en el NVIC para los pines 10 al 15
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	__NOP();

}

void tim4_blinky_Init(void) {
	__HAL_RCC_TIM4_CLK_ENABLE();
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 9999;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 2499;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim4);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	HAL_TIM_Base_Start_IT(&htim4);
	__NOP();
}

void uart2_Init(void) {
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 19200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart2);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void I2C_Init(void) {
	__HAL_RCC_I2C1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
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

void lcd_send_cmd(char cmd) {
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd & 0xF0);
	data_l = ((cmd << 4) & 0xF0);
	data_t[0] = data_u | 0x0C;
	data_t[1] = data_u | 0x08;
	data_t[2] = data_l | 0x0C;
	data_t[3] = data_l | 0x08;
	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_t, 4, 100);
}

void lcd_send_data(char data) {
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data & 0xF0);
	data_l = ((data << 4) & 0xF0);
	data_t[0] = data_u | 0x0D;
	data_t[1] = data_u | 0x09;
	data_t[2] = data_l | 0x0D;
	data_t[3] = data_l | 0x09;
	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_t, 4, 100);
}

void lcd_init(void) {
	HAL_Delay(50);
	lcd_send_cmd(0x30);
	HAL_Delay(5);
	lcd_send_cmd(0x30);
	HAL_Delay(1);
	lcd_send_cmd(0x30);
	HAL_Delay(10);
	lcd_send_cmd(0x20);
	HAL_Delay(10);
	lcd_send_cmd(0x28);
	HAL_Delay(1);
	lcd_send_cmd(0x08);
	HAL_Delay(1);
	lcd_send_cmd(0x01);
	HAL_Delay(2);
	lcd_send_cmd(0x06);
	HAL_Delay(1);
	lcd_send_cmd(0x0C);
	HAL_Delay(1);
}

void lcd_send_string(char *str) {
	while (*str)
		lcd_send_data(*str++);
}

void adc_dma_Init(void) {
	// 1. Habilitar Relojes
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE(); // El ADC1 pertenece al DMA2 en el STM32F4

	// 2. Configurar Pin PA4 como entrada analógica
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// 3. Configurar DMA2 Stream 0 (Canal 0) para el ADC1
	hdma_adc1.Instance = DMA2_Stream0;
	hdma_adc1.Init.Channel = DMA_CHANNEL_0;
	hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY; // Del ADC a la RAM
	hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE; // No incrementar dirección del periférico
	hdma_adc1.Init.MemInc = DMA_MINC_ENABLE; // Sí incrementar dirección de la RAM
	hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // Datos de 16 bits
	hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_adc1.Init.Mode = DMA_CIRCULAR; // Modo infinito (sobrescribe datos viejos)
	hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
	hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&hdma_adc1);

	// 4. Vincular el Handler del DMA al Handler del ADC
	__HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

	// 5. Configurar el ADC1 en Modo Continuo con peticiones DMA
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B; // 12 bits (0 a 4095)
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;     // ¡MODO CONTINUO ACTIVADO!
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE; // ¡PETICIONES DMA CONTINUAS ACTIVADAS!
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	HAL_ADC_Init(&hadc1);

	// 6. Configurar el Canal 4 (Pin PA4)
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES; // Tiempo largo para evitar ruido
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}
void autotune_calculate_position(void) {
	// 1. Convertir la frecuencia actual (en kHz) a float
	float f_khz = (float) target_frequency;

	// 2. Calcular la Capacidad teórica requerida en pF
	// Constante K = 10^6 / (4 * PI^2 * L_uH) = 6178120953.8
	float c_pf = 6178120953.8f / (f_khz * f_khz);

	// 3. Mapeo lineal: Paso = (C_pF - 30 pF) * (2000 pasos / 50 pF)
	float calculated_steps = (c_pf - 30.0f) * 40.0f;

	// 4. Redondear al paso entero más cercano
	int16_t target = (int16_t) roundf(calculated_steps);

	// 5. Límites de seguridad física para el motor del capacitor (0 a 2000)
	if (target < 0) {
		target = 0;  // Frecuencia demasiado alta para el capacitor (>14.35 MHz)
	} else if (target > 2000) {
		target = 2000; // Frecuencia demasiado baja para el capacitor (<8.79 MHz)
	}

	// 6. Asignar el nuevo valor objetivo
	target_cap = target;

	// 7. Sincronizar el hardware del Timer 3 (Encoder de Capacidad) con la nueva posición
	__HAL_TIM_SET_COUNTER(&htim3, target_cap * 4);

	// 8. Notificar por el puerto serie
	char msg[100];
	sprintf(msg,
			"AUTO-TUNE: Freq=%lu kHz | C_calc=%.2fpF | Target CAP=%d pasos\r\n",
			target_frequency, c_pf, target_cap);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);

	// --- NUEVO: ACTIVAMOS EL BARRIDO AUTOMÁTICO DE LA ANTENA ---
	sweep_active = 1;
	sweep_step = 0; // Paso 0: Esperar a que el capacitor se asiente
}

//Callbacks
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) {
		HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		flag_refresh_250ms = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		rx_char = rx_data;
		flag_rx = 1;
		HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx_data, 1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_10) {
		flag_boton_sw = 1; // Se activará de forma limpia y perfecta al presionar
	}
}
