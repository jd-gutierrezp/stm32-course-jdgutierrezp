/*
 * main.c
 *
 * Author: jdgutierrezp
*/
/*DISTRIBUCIÓN Y RELACIÓN DEL CÓDIGO
 *
 * 1. #INCLUDES: Incorpora la librería HAL de STM32F4 y las herramientas estándar
 * (stdio, string) dedicadas exclusivamente a la construcción de los mensajes
 * de telemetría y comandos.
 *
 * 2. DEFINICIÓN DE VARIABLES: Declara los objetos de control de hardware
 * (htim2, htim3, hadc1, etc.) y las variables del tipo "volátile" (banderas y contadores).
 * Estas últimas actúan como el único puente de comunicación entre las rutinas de
 * interrupción y la máquina de estados principal.
 *
 * 3. CABECERA DE FUNCIONES: Lista los prototipos de las rutinas de inicialización
 * de relojes, pines y periféricos específicos del proyecto.
 *
 * 4. FUNCIÓN MAIN: Arranca la HAL, configura el reloj del sistema a 16 MHz y
 * ejecuta una a una las funciones de inicialización. Finalmente, activa las
 * interrupciones base (ADC_Start_IT, UART_Receive_IT) y establece los valores
 * de seguridad antes de ceder el control al while.
 *
 * 5. BUCLE WHILE (1): Ejecuta continuamente la Máquina de Estados Finitos (FSM)
 * de cuatro pasos (Encoder, UART, Telemetría, ADC). Su labor exclusiva es
 * consumir las banderas levantadas por los Callbacks, calcular matemáticamente
 * los ciclos de trabajo (PWM) para el Timer 3 y despachar la transmisión serial.
 *
 * 6. FUNCIONES : Contiene el detalle técnico de los periféricos a configurar.
 * Define prescalers, periodos (ej. el tope de 3999 para 1 kHz),
 * rutas de reloj y funciones alternativas de los pines.
 *
 * 7. CALLBACKS (INTERRUPCIONES): Captura los eventos de hardware asíncronos
 * (el desborde del TIM4 a los 250ms, el fin de conversión del ADC1 y la llegada
 * de un byte al USART2). Su única responsabilidad es respaldar el dato crudo,
 * levantar la bandera correspondiente (ej. flag_rx = 1) y rearmar la interrupción.
 *
 * RELACIÓN Y FLUJO DE DATOS:
 * Las FUNCIONES preparan el hardware llamado desde el MAIN. Una vez dentro del
 * WHILE, el sistema gira continuamente revisando los estados. Cuando ocurre un
 * evento físico, los CALLBACKS pausan el microcontrolador microsegundos, alteran
 * las VARIABLES globales (banderas) y regresan el control. El WHILE, en su turno
 * respectivo, detecta la bandera, procesa la los eventos (como compensar la
 * zona muerta del ADC o formatear textos) y actualiza los registros físicos,
 * garantizando que ninguna tarea bloquee a las demás.
*/

/* ARQUITECTURA FSM: MÁQUINA DE ESTADOS FINITOS (FSM) GENERAL
 * * Se implementa una arquitectura de maquina de estado finitos, basada en un ciclo
 * continuo de 4 pasos (general_state).
 *
 ** Flujo de ejecución:
 * -> ESTADO 0 (Encoder): Verifica si la perilla física giró. Si detecta un
 * cambio, actualiza el porcentaje (de 0 a 100%), lo mapea matemáticamente al
 * tope del temporizador (multiplicando el porcentaje por 3999 y dividiendo
 * entre 100) para ajustar la señal PWM, y cede el turno al estado 1.
 *
 * -> ESTADO 1 (UART): Inspecciona si llegó un comando de texto desde la PC.
 * Aquí opera una "Sub-Máquina" (uart_state) que evalúa si la letra es válida
 * (ya sea sumando/restando un 10% (A ó D)   o fijando preajustes de 25% y 75%(F u O)),
 * transforma ese porcentaje al rango del Timer (0-3999) y modifica el PWM.
 * Al finalizar, cede el turno al estado 2.
 *
 * -> ESTADO 2 (ADC): Verifica si el sensor analógico tiene una nueva lectura
 * lista. Si hay datos, recalcula el PWM implementando una zona muerta de
 * protección: si la lectura cruda es menor o igual a 220, apaga el PWM
 * por completo (valor 0); si es mayor, resta esos 220 y escala el rango
 * analógico restante hacia el tope de 3999. cede el turno al estado 3
 *
 * -> ESTADO 3 (Telemetría): Observa si el Timer 4 levantó la bandera de los
 * 250ms. Solo si el tiempo se cumplió, empaqueta los valores del ADC,
 * Encoder y Serial, y los envía como un reporte a la computadora.
 * Finalmente, reinicia la FSM
 * ============================================================================== */

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


/*DEFINICION DE VARIABLES (PERIFÉRICOS)*/

TIM_HandleTypeDef htim2 = { 0 }; 	// Administra el Timer 2 (Genera el muestreo de 20ms)
TIM_HandleTypeDef htim3 = { 0 }; 	// Administra el Timer 3 (Genera el PWM a 1 kHz para ADC, Encoder y UART)
ADC_HandleTypeDef hadc1 = { 0 }; 	// Administra el periférico ADC (Lectura analógica)
TIM_HandleTypeDef htim4 = { 0 }; 	// Administra el Timer 4 (Genera la interrupción del Blinky cada 250ms)
TIM_HandleTypeDef htim5 = { 0 }; 	// Administra el Timer 5 (Lee los pulsos del Encoder)
UART_HandleTypeDef huart2 = { 0 }; 	// Administra la comunicación serial USART2


/* VARIABLES DE USUARIO*/

/* Uso ADC */
volatile uint16_t raw_adc = 0;			// Almacena el valor crudo leído por el ADC (0-4095)
volatile uint8_t flag_adc_event = 0;	// Indica si hay una nueva lectura del ADC lista para procesar
volatile uint32_t adc_pwm_width = 0;    // Almacena el valor calculado del ciclo de trabajo (PWM) para el ADC
volatile uint8_t adc_percent = 0;		//variable para el calculo del porcentaje del adc


/* Uso UART */
volatile uint8_t rx_data = 0;       	// Actúa como buffer temporal para recibir 1 solo byte por hardware
volatile uint8_t flag_rx = 0;       	// Indica a la FSM que llegó un nuevo carácter válido
volatile uint8_t rx_char = 0;			// Guarda el carácter recibido de forma segura para procesarlo
volatile uint32_t serial_pwm_width = 0; // Almacena el valor del ciclo de trabajo (PWM) calculado por comandos seriales
volatile int32_t serial_pwm_percent = 0;// Almacena el porcentaje del PWM (0-100) ingresado por puerto serial

/* Uso Encoder */
volatile uint32_t posicion_anterior = 0;// Guarda la lectura previa del encoder para comparar cambios
volatile uint32_t posicion_actual = 0;	// Guarda la lectura actual del encoder
volatile int32_t enc_pwm_width_percent = 0; // Almacena el porcentaje de PWM (0-100) controlado por el encoder
volatile uint32_t encoder_pwm_width = 0;// Almacena el valor del ciclo de trabajo (PWM) calculado por el encoder

/* FSM (Máquinas de Estados) */
volatile uint8_t general_state = 0; 	// Controla la FSM Principal: 0=ENCODER, 1=UART, 2=ADC, 3=REFRESCO SERIAL
volatile uint8_t uart_state = 0; 		// Controla la Sub-FSM UART: 0=IDLE, 1=PROCESAR, 2=MODIFICAR_PWM, 3=ACCION_EXTRA
volatile uint8_t flag_refresh_250ms = 0;// Indica que pasaron 250ms y se debe enviar el estado general por serial


/*CABECERA DE FUNCIONES (PROTOTIPOS)*/

static void SystemClock_Config(void);   // Configura la señal de reloj principal del sistema a 16 MHz
static void gpio_Init(void);			// Configura el pin PH1 como salida para el LED Blinky
static void tim3_pwm_Init(void);		// Configura el Timer 3 para generar las señales PWM
static void tim2_Init(void);			// Configura el Timer 2 para llevar la base de tiempo de refresco de 20 ms
static void tim4_blinky_Init(void);		// Configura el Timer 4 para interrumpir cada 250 ms
static void adc_Init(void);				// Configura el hardware del ADC
static void tim5_encoder_Init(void);	// Configura el Timer 5 en modo lectura del Encoder
static void uart2_Init(void);			// Configura los pines y parámetros de la comunicación serial
void MCO1_GPIO_Init(void);				// Configura el pin PA8 para sacar la señal de reloj del sistema (MCO1)


/*FUNCION PRINCIPAL (MAIN)*/

int main(void) {
	HAL_Init();                         // Inicializa la librería HAL y los servicios básicos del microcontrolador
	SystemClock_Config();               // Ajusta el reloj interno a la velocidad deseada

    // Inicialización de Periféricos
    gpio_Init();						// Inicializa el pin del LED (Blinky)
	tim2_Init();  						// Arranca la base de tiempo general
	tim3_pwm_Init(); 					// Arranca el PWM a 1Khz para los canales del ADC, Encoder y USART
	adc_Init(); 						// Prepara el ADC en el pin PC6 para ser disparado por el TIM2
	tim4_blinky_Init(); 				// Arranca el temporizador del LED Blinky a 250 ms
	tim5_encoder_Init(); 				// Arranca la lectura de los pines PA0 y PA1 para el encoder
	uart2_Init(); 						// Abre el puerto serial para transmisión y recepción
	MCO1_GPIO_Init();                   // Habilita la salida del reloj de prueba

	HAL_ADC_Start_IT(&hadc1);           // Arranca el periférico ADC y habilita sus interrupciones

	/* ACTIVAR RECEPCIÓN POR INTERRUPCIÓN */
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx_data, 1); // Pone al hardware a escuchar el primer carácter serial

	/* MENSAJE INICIAL POR POLLING */
	char initial_msg[] ="Sistema Iniciado. Comandos: A (Suma), D (Resta), O (Led 75), F (Led 25)\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*) initial_msg, strlen(initial_msg), HAL_MAX_DELAY); // Envía el texto de bienvenida a la PC

    // Valores iniciales de seguridad
	posicion_anterior = __HAL_TIM_GET_COUNTER(&htim5) / 4; // Captura la posición de arranque del encoder (dividida por resolución)
	enc_pwm_width_percent = 0;          // Inicializa el porcentaje en cero
	encoder_pwm_width = 0;              // Inicializa el ancho de pulso en cero


	while (1) {
		switch (general_state) {        // Evalúa en qué estado de la máquina principal se encuentra

		case 0: 		// ESTADO GENERAL 0: ACTUALIZAR ENCODER
		{
			posicion_actual = __HAL_TIM_GET_COUNTER(&htim5) / 4; // Lee la posición del momento

			if (posicion_actual != posicion_anterior) {          // Verifica si la perilla se movió
				int16_t diferencia = (int16_t) (posicion_actual - posicion_anterior); // Calcula cuánto y hacia dónde giró

				if (diferencia > 0) {                            // Si giró a la derecha (incremento)
					enc_pwm_width_percent += 1;                  // Aumenta el porcentaje de PWM
					if (enc_pwm_width_percent > 100)             // Limita el máximo a 100%
						enc_pwm_width_percent = 100;
				} else if (diferencia < 0) {                     // Si giró a la izquierda (decremento)
					enc_pwm_width_percent -= 1;                  // Disminuye el porcentaje de PWM
					if (enc_pwm_width_percent < 0)               // Limita el mínimo a 0%
						enc_pwm_width_percent = 0;
				}

				encoder_pwm_width = (enc_pwm_width_percent * 3999) / 100; // Mapea el porcentaje al tope del timer (3999)
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, encoder_pwm_width); // Actualiza la señal física del PWM
				posicion_anterior = posicion_actual;             // Actualiza la referencia para la próxima lectura
			}

			general_state = 1; // Cede el turno de procesamiento al bloque UART
			break;
		}

		case 1:         // ESTADO GENERAL 1: ACTUALIZAR UART (SUB-FSM)
		{
			switch (uart_state) { // Evalúa en qué sub-estado de recepción se encuentra

			case 0: // Sub-Estado 0: IDLE (Reposo)
				if (flag_rx == 1) { // Verifica si llegó una nueva letra desde el PC
					flag_rx = 0;    // Apaga la bandera de aviso
					uart_state = 1; // Pasa al estado de procesar la letra
				}
				break;

			case 1: // Sub-Estado 1: PROCESAR
				switch (rx_char) {  // Analiza qué letra exacta llegó
				case 'A': case 'a':
				case 'D': case 'd':
					uart_state = 2; // Si es comando de ajuste manual, va al estado de modificar PWM
					break;

				case 'O': case 'o':
				case 'F': case 'f':
					uart_state = 3; // Si es un comando prestablecido, va al estado de acción extra
					break;

				default: {          // Si el usuario presiona una tecla no registrada
					char serial_invalid_msg[] = "Comando invalido\r\n";
					HAL_UART_Transmit(&huart2, (uint8_t*) serial_invalid_msg, strlen(serial_invalid_msg), HAL_MAX_DELAY); // Envía mensaje de error
					uart_state = 0; // Regresa al reposo a esperar una nueva tecla
					break;
				}
				}
				break;

			case 2: // Sub-Estado 2: MODIFICAR_PWM
			{
				switch (rx_char) {
				case 'A': case 'a':
					serial_pwm_percent += 10;           // Sube un 10% el duty cycle
					if (serial_pwm_percent > 100)       // Evita que pase del 100%
						serial_pwm_percent = 100;
					break;
				case 'D': case 'd':
					serial_pwm_percent -= 10;           // Baja un 10% el duty cycle
					if (serial_pwm_percent < 0)         // Evita valores negativos
						serial_pwm_percent = 0;
					break;
				}

				serial_pwm_width = (serial_pwm_percent * 3999) / 100; // Mapea matemáticamente el porcentaje al Timer
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, serial_pwm_width); // Aplica el cambio en el pin físico

				char serial_msg[50];
				sprintf(serial_msg, "PWM Serial al %ld%%\r\n", serial_pwm_percent); // Construye un texto de confirmación
				HAL_UART_Transmit(&huart2, (uint8_t*) serial_msg, strlen(serial_msg), HAL_MAX_DELAY); // Envía la confirmación al PC

				uart_state = 0; // Regresa al reposo
				break;
			}

			case 3: // Sub-Estado 3: ACCION_EXTRA
			{
				switch (rx_char) {
				case 'O': case 'o': {
					serial_pwm_percent = 75;            // Fija el valor directamente al 75%
					char serial_75_msg[] = "PWM Serial: PRESET 75%\r\n";
					HAL_UART_Transmit(&huart2, (uint8_t*) serial_75_msg, strlen(serial_75_msg), HAL_MAX_DELAY); // Informa el cambio
					break;
				}
				case 'F': case 'f': {
					serial_pwm_percent = 25;            // Fija el valor directamente al 25%
					char serial_25_msg[] = "PWM Serial: PRESET 25%\r\n";
					HAL_UART_Transmit(&huart2, (uint8_t*) serial_25_msg, strlen(serial_25_msg), HAL_MAX_DELAY); // Informa el cambio
					break;
				}
				}

				serial_pwm_width = (serial_pwm_percent * 3999) / 100; // Calcula el valor en pulsos
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, serial_pwm_width); // Lo aplica al periférico de hardware

				uart_state = 0; // Regresa al reposo
				break;
			}
			}

			general_state = 2; //Cede el turno al estado encargado de leer el ADC
			break;
		}
		   case 2:         // ESTADO GENERAL 3: ACTUALIZAR ADC
				{
					if (flag_adc_event == 1)                    // Verifica si el ADC terminó una nueva conversión
					{
						flag_adc_event = 0;                     // Baja la bandera de aviso

						adc_pwm_width = (raw_adc * 3999) / 4095;// Mapeo inicial estándar
						adc_percent = 0.0258*(raw_adc-220);		//se hace el calculo para el porcentaje del adc

						if (raw_adc <= 220) {                   // Si el voltaje es muy bajo (Zona muerta inferior)
							adc_pwm_width = 0;                  // Apaga el PWM por completo
							adc_percent = 0;					//l a condicion si el valor es menor 220 y evitar valors negativos del calculo
						}
						else {                                  // Si supera el umbral
							adc_pwm_width = ((raw_adc - 220) * 3999 / 3875); // Realiza el cálculo compensando la zona muerta
						}

						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, adc_pwm_width); // Fija el nuevo brillo en el pin correspondiente
					}
					general_state = 3;// Cede el turno de procesamiento al bloque del refresco de los datos del serial (monitor serial)
					break;
				}

		case 3:         // ESTADO GENERAL 2: ACTUALIZAR PANTALLA
		{
			if (flag_refresh_250ms == 1) {              // Comprueba si el Timer 4 levantó la bandera de los 250ms
				flag_refresh_250ms = 0;                 // Baja la bandera inmediatamente para evitar reingresos

				char serial_refresh_msg[100];           // Reserva memoria para el mensaje final
				sprintf(serial_refresh_msg,"ADC (B): %u%% | Encoder (G): %ld%% | PWM Serial (R): %ld%%\r\n", adc_percent,
						enc_pwm_width_percent, serial_pwm_percent); // Fusiona los valores de las variables en una sola frase de texto

				HAL_UART_Transmit(&huart2, (uint8_t*) serial_refresh_msg, strlen(serial_refresh_msg), HAL_MAX_DELAY); // Envía el reporte a la terminal
			}

			general_state = 0; // Reinicia la  FSM volviendo el turno al Encoder
			break;
		}

		}
	}
}


/*CONFIGURACION DE RELOJ*/					// SE DEJA CONFIGURADO POR DEFECTO COMO SE HIZO EN CLASE

static void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };       // Crea estructura para el oscilador
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };       // Crea estructura para la distribución de reloj

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // Selecciona el oscilador interno (HSI)
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;            // Enciende el oscilador HSI
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Usa la calibración de fábrica
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;      // Desactiva el multiplicador (PLL)
	HAL_RCC_OscConfig(&RCC_OscInitStruct);              // Aplica los parámetros al hardware

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2; // Selecciona todos los buses
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; // Pasa el oscilador interno al reloj principal del sistema
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  // Ajusta divisor principal (HCLK) a 16 MHz
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;   // Ajusta bus periférico 1 a 16 MHz
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   // Ajusta bus periférico 2 a 16 MHz

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0); // Carga la configuración con 0 estados de espera para la Flash
}


/*CONFIGURACION DE PINES (GPIO)*/

static void gpio_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 }; 	        // Crea estructura para administrar los pines

	__HAL_RCC_GPIOH_CLK_ENABLE();				        // Enciende la señal de reloj para el puerto H

	GPIO_InitStruct.Pin = GPIO_PIN_1;			        // Selecciona el Pin 1
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;	        // Configura el pin como Salida Digital (Push-Pull)
	GPIO_InitStruct.Pull = GPIO_NOPULL;			        // Desactiva resistencias internas (Ni Pull-up, Ni Pull-down)
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;        // Ajusta la velocidad de conmutación en "Baja"
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);		        // Carga y aplica la configuración a los registros físicos FSR
	__NOP();
}


/*CONFIGURACION DEL ADC*/

static void adc_Init(void) {
	GPIO_InitTypeDef GPIO_adc_ch15 = { 0 };		        // Crea estructura de configuración para el pin analógico

	__HAL_RCC_GPIOC_CLK_ENABLE(); 				        // Activa la señal de reloj del Puerto C

	GPIO_adc_ch15.Pin = GPIO_PIN_5;				        // Selecciona el pin 5
	GPIO_adc_ch15.Mode = GPIO_MODE_ANALOG;		        // Pone el pin en modo puramente analógico
	GPIO_adc_ch15.Pull = GPIO_NOPULL;			        // Deshabilita resistencias internas

	HAL_GPIO_Init(GPIOC, &GPIO_adc_ch15);		        // Escribe la configuración en el hardware
	__NOP();

	__HAL_RCC_ADC1_CLK_ENABLE(); 				        // Enciende el reloj del periférico ADC1

	hadc1.Instance = ADC1;                              // Apunta al módulo de Hardware ADC1
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2; // Divide la frecuencia para la lectura
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;         // Ajusta la lectura a 12 bits (0-4095)
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;         // Alinea el resultado a la derecha en la memoria
	hadc1.Init.ScanConvMode = DISABLE;                  // Apaga el escaneo de múltiples canales
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;      // Marca el fin de ciclo en cada conversión simple
	hadc1.Init.ContinuousConvMode = DISABLE;            // Desactiva la lectura automática continua
	hadc1.Init.NbrOfConversion = 1;                     // Fija un solo canal por ciclo
	hadc1.Init.DiscontinuousConvMode = DISABLE;         // Apaga el modo discontinuo
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO; // Obliga al ADC a ser disparado por el Timer 2
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING; // Reacciona al flanco de subida del Trigger
	hadc1.Init.DMAContinuousRequests = DISABLE;         // Desactiva el controlador de memoria DMA

	HAL_ADC_Init(&hadc1);                               // Graba la configuración global en el ADC

	ADC_ChannelConfTypeDef adc_ch15 = { 0 };            // Crea estructura para el canal específico
	adc_ch15.Channel = ADC_CHANNEL_15;                  // Apunta al canal analógico 15
	adc_ch15.Rank = 1;                                  // Le da la prioridad inicial número 1
	adc_ch15.SamplingTime = ADC_SAMPLETIME_56CYCLES;    // Da un tiempo de carga razonable al capacitor interno
	adc_ch15.Offset = 0;                                // Configura el desfase en cero

	HAL_ADC_ConfigChannel(&hadc1, &adc_ch15);           // Asocia el canal con su configuración

	HAL_NVIC_EnableIRQ(ADC_IRQn);                       //matricula la interrupcion en el NVIC
	__NOP();
}


/*CONFIGURACION TIMER 2 (Muestreo 20ms)*/

static void tim2_Init(void) {
	__HAL_RCC_TIM2_CLK_ENABLE();			            // Enciende el RCC del Timer 2

	htim2.Instance = TIM2;                              // Selecciona el módulo hardware TIM2
	htim2.Init.Prescaler = 159;                         // Divide el reloj principal para que corra a 100 kHz (10 us/tick)
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;        // Hace que el reloj cuente hacia arriba
	htim2.Init.Period = 1999;                           // Limita la cuenta hasta llegar a 20 ms
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  // Omite división secundaria
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // Permite que el ciclo se recargue en automático
	HAL_TIM_Base_Init(&htim2);                          // Efectúa la inicialización del Timer

	TIM_MasterConfigTypeDef sMasterConfig = { 0 }; 		// Crea la configuración en modo maestro
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;// Empuja un disparo por hardware al finalizar cada ciclo de cuenta
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; // Apaga la cadena de múltiples esclavos
	HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig); // Aplica el comportamiento de Trigger al micro

	HAL_TIM_Base_Start(&htim2);                         // Arranca formalmente el conteo continuo del timer
}


/*CONFIGURACION TIMER 3 (Señales PWM)*/

static void tim3_pwm_Init(void) {
	__HAL_RCC_TIM3_CLK_ENABLE(); 					    // Habilita el reloj para el Timer 3
	__HAL_RCC_GPIOA_CLK_ENABLE(); 					    // Habilita el reloj del puerto A
	__HAL_RCC_GPIOB_CLK_ENABLE(); 					    // Habilita el reloj del puerto B

	GPIO_InitTypeDef GPIO_InitStruct = { 0 }; 		    // Prepara una variable para la gestión de pines

	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_1;	// Especifica los 3 pines de salida deseados
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;			    // Cambia su función a Alternativa Push-Pull (Controlados por periférico)
	GPIO_InitStruct.Pull = GPIO_NOPULL;				    // Remueve resistencias pull-up/pull-down
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;	    // Fija una transición de voltaje lenta/limpia
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;		    // Conecta internamente estos pines a la circuitería del TIM3
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);             // Registra PA6 y PA7
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);             // Registra PB1

	htim3.Instance = TIM3;                              // Selecciona el módulo TIM3
	htim3.Init.Prescaler = 3;       				    // Divide el reloj por 4
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;	    // Conteo normal ascendente
	htim3.Init.Period = 3999;       				    // Fija la máxima cuenta para lograr una frecuencia de 1 kHz exacta
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  // Desactiva divisiones de filtro
	HAL_TIM_PWM_Init(&htim3);                           // Construye la configuración base en el módulo de modulación de ancho de pulso

	TIM_OC_InitTypeDef sConfigOC = { 0 };               // Variable de Configuración del "Output Compare"
	sConfigOC.OCMode = TIM_OCMODE_PWM1;                 // Selecciona el esquema estándar donde pin sube y luego baja
	sConfigOC.Pulse = 0;                                // Arranca con el PWM en nivel 0 absoluto
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;         // Mantiene estado Alto durante el periodo activo
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;          // Ignora el modo acelerado

	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1); // Configura el canal 1 (ADC)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);                     // Enciende la salida de PWM en su pin PA6

	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2); // Configura el canal 2 (Encoder)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);                     // Enciende la salida de PWM en su pin PA7

	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4); // Configura el canal 4 (UART)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);                     // Enciende la salida de PWM en su pin PB1
}


/*CONFIGURACION TIMER 4 (Blinky / 250ms)*/

static void tim4_blinky_Init(void) {
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


/*CONFIGURACION TIMER 5 (Modo Encoder)*/

static void tim5_encoder_Init(void) {
	__HAL_RCC_TIM5_CLK_ENABLE();				        // Abastece de reloj al Timer 5
	__HAL_RCC_GPIOA_CLK_ENABLE();				        // Abastece de reloj al Puerto A

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };           // Prepara el esqueleto de configuración de pines

	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;      // Apunta a los pines conectados al Encoder físico
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;             // Fija la Función Alternativa para dejarlos bajo control del Timer
	GPIO_InitStruct.Pull = GPIO_PULLUP;                 // Activa resistencia de Pull-up vital para el sensor mecánico
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;       // Selecciona captura de muy alta velocidad para no perder pulsos
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;          // Especifica a qué periférico enlazar (AF2 = TIM5)
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);             // Carga todo esto en el banco de registros

	TIM_Encoder_InitTypeDef Encoder_Config = { 0 };     // Declara la subestructura exclusiva del decodificador

	htim5.Instance = TIM5;                              // Se apropia del recurso TIM5
	htim5.Init.Prescaler = 0;                           // Registra los pulsos uno a uno sin división
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;        // Define el sentido inicial de la matemática
	htim5.Init.Period = 65535;                          // Setea el límite más grande posible en 16bits para absorber giros amplios
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  // Suprime reloj derivado
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // Permite actualización suave del límite

	Encoder_Config.EncoderMode = TIM_ENCODERMODE_TI12;  // Escucha cambios en ambas pistas (A y B) ganando resolución x4

	// Propiedades de la pista 1 (Pin PA0)
	Encoder_Config.IC1Polarity = TIM_ICPOLARITY_RISING; // Define el estado de reposo a esperar
	Encoder_Config.IC1Selection = TIM_ICSELECTION_DIRECTTI; // Asocia el canal físico 1 al lógico 1
	Encoder_Config.IC1Prescaler = TIM_ICPSC_DIV1;       // Atrapa eventos al 100% de la frecuencia real
	Encoder_Config.IC1Filter = 10;                      // Aplica amortiguación digital para limpiar el ruido del contacto mecánico

	// Propiedades de la pista 2 (Pin PA1)
	Encoder_Config.IC2Polarity = TIM_ICPOLARITY_RISING; // Define el estado de reposo a esperar
	Encoder_Config.IC2Selection = TIM_ICSELECTION_DIRECTTI; // Asocia el canal físico 2 al lógico 2
	Encoder_Config.IC2Prescaler = TIM_ICPSC_DIV1;       // Atrapa eventos al 100% de la frecuencia real
	Encoder_Config.IC2Filter = 10;                      // Aplica amortiguación digital para limpiar el ruido del contacto mecánico

	HAL_TIM_Encoder_Init(&htim5, &Encoder_Config);      // Ejecuta la consolidación de toda esta configuración especializada

	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);     // Lanza el hardware a monitorizar ambos canales al mismo tiempo
}


/*CONFIGURACION SERIAL (USART2)*/

static void uart2_Init(void) {
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


/*RUTINAS DE INTERRUPCIÓN (CALLBACKS)*/


/* Se ejecuta automáticamente cada que un Timer finaliza su cuenta máxima */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) {                       // Verifica que la llamada proviene exclusivamente del TIM4 (250ms)
		HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);          // Cambia el estado actual del LED de placa (Blinky)
		flag_refresh_250ms = 1;                         // Informa a la Máquina Principal que es hora de actualizar la informaion en el serial
	}
}

/* Se ejecuta automáticamente cuando el periférico analógico termina de medir voltaje */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {                       // Comprueba la fuente de hardware
		raw_adc = hadc->Instance->DR;                   // Almacena el nuevo voltaje mapeado proveniente del Registro de Datos (DR)
		flag_adc_event = 1;                             // Deja la flag levantada para procesarlo en el bloque "while"

		HAL_ADC_Start_IT(hadc);                         // Arma nuevamente el sistema para la siguiente medición
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

/* NOTAS DE DISEÑO DEL CÓDIGO:
 * 1. Sobre la transmisión serial: Se establece que es posible optimizar la función
 * HAL_UART_Transmit restando "-1" a la longitud del texto y usando un timeout
 * de "100" ms por seguridad. Sin embargo, por facilidad de implementación,
 * mantiene el parámetro HAL_MAX_DELAY.
 * 2. Sobre los mensajes de texto: Se define los arreglos "char" (textos a enviar)
 * de manera local dentro de cada caso específico. Se omite definirlos de forma
 * global para mantener la independencia de cada bloque y evitar confusiones.
 * 3* Sobre el nombre de cierta variables se ha decidido poner nombre relacionadas
 * con su accion inmediata por lo que algunas pueden verse un poco largas.
 * 4. Es posible poner el % del ADC si usa la ecuacion adc_percent=0.258(raw_adc-220),
 * esto garantizar ver el % del adc, pero hay que poner un condicional para ver que
 * que uando llegue a la consicion de raw_adc<220 llevar a adc_percent=0, dado que valor
 * minimo de raw_adc es 213, segun el live expressions.
 */

