/*
 * main.c
 * Sistema de sintonía automática/manual para antena loop magnética
 * - Encoder 1 (TIM5): mueve manualmente el motor de la ANTENA (giro)
 * - Encoder 2 (TIM3): mueve manualmente el motor del CAPACITOR variable
 * - Encoder 3 (TIM2): ajusta la FRECUENCIA objetivo y su botón cambia
 *                      entre modo MANUAL y AUTOMATICO
 * - OLED SSD1306 (I2C1): muestra FREC, CAP, orientación de antena y
 *                         nivel de señal captado por el analizador de RF
 * - ADC1 (PA3): lee la salida analógica del detector logarítmico de RF
 *               AD8307 (voltaje ~ proporcional a dB, no a potencia lineal).
 *               Al ser logarítmica, el valor crudo del ADC ya sirve
 *               directamente para encontrar el pico de señal.
 *
 * Drivers de motor: A4988 (STEP/DIR). Recuerda unir SLEEP y RESET del
 * A4988 entre sí y a Vcc en tu cableado (si no, el driver no arranca).
 */

/* ========================================================================= */
/*                              1. INCLUDES                                  */
/* ========================================================================= */
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>


/* ========================================================================= */
/*                          2. VARIABLES GLOBALES                            */
/* ========================================================================= */

/* --- Handles de periféricos --- */
TIM_HandleTypeDef htim4 = { 0 }; // Blinky (LED de vida)
TIM_HandleTypeDef htim5 = { 0 }; // Encoder 1 -> motor antena
TIM_HandleTypeDef htim3 = { 0 }; // Encoder 2 -> motor capacitor
TIM_HandleTypeDef htim2 = { 0 }; // Encoder 3 -> frecuencia objetivo
I2C_HandleTypeDef  hi2c1 = { 0 }; // Pantalla OLED
ADC_HandleTypeDef  hadc1 = { 0 }; // Nivel de señal del analizador de RF

/* --- Posiciones crudas anteriores de cada encoder (para calcular delta) --- */
volatile uint16_t posicion_anterior_1 = 0; // encoder antena
volatile uint16_t posicion_anterior_2 = 0; // encoder capacitor
volatile uint16_t posicion_anterior_3 = 0; // encoder frecuencia

/* --- Modo de operación --- */
typedef enum { MODO_MANUAL = 0, MODO_AUTOMATICO } ModoOperacion;
volatile ModoOperacion modo_actual = MODO_MANUAL;

/* --- Estado de la aplicación (lo que se dibuja en pantalla) --- */
volatile float   frecuencia_obj_kHz   = 9955.0f; // fijada con el encoder 3
volatile float   capacitancia_pF      = 200.0f;  // calculada o de posición manual
volatile int32_t pos_motor_antena     = 0;       // pasos absolutos acumulados (para la aguja)
volatile int32_t pos_motor_cap        = 0;       // pasos absolutos acumulados del capacitor
volatile uint8_t  nivel_senal          = 0;       // 0-4, para las barras del S-meter
volatile uint8_t  pantalla_actualizar  = 1;       // bandera para refrescar el OLED

/* --- Buffer de la pantalla OLED --- */
#define SSD1306_ADDR 0x78
static uint8_t SSD1306_Buffer[1024];
char strBuff[24];

/* --- Fuente 5x7 ASCII --- */
const uint8_t Font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x5F, 0x00, 0x00},
    {0x00, 0x07, 0x00, 0x07, 0x00}, {0x14, 0x7F, 0x14, 0x7F, 0x14},
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, {0x23, 0x13, 0x08, 0x64, 0x62},
    {0x36, 0x49, 0x55, 0x22, 0x50}, {0x00, 0x05, 0x03, 0x00, 0x00},
    {0x00, 0x1C, 0x22, 0x41, 0x00}, {0x00, 0x41, 0x22, 0x1C, 0x00},
    {0x14, 0x08, 0x3E, 0x08, 0x14}, {0x08, 0x08, 0x3E, 0x08, 0x08},
    {0x00, 0x50, 0x30, 0x00, 0x00}, {0x08, 0x08, 0x08, 0x08, 0x08},
    {0x00, 0x60, 0x60, 0x00, 0x00}, {0x20, 0x10, 0x08, 0x04, 0x02},
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E},
    {0x00, 0x36, 0x36, 0x00, 0x00}, {0x00, 0x56, 0x36, 0x00, 0x00},
    {0x08, 0x14, 0x22, 0x41, 0x00}, {0x14, 0x14, 0x14, 0x14, 0x14},
    {0x00, 0x41, 0x22, 0x14, 0x08}, {0x02, 0x01, 0x51, 0x09, 0x06},
    {0x32, 0x49, 0x79, 0x41, 0x3E}, {0x7E, 0x11, 0x11, 0x11, 0x7E},
    {0x7F, 0x49, 0x49, 0x49, 0x36}, {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, {0x7F, 0x49, 0x49, 0x49, 0x41},
    {0x7F, 0x09, 0x09, 0x09, 0x01}, {0x3E, 0x41, 0x49, 0x49, 0x7A},
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01}, {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40}, {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x09, 0x09, 0x09, 0x06}, {0x3E, 0x41, 0x51, 0x21, 0x5E},
    {0x7F, 0x09, 0x19, 0x29, 0x46}, {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7F, 0x01, 0x01}, {0x3F, 0x40, 0x40, 0x40, 0x3F},
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, {0x3F, 0x40, 0x38, 0x40, 0x3F},
    {0x63, 0x14, 0x08, 0x14, 0x63}, {0x07, 0x08, 0x70, 0x08, 0x07},
    {0x61, 0x51, 0x49, 0x45, 0x43}, {0x00, 0x7F, 0x41, 0x41, 0x00},
    {0x02, 0x04, 0x08, 0x10, 0x20}, {0x00, 0x41, 0x41, 0x7F, 0x00},
    {0x04, 0x02, 0x01, 0x02, 0x04}, {0x40, 0x40, 0x40, 0x40, 0x40},
    {0x00, 0x01, 0x02, 0x04, 0x00}, {0x20, 0x54, 0x54, 0x54, 0x78},
    {0x7F, 0x48, 0x44, 0x44, 0x38}, {0x38, 0x44, 0x44, 0x44, 0x20},
    {0x38, 0x44, 0x44, 0x48, 0x7F}, {0x38, 0x54, 0x54, 0x54, 0x18},
    {0x08, 0x7E, 0x09, 0x01, 0x02}, {0x0C, 0x52, 0x52, 0x52, 0x3E},
    {0x7F, 0x08, 0x04, 0x04, 0x78}, {0x00, 0x44, 0x7D, 0x40, 0x00},
    {0x20, 0x40, 0x44, 0x3D, 0x00}, {0x7F, 0x10, 0x28, 0x44, 0x00},
    {0x00, 0x41, 0x7F, 0x40, 0x00}, {0x7C, 0x04, 0x18, 0x04, 0x78},
    {0x7C, 0x08, 0x04, 0x04, 0x78}, {0x38, 0x44, 0x44, 0x44, 0x38},
    {0x7C, 0x14, 0x14, 0x14, 0x08}, {0x08, 0x14, 0x14, 0x18, 0x7C},
    {0x7C, 0x08, 0x04, 0x04, 0x08}, {0x48, 0x54, 0x54, 0x54, 0x20},
    {0x04, 0x3F, 0x44, 0x40, 0x20}, {0x3C, 0x40, 0x40, 0x20, 0x7C},
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, {0x3C, 0x40, 0x30, 0x40, 0x3C},
    {0x44, 0x28, 0x10, 0x28, 0x44}, {0x0C, 0x50, 0x50, 0x50, 0x3C},
    {0x44, 0x64, 0x54, 0x4C, 0x44}, {0x00, 0x08, 0x36, 0x41, 0x00},
    {0x00, 0x00, 0x7F, 0x00, 0x00}, {0x00, 0x41, 0x36, 0x08, 0x00},
    {0x0C, 0x02, 0x0C, 0x02, 0x0C},
};

/* ------------------------------------------------------------------------
 * CONSTANTES DE CALIBRACIÓN -- AJUSTA ESTOS VALORES A TU HARDWARE REAL
 * ------------------------------------------------------------------------ */
#define L_ANTENA_HENRIOS      2.2e-6f   // TODO: inductancia medida de tu loop grande
#define CAP_MIN_PF            10.0f     // TODO: capacitancia mínima mecánica del variable
#define CAP_MAX_PF            365.0f    // TODO: capacitancia máxima mecánica del variable
#define PASOS_FIN_A_FIN_CAP   2038      // TODO: pasos totales de tope a tope del capacitor
#define PASOS_VUELTA_ANTENA   2038      // TODO: pasos de una vuelta completa del motor de antena
#define PASO_FRECUENCIA_KHZ   5.0f      // cuánto cambia FREC por cada click del encoder 3
#define UMBRAL_ADC_POR_BARRA  800       // TODO: calibrar según la salida real del AD8307 (mV/dB de tu montaje)

/* ========================================================================= */
/*                       3. CABECERAS DE FUNCIONES                           */
/* ========================================================================= */

/* --- Configuración de hardware --- */
static void SystemClock_Config(void);
static void gpio_Init(void);
static void tim4_blinky_Init(void);
static void tim5_encoder1_Init(void);
static void tim3_encoder2_Init(void);
static void tim2_encoder3_Init(void);
static void i2c1_Init(void);
static void adc1_Init(void);

/* --- Driver OLED SSD1306 --- */
void WriteCmd(uint8_t c);
void SSD1306_Init(void);
void SSD1306_Fill(uint8_t color);
void SSD1306_UpdateScreen(void);
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void SSD1306_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r);
void SSD1306_WriteString(uint8_t x, uint8_t y, char* str);

/* --- Motores paso a paso --- */
void mover_motor_antena(int16_t pasos);
void mover_motor_capacitor(int16_t pasos);

/* --- Lógica de la aplicación --- */
float calcular_capacitancia_pF(float frecuencia_kHz);
int32_t capacitancia_a_pasos(float cap_pF);
uint16_t leer_nivel_rf_bruto(void);
void actualizar_nivel_senal(void);
void ejecutar_sintonia_automatica(void);
void actualizar_pantalla(void);

/* ========================================================================= */
/*                              4. FUNCIÓN MAIN                              */
/* ========================================================================= */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    gpio_Init();
    tim4_blinky_Init();
    tim5_encoder1_Init();
    tim3_encoder2_Init();
    tim2_encoder3_Init();
    i2c1_Init();
    adc1_Init();

    SSD1306_Init();
    SSD1306_Fill(0);
    SSD1306_UpdateScreen();

    /* Captura la posición inicial de hardware de los 3 encoders */
    posicion_anterior_1 = __HAL_TIM_GET_COUNTER(&htim5);
    posicion_anterior_2 = __HAL_TIM_GET_COUNTER(&htim3);
    posicion_anterior_3 = __HAL_TIM_GET_COUNTER(&htim2);

    uint32_t ultimo_refresco_lento = HAL_GetTick();

    while (1)
    {
        /* -----------------------------------------------------------
         * 1. BOTÓN DEL ENCODER 1 (PA4): pone a cero la aguja de antena
         * ----------------------------------------------------------- */
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) {
            HAL_Delay(30);
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) {
                pos_motor_antena = 0;
                pantalla_actualizar = 1;
                while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) HAL_Delay(10);
            }
        }

        /* -----------------------------------------------------------
         * 2. BOTÓN DEL ENCODER 2 (PC10): fija el 0 pF del capacitor
         * ----------------------------------------------------------- */
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_RESET) {
            HAL_Delay(30);
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_RESET) {
                pos_motor_cap = 0;
                pantalla_actualizar = 1;
                while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_RESET) HAL_Delay(10);
            }
        }

        /* -----------------------------------------------------------
         * 3. BOTÓN DEL ENCODER 3 (PB4): alterna MANUAL <-> AUTOMATICO
         *    Al entrar (o reintentar) en AUTOMATICO, ejecuta la sintonía
         * ----------------------------------------------------------- */
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET) {
            HAL_Delay(30);
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET) {
                if (modo_actual == MODO_MANUAL) {
                    modo_actual = MODO_AUTOMATICO;
                    ejecutar_sintonia_automatica();
                    modo_actual = MODO_MANUAL; // vuelve a manual al terminar el barrido
                }
                pantalla_actualizar = 1;
                while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET) HAL_Delay(10);
            }
        }

        /* -----------------------------------------------------------
         * 4. ENCODER 3: cambia la frecuencia objetivo (activo siempre)
         * ----------------------------------------------------------- */
        uint16_t pos_actual_3 = __HAL_TIM_GET_COUNTER(&htim2);
        int16_t diff_3 = (int16_t)(pos_actual_3 - posicion_anterior_3);
        int16_t clicks_3 = diff_3 / 4;
        if (clicks_3 != 0) {
            posicion_anterior_3 += (clicks_3 * 4);
            frecuencia_obj_kHz += clicks_3 * PASO_FRECUENCIA_KHZ;
            if (frecuencia_obj_kHz < 0) frecuencia_obj_kHz = 0;
            pantalla_actualizar = 1;
        }

        /* -----------------------------------------------------------
         * 5. ENCODERS 1 y 2: mueven los motores SOLO en modo manual
         * ----------------------------------------------------------- */
        if (modo_actual == MODO_MANUAL) {
            uint16_t pos_actual_1 = __HAL_TIM_GET_COUNTER(&htim5);
            int16_t diff_1 = (int16_t)(pos_actual_1 - posicion_anterior_1);
            int16_t pasos_1 = diff_1 / 4;

            uint16_t pos_actual_2 = __HAL_TIM_GET_COUNTER(&htim3);
            int16_t diff_2 = (int16_t)(pos_actual_2 - posicion_anterior_2);
            int16_t pasos_2 = diff_2 / 4;

            if (pasos_1 != 0) {
                posicion_anterior_1 += (pasos_1 * 4);
                mover_motor_antena(pasos_1);
                pantalla_actualizar = 1;
            }
            if (pasos_2 != 0) {
                posicion_anterior_2 += (pasos_2 * 4);
                mover_motor_capacitor(pasos_2);
                pantalla_actualizar = 1;
            }
        } else {
            /* En modo automático no se acumulan movimientos manuales pendientes */
            posicion_anterior_1 = __HAL_TIM_GET_COUNTER(&htim5);
            posicion_anterior_2 = __HAL_TIM_GET_COUNTER(&htim3);
        }

        /* -----------------------------------------------------------
         * 6. Refresco periódico del nivel de señal y de la pantalla
         * ----------------------------------------------------------- */
        if (HAL_GetTick() - ultimo_refresco_lento >= 150) {
            ultimo_refresco_lento = HAL_GetTick();
            actualizar_nivel_senal();
            pantalla_actualizar = 1;
        }

        if (pantalla_actualizar) {
            pantalla_actualizar = 0;
            actualizar_pantalla();
        }
    }
}

/* ========================================================================= */
/*                          5. FUNCIONES                                     */
/* ========================================================================= */

/* --------------------------------------------------------------------------
 * MOTORES PASO A PASO
 * -------------------------------------------------------------------------- */
void mover_motor_antena(int16_t pasos)
{
    int16_t abs_pasos = pasos;
    if (pasos > 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        abs_pasos = -pasos;
    }
    for (int i = 0; i < abs_pasos; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_Delay(2);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_Delay(2);
    }
    pos_motor_antena += pasos;
}

void mover_motor_capacitor(int16_t pasos)
{
    int16_t abs_pasos = pasos;
    if (pasos > 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        abs_pasos = -pasos;
    }
    for (int i = 0; i < abs_pasos; i++) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(2);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_Delay(2);
    }
    pos_motor_cap += pasos;
    capacitancia_pF = CAP_MIN_PF + ((float)pos_motor_cap / (float)PASOS_FIN_A_FIN_CAP) * (CAP_MAX_PF - CAP_MIN_PF);
}

/* --------------------------------------------------------------------------
 * LÓGICA DE SINTONÍA
 * -------------------------------------------------------------------------- */
float calcular_capacitancia_pF(float frecuencia_kHz)
{
    float f_hz = frecuencia_kHz * 1000.0f;
    if (f_hz <= 0.0f) return CAP_MAX_PF;
    float c_faradios = 1.0f / (powf(2.0f * (float)M_PI * f_hz, 2.0f) * L_ANTENA_HENRIOS);
    return c_faradios * 1.0e12f; // a pF
}

int32_t capacitancia_a_pasos(float cap_pF)
{
    if (cap_pF < CAP_MIN_PF) cap_pF = CAP_MIN_PF;
    if (cap_pF > CAP_MAX_PF) cap_pF = CAP_MAX_PF;
    float fraccion = (cap_pF - CAP_MIN_PF) / (CAP_MAX_PF - CAP_MIN_PF);
    return (int32_t)(fraccion * PASOS_FIN_A_FIN_CAP);
}

uint16_t leer_nivel_rf_bruto(void)
{
    /* Lee la salida analógica del AD8307 (0-4095 = 0-3.3V aprox).
     * Al ser una respuesta logarítmica (~25 mV/dB), el valor crudo del
     * ADC ya es proporcional a dB, así que buscar su máximo durante el
     * barrido de la antena equivale directamente a buscar el pico de
     * potencia de RF recibida. */
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t valor = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return valor;
}

void actualizar_nivel_senal(void)
{
    uint16_t bruto = leer_nivel_rf_bruto();
    if (bruto < 1 * UMBRAL_ADC_POR_BARRA)      nivel_senal = 0;
    else if (bruto < 2 * UMBRAL_ADC_POR_BARRA) nivel_senal = 1;
    else if (bruto < 3 * UMBRAL_ADC_POR_BARRA) nivel_senal = 2;
    else if (bruto < 4 * UMBRAL_ADC_POR_BARRA) nivel_senal = 3;
    else                                       nivel_senal = 4;
}

void ejecutar_sintonia_automatica(void)
{
    /* 1. Calcular la capacitancia ideal para la frecuencia deseada
     *    y mover el motor del capacitor a esa posición */
    capacitancia_pF = calcular_capacitancia_pF(frecuencia_obj_kHz);
    int32_t pasos_cap_objetivo = capacitancia_a_pasos(capacitancia_pF);
    int32_t delta_cap = pasos_cap_objetivo - pos_motor_cap;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, (delta_cap > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    int32_t pasos_restantes = (delta_cap > 0) ? delta_cap : -delta_cap;
    for (int32_t i = 0; i < pasos_restantes; i++) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(2);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_Delay(2);
    }
    pos_motor_cap = pasos_cap_objetivo;

    /* 2. Girar la antena una vuelta completa buscando el pico de señal
     *    que reporte el analizador de RF */
    uint16_t mejor_nivel = 0;
    int32_t pos_mejor = pos_motor_antena;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    for (int32_t i = 0; i < PASOS_VUELTA_ANTENA; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_Delay(2);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_Delay(2);
        pos_motor_antena++;

        uint16_t nivel = leer_nivel_rf_bruto();
        if (nivel > mejor_nivel) {
            mejor_nivel = nivel;
            pos_mejor = pos_motor_antena;
        }

        if ((i % 15) == 0) {
            actualizar_nivel_senal();
            actualizar_pantalla();
        }
    }

    /* 3. Regresar la antena a la posición donde se detectó el pico */
    int32_t regreso = pos_motor_antena - pos_mejor;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (regreso > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    int32_t pasos_regreso = (regreso > 0) ? regreso : -regreso;
    for (int32_t i = 0; i < pasos_regreso; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_Delay(2);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_Delay(2);
        pos_motor_antena += (regreso > 0) ? -1 : 1;
    }

    actualizar_nivel_senal();
}

/* --------------------------------------------------------------------------
 * PANTALLA
 * -------------------------------------------------------------------------- */
#define OLED_CIRCLE_CX 45
#define OLED_CIRCLE_CY 40
#define OLED_CIRCLE_R  22
#define OLED_BAR_X0    95

void actualizar_pantalla(void)
{
    SSD1306_Fill(0);

    sprintf(strBuff, "FREC:%u kHz", (unsigned int)frecuencia_obj_kHz);
    SSD1306_WriteString(0, 0, strBuff);

    sprintf(strBuff, "CAP:%u pF", (unsigned int)capacitancia_pF);
    SSD1306_WriteString(80, 0, strBuff);

    for (int x = 0; x < 128; x++) SSD1306_DrawPixel(x, 12, 1);

    /* Círculo indicador: orientación actual de la antena */
    SSD1306_DrawCircle(OLED_CIRCLE_CX, OLED_CIRCLE_CY, OLED_CIRCLE_R);
    int32_t pos_normalizada = pos_motor_antena % PASOS_VUELTA_ANTENA;
    if (pos_normalizada < 0) pos_normalizada += PASOS_VUELTA_ANTENA;
    float angulo_rad = (2.0f * (float)M_PI * (float)pos_normalizada) / (float)PASOS_VUELTA_ANTENA;
    int16_t punta_x = OLED_CIRCLE_CX + (int16_t)(OLED_CIRCLE_R * cosf(angulo_rad));
    int16_t punta_y = OLED_CIRCLE_CY + (int16_t)(OLED_CIRCLE_R * sinf(angulo_rad));
    SSD1306_DrawLine(OLED_CIRCLE_CX, OLED_CIRCLE_CY, punta_x, punta_y);

    /* Barras: nivel de señal captado por el analizador de RF */
    uint8_t alturas[4] = {10, 18, 26, 34};
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t bx0 = OLED_BAR_X0 + i * 8;
        if (i < nivel_senal) {
            for (int yy = 62 - alturas[i]; yy < 62; yy++)
                for (int xx = bx0; xx < bx0 + 6; xx++)
                    SSD1306_DrawPixel(xx, yy, 1);
        }
    }

    SSD1306_UpdateScreen();
}

/* --------------------------------------------------------------------------
 * DRIVER SSD1306
 * -------------------------------------------------------------------------- */
void WriteCmd(uint8_t c)
{
    HAL_I2C_Mem_Write(&hi2c1, SSD1306_ADDR, 0x00, 1, &c, 1, 10);
}

void SSD1306_Init(void)
{
    HAL_Delay(100);
    WriteCmd(0xAE); WriteCmd(0x20); WriteCmd(0x10); WriteCmd(0xB0);
    WriteCmd(0xC8); WriteCmd(0x00); WriteCmd(0x10); WriteCmd(0x40);
    WriteCmd(0x81); WriteCmd(0xFF); WriteCmd(0xA1); WriteCmd(0xA6);
    WriteCmd(0xA8); WriteCmd(0x3F); WriteCmd(0xA4); WriteCmd(0xD3);
    WriteCmd(0x00); WriteCmd(0xD5); WriteCmd(0xF0); WriteCmd(0xD9);
    WriteCmd(0x22); WriteCmd(0xDA); WriteCmd(0x12); WriteCmd(0xDB);
    WriteCmd(0x20); WriteCmd(0x8D); WriteCmd(0x14); WriteCmd(0xAF);
}

void SSD1306_Fill(uint8_t color)
{
    memset(SSD1306_Buffer, (color == 0) ? 0 : 0xFF, sizeof(SSD1306_Buffer));
}

void SSD1306_UpdateScreen(void)
{
    for (int i = 0; i < 8; i++) {
        WriteCmd(0xB0 + i);
        WriteCmd(0x00);
        WriteCmd(0x10);
        HAL_I2C_Mem_Write(&hi2c1, SSD1306_ADDR, 0x40, 1, &SSD1306_Buffer[128 * i], 128, 50);
    }
}

void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= 128 || y >= 64) return;
    if (color) SSD1306_Buffer[x + (y / 8) * 128] |= (1 << (y % 8));
    else       SSD1306_Buffer[x + (y / 8) * 128] &= ~(1 << (y % 8));
}

void SSD1306_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
    int16_t dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int16_t dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx + dy, e2;
    while (1) {
        SSD1306_DrawPixel((uint8_t)x0, (uint8_t)y0, 1);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r)
{
    int16_t x = r, y = 0, err = 0;
    while (x >= y) {
        SSD1306_DrawPixel((uint8_t)(x0 + x), (uint8_t)(y0 + y), 1);
        SSD1306_DrawPixel((uint8_t)(x0 + y), (uint8_t)(y0 + x), 1);
        SSD1306_DrawPixel((uint8_t)(x0 - y), (uint8_t)(y0 + x), 1);
        SSD1306_DrawPixel((uint8_t)(x0 - x), (uint8_t)(y0 + y), 1);
        SSD1306_DrawPixel((uint8_t)(x0 - x), (uint8_t)(y0 - y), 1);
        SSD1306_DrawPixel((uint8_t)(x0 - y), (uint8_t)(y0 - x), 1);
        SSD1306_DrawPixel((uint8_t)(x0 + y), (uint8_t)(y0 - x), 1);
        SSD1306_DrawPixel((uint8_t)(x0 + x), (uint8_t)(y0 - y), 1);
        y++;
        if (err <= 0) { err += 2 * y + 1; }
        if (err > 0)  { x--; err -= 2 * x + 1; }
    }
}

void SSD1306_WriteString(uint8_t x, uint8_t y, char* str)
{
    while (*str) {
        char c = *str;
        uint8_t idx = (c >= 32 && c <= 126) ? (c - 32) : 0;
        for (int i = 0; i < 5; i++) {
            uint8_t b = Font5x7[idx][i];
            for (int j = 0; j < 8; j++) {
                if ((b >> j) & 1) SSD1306_DrawPixel(x + i, y + j, 1);
            }
        }
        x += 6;
        str++;
    }
}

/* --------------------------------------------------------------------------
 * CONFIGURACIONES DE HARDWARE (INITS)
 * -------------------------------------------------------------------------- */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /* Regulador de voltaje principal */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /* HSI (16 MHz) -> PLL -> SYSCLK 84 MHz */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { while(1); }

    /* AHB = 84 MHz, APB1 = 42 MHz (max 50 MHz), APB2 = 84 MHz (max 100 MHz) */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) { while(1); }
}

static void gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* LED de vida (PH1) */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /* --- MOTOR ANTENA (GPIOA): PA5 STEP, PA6 DIR --- */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA4: botón encoder 1 (cero de la aguja de antena) */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* --- MOTOR CAPACITOR (GPIOC): PC8 STEP, PC9 DIR --- */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PC10: botón encoder 2 (cero del capacitor) */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PB4: botón encoder 3 (cambia MANUAL/AUTOMATICO)
     * NOTA: PB4 es NJTRST por defecto (parte del JTAG). Al usarlo como GPIO
     * se pierde la depuración por JTAG completo, pero el SWD (2 pines)
     * sigue funcionando con normalidad. */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void tim4_blinky_Init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 83999;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 249;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim4);

    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    HAL_TIM_Base_Start_IT(&htim4);
}

static void tim5_encoder1_Init(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    TIM_Encoder_InitTypeDef Encoder_Config = { 0 };
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 65535;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    Encoder_Config.EncoderMode = TIM_ENCODERMODE_TI12;
    Encoder_Config.IC1Polarity = TIM_ICPOLARITY_RISING;
    Encoder_Config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_Config.IC1Prescaler = TIM_ICPSC_DIV1;
    Encoder_Config.IC1Filter = 10;
    Encoder_Config.IC2Polarity = TIM_ICPOLARITY_RISING;
    Encoder_Config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_Config.IC2Prescaler = TIM_ICPSC_DIV1;
    Encoder_Config.IC2Filter = 10;

    HAL_TIM_Encoder_Init(&htim5, &Encoder_Config);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

static void tim3_encoder2_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    TIM_Encoder_InitTypeDef Encoder_Config = { 0 };
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    Encoder_Config.EncoderMode = TIM_ENCODERMODE_TI12;
    Encoder_Config.IC1Polarity = TIM_ICPOLARITY_RISING;
    Encoder_Config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_Config.IC1Prescaler = TIM_ICPSC_DIV1;
    Encoder_Config.IC1Filter = 10;
    Encoder_Config.IC2Polarity = TIM_ICPOLARITY_RISING;
    Encoder_Config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_Config.IC2Prescaler = TIM_ICPSC_DIV1;
    Encoder_Config.IC2Filter = 10;

    HAL_TIM_Encoder_Init(&htim3, &Encoder_Config);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

static void tim2_encoder3_Init(void)
{
    /* Encoder 3 (frecuencia): PA15 = TIM2_CH1, PB3 = TIM2_CH2
     * NOTA: PA15/PB3 son JTDI/JTDO por defecto (JTAG). Al usarlos como AF de
     * TIM2 se pierde JTAG completo, pero el SWD sigue funcionando normalmente. */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    TIM_Encoder_InitTypeDef Encoder_Config = { 0 };
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    Encoder_Config.EncoderMode = TIM_ENCODERMODE_TI12;
    Encoder_Config.IC1Polarity = TIM_ICPOLARITY_RISING;
    Encoder_Config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_Config.IC1Prescaler = TIM_ICPSC_DIV1;
    Encoder_Config.IC1Filter = 10;
    Encoder_Config.IC2Polarity = TIM_ICPOLARITY_RISING;
    Encoder_Config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_Config.IC2Prescaler = TIM_ICPSC_DIV1;
    Encoder_Config.IC2Filter = 10;

    HAL_TIM_Encoder_Init(&htim2, &Encoder_Config);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

static void i2c1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_i2c_config = { 0 };
    GPIO_i2c_config.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_i2c_config.Mode = GPIO_MODE_AF_OD;
    GPIO_i2c_config.Pull = GPIO_NOPULL;
    GPIO_i2c_config.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_i2c_config);

    __HAL_RCC_I2C1_CLK_ENABLE();

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

static void adc1_Init(void)
{
    /* PA3 = ADC1_IN3: se conecta a la salida "OUT" del módulo AD8307-EVAL.
     * Verifica el rango real de voltaje de tu módulo (normalmente 0-2V,
     * a veces con buffer que lo lleva más cerca de 0-3.3V) para no
     * saturar ni desperdiciar rango del ADC del STM32. */
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef sConfig = { 0 };
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);
    }
}
