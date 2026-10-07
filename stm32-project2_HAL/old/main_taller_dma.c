/*
 * main_taller_dma.c
 *
 *  Created on: Jul 21, 2026
 *      Author: juand
 */


#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

#define BUFFER_SIZE 2000

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim11;

uint16_t adc_buffer[BUFFER_SIZE];
volatile uint8_t half_transfer_ready = 0;
volatile uint8_t full_transfer_ready = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, BUFFER_SIZE);

    while (1)
    {
        if (half_transfer_ready)
        {
            half_transfer_ready = 0;

            for (int i = 0; i < BUFFER_SIZE / 2; i++)
            {
                uint32_t mv = (adc_buffer[i] * 3300) / 4095;
                char tx_buf[32];
                int len = sprintf(tx_buf, "%lu.%03lu V\r\n", mv / 1000, mv % 1000);
                HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, len, 10);
            }
        }

        if (full_transfer_ready)
        {
            full_transfer_ready = 0;

            for (int i = BUFFER_SIZE / 2; i < BUFFER_SIZE; i++)
            {
                uint32_t mv = (adc_buffer[i] * 3300) / 4095;
                char tx_buf[32];
                int len = sprintf(tx_buf, "%lu.%03lu V\r\n", mv / 1000, mv % 1000);
                HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, len, 10);
            }

            HAL_UART_Transmit(&huart2, (uint8_t *)"--- Reiniciando Captura DMA ---\r\n\r\n", 35, 100);

            HAL_Delay(500);

            HAL_ADC_Stop_DMA(&hadc1);
            HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, BUFFER_SIZE);
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_HCLK   |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin   = GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    GPIO_InitStruct.Pin  = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_DMA_Init(void)
{
    /* Habilita el reloj del periférico DMA2. El ADC1 en el STM32F411 está conectado al DMA2 */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* 1. Selecciona la instancia física del DMA (Stream 0 de DMA2) */
    hdma_adc1.Instance                 = DMA2_Stream0;

    /* 2. Selecciona el canal 0, que es el asignado al ADC1 dentro de este Stream */
    hdma_adc1.Init.Channel             = DMA_CHANNEL_0;

    /* 3. Dirección de la transferencia: Desde el periférico hacia la memoria RAM */
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;

    /* 4. Incremento de dirección del periférico: DESHABILITADO.
          El periférico origen es el registro de datos del ADC (ADC_DR), el cual se encuentra
          en una única dirección física de memoria constante. No debe incrementarse. */
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;

    /* 5. Incremento de dirección en memoria RAM: HABILITADO.
          Permite que el DMA avance a la siguiente posición del array de destino (adc_buffer)
          cada vez que recibe un nuevo dato del ADC, evitando sobrescribir siempre la primera posición. */
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;

    /* 6. Alineación de datos en el periférico: Media palabra (16-bits / Half-word)
          El ADC1 entrega resultados de 12 bits, que caben en 16 bits. */
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;

    /* 7. Alineación de datos en la memoria RAM: Media palabra (16-bits)
          Para coincidir con el tamaño de adc_buffer (uint16_t). */
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;

    /* 8. Modo DMA: Normal.
          El DMA llenará el buffer una vez y se detendrá. En el bucle principal se reinicia de forma manual. */
    hdma_adc1.Init.Mode                = DMA_NORMAL;

    /* 9. Prioridad del Stream de DMA: Alta */
    hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;

    /* 10. Uso del buffer FIFO: Deshabilitado.
     *        Se transfieren los datos directamente sin almacenamiento intermedio. */
    hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    /* Inicializa el DMA con los parámetros configurados */
    HAL_DMA_Init(&hdma_adc1);

    /* Vincula la configuración de DMA al manejador del periférico ADC */
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    /* Habilita la interrupción del Stream 0 de DMA2 en el controlador NVIC */
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV8;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

    HAL_ADC_Init(&hadc1);

    sConfig.Channel      = ADC_CHANNEL_1;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    sConfig.Offset       = 0;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

static void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&huart2);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        half_transfer_ready = 1;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        full_transfer_ready = 1;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}
