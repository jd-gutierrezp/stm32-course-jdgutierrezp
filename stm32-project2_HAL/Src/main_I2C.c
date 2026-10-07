#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>

#define WHOAMI (0x68 << 1)
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define PWR_MGMT_1_REG 0x6B
#define ACCEL_XOUT_H_REG 0x3B

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
float Ax, Ay, Az;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_USART2_UART_Init(void);
void MPU6050_Init(void);
void MPU6050_Read_Accel(void);
void Error_Handler(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MPU6050_Init();

    char msg[128];

    while (1) {
        MPU6050_Read_Accel();

        // Imprimimos los flotantes directamente usando %.3f.
        // NOTA: Para que esto funcione en STM32, asegúrate de activar la opción
        // "Use float with printf from newlib-nano (-u _printf_float)" en las propiedades de tu proyecto.
        int len = sprintf(msg, "Ax: %.3f g | Ay: %.3f g | Az: %.3f g\r\n", Ax, Ay, Az);
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, len, 100);

        HAL_Delay(1000);
    }
}

void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
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
    // Habilitar Clock Stretching (estiramiento del reloj) para dar tiempo de procesamiento al sensor
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

void MPU6050_Init(void) {
    uint8_t Data;

    // Se escribe el dato 0x00 en el registro PWR_MGMT_1_REG (0x6B) del dispositivo WHOAMI (0xD0).
    // Esto despierta al sensor MPU6050 desactivando el modo de bajo consumo (Sleep Mode),
    // el cual viene activo por defecto al iniciar, y selecciona el oscilador interno de 8MHz.
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, WHOAMI, PWR_MGMT_1_REG, 1, &Data, 1, 100);

    // Se escribe el dato 0x07 en el registro SMPLRT_DIV_REG (0x19) del dispositivo WHOAMI (0xD0).
    // Configura el divisor de tasa de muestreo. Al escribir 7, la frecuencia del giroscopio (8 kHz por defecto)
    // se divide entre 8 (1 + 7), estableciendo una tasa de muestreo de salida de 1 kHz para los sensores.
    Data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, WHOAMI, SMPLRT_DIV_REG, 1, &Data, 1, 100);

    // Se escribe el dato 0x00 en el registro ACCEL_CONFIG_REG (0x1C) del dispositivo WHOAMI (0xD0).
    // Configura el rango de escala completa del acelerómetro en ±2g (bits AFS_SEL a 00).
    // Esto establece una sensibilidad del acelerómetro de 16384 LSB/g, que es el factor divisor empleado luego.
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, WHOAMI, ACCEL_CONFIG_REG, 1, &Data, 1, 100);
}

void MPU6050_Read_Accel(void) {
    uint8_t Rec_Data[6];
    HAL_StatusTypeDef status;

    // Timeout reducido a 100ms para evitar congelamientos largos si se desconecta
    status = HAL_I2C_Mem_Read(&hi2c1, WHOAMI, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 100);

    if (status == HAL_OK) {
        int16_t x = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        int16_t y = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        int16_t z = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

        // Si los tres ejes leen exactamente 0, es señal física de que el sensor se reinició y está en Sleep Mode
        if (x == 0 && y == 0 && z == 0) {
            HAL_UART_Transmit(&huart2, (uint8_t *)"[DEBUG] Sensor en Sleep Mode (0,0,0). Waking up...\r\n", 52, 100);
            MPU6050_Init();
        } else {
            Accel_X_RAW = x;
            Accel_Y_RAW = y;
            Accel_Z_RAW = z;

            Ax = Accel_X_RAW / 16384.0;
            Ay = Accel_Y_RAW / 16384.0;
            Az = Accel_Z_RAW / 16384.0;
        }
    } else {
        char err_msg[64];
        int len = sprintf(err_msg, "[DEBUG] Error I2C (status=%d). Reintentando...\r\n", status);
        HAL_UART_Transmit(&huart2, (uint8_t *)err_msg, len, 100);

        // Si el bus I2C está bloqueado, se re-inicializa el periférico I2C1 de la STM32
        if (status == HAL_BUSY) {
            HAL_I2C_DeInit(&hi2c1);
            MX_I2C1_Init();
        }

        MPU6050_Init();
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
                                  RCC_CLOCKTYPE_HCLK   |
                                  RCC_CLOCKTYPE_PCLK1  |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

void Error_Handler(void) {
    while (1) {
    }
}

