/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "boiler.h"
#include "kalman.h"
#include "led.h"
#include "pump.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "switch.h"
#include "waterlevel.h"
#include <cstring>
#include <functional>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COFFEE_TEMP 93
#define STEAM_TEMP 124

typedef struct __attribute__((__packed__)) {
    float tempBoiler;
    float tempSteam;
    float c;
    float d;
} FloatStruct;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int32_t last_time;
int32_t time;

#define RxSIZE 16
uint8_t aTxBuffer[RxSIZE];
uint8_t aRxBuffer[RxSIZE];

uint8_t rxcount = 0;
uint8_t txcount = 0;

int countAddr = 0;
int countrxcplt = 0;
int counterror = 0;

FloatStruct buffer = {1.0f, 2.0f, 3.0f, 4.0f};

uint32_t valuesADC[6];

TempProbe tempProbeCoffee;
TempProbe tempProbeSteam;

float tempCoffee = 0;
float tempSteam = 0;

int coffeTargetTemp = COFFEE_TEMP;

Pump coffeePump(PUMP_COFFEE_GPIO_Port, PUMP_COFFEE_Pin);
Pump steamPump(PUMP_STEAM_GPIO_Port, PUMP_STEAM_Pin);

WaterLevel coffeeWaterLevel(WATER_LEVEL_EN_1_GPIO_Port, WATER_LEVEL_EN_1_Pin,
                            &valuesADC[4]);

WaterLevel steamWaterLevel(WATER_LEVEL_EN_2_GPIO_Port, WATER_LEVEL_EN_2_Pin,
                           &valuesADC[5]);

LED led1(LED1_GPIO_Port, LED1_Pin);
LED led2(LED2_GPIO_Port, LED2_Pin);

Switch switch1(SW1_GPIO_Port, SW1_Pin);
Switch switch2(SW2_GPIO_Port, SW2_Pin);

Boiler coffeeBoiler(led1, tempProbeCoffee, coffeeWaterLevel, coffeePump);
Boiler steamBoiler(led2, tempProbeSteam, steamWaterLevel, steamPump);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU
     * Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */
    std::memcpy(aRxBuffer, &coffeTargetTemp, 4);

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();
    MX_I2C1_Init();

    if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK) {
        /* Transfer error in reception process */
        Error_Handler();
    }
    /* USER CODE BEGIN 2 */

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&valuesADC, 6);

    tempProbeCoffee.setADCRef(&valuesADC[2]);
    tempProbeSteam.setADCRef(&valuesADC[0]);

    coffeeBoiler.setHeater(HEATER_COFFEE_GPIO_Port, HEATER_COFFEE_Pin);
    coffeeBoiler.setMinMaxTemp(20, 100);
    coffeeBoiler.setTargetTemp(COFFEE_TEMP);
    // coffeeBoiler.setPidParam(5, 0.3, 180);
    coffeeBoiler.setPidParam(5, 0.3, 200);

    steamBoiler.setHeater(HEATER_STEAM_GPIO_Port, HEATER_STEAM_Pin);
    steamBoiler.setTargetTemp(STEAM_TEMP);
    steamBoiler.setPidParam(15, 0.3, 280);

    switch1.setEventHandle1(std::bind(&Boiler::toggleState, &coffeeBoiler));
    switch2.setEventHandle1(std::bind(&Boiler::toggleState, &steamBoiler));
    // switch1.setEventHandle2(std::bind(&Pump::pump, &coffeePump, 5));

    coffeeWaterLevel.setHandleLevelLow(std::bind(&Pump::pump, &coffeePump, 3));
    steamWaterLevel.setHandleLevelLow(std::bind(&Pump::pump, &steamPump, 3));

    // switch2.setEventHandle2(std::bind(&LED::doBlink, &led2));
    // switch2.setEventHandle1(std::bind(&LED::toggle, &led2));
    // switch2.setEventHandle1(std::bind(&Pump::togglePump, &coffeePump));

    HAL_Delay(1000);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */
        time = HAL_GetTick() - last_time;
        last_time = HAL_GetTick();

        /* USER CODE BEGIN 3 */
        coffeeBoiler.update();
        steamBoiler.update();

        switch1.update();
        switch2.update();

        tempCoffee = coffeeBoiler.getTemperature();
        tempSteam = steamBoiler.getTemperature();

        if (coffeTargetTemp != coffeeBoiler.getTargetTemp())
            coffeeBoiler.setTargetTemp(coffeTargetTemp);

        buffer.tempBoiler = coffeeBoiler.getTemperature();
        buffer.tempSteam = steamBoiler.getTemperature();

        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&valuesADC, 6);

        HAL_Delay(10);
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) !=
        HAL_OK) {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.NbrOfConversion = 6;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_12;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_11;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_16;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_15;
    sConfig.Rank = ADC_REGULAR_RANK_4;
    sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = ADC_REGULAR_RANK_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = ADC_REGULAR_RANK_6;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x10707DBC;
    hi2c1.Init.OwnAddress1 = 36;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) !=
        HAL_OK) {
        Error_Handler();
    }

    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA,
                      WATER_LEVEL_EN_2_Pin | WATER_LEVEL_EN_1_Pin |
                          PUMP_COFFEE_Pin | HEATER_COFFEE_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB,
                      LED1_Pin | LED2_Pin | PUMP_STEAM_Pin | HEATER_STEAM_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pins : WATER_LEVEL_EN_2_Pin WATER_LEVEL_EN_1_Pin
     * PUMP_COFFEE_Pin HEATER_COFFEE_Pin */
    GPIO_InitStruct.Pin = WATER_LEVEL_EN_2_Pin | WATER_LEVEL_EN_1_Pin |
                          PUMP_COFFEE_Pin | HEATER_COFFEE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : SW1_Pin ZERO_CROSS_Pin */
    GPIO_InitStruct.Pin = SW1_Pin | ZERO_CROSS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : SW2_Pin */
    GPIO_InitStruct.Pin = SW2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LED1_Pin LED2_Pin PUMP_STEAM_Pin HEATER_STEAM_Pin */
    GPIO_InitStruct.Pin =
        LED1_Pin | LED2_Pin | PUMP_STEAM_Pin | HEATER_STEAM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//     // Conversion Complete & DMA Transfer Complete As Well
//     //TIM2->CCR1 = (((uint16_t)AD_RES)<<4);
//     //temp = (1-alpha )*temp+alpha*value_adc*factor;
//	temp = temp+1;
// }
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ZERO_CROSS_Pin) {
        coffeeBoiler.firePulse();
        steamBoiler.firePulse();

        coffeePump.firePulse();
        steamPump.firePulse();
    } else {
        __NOP();
    }
}
/**
 * @brief  Tx Transfer completed callback.
 *   I2cHandle: I2C handle.
 * @note   This example shows a simple way to report end of IT Tx transfer, and
 *         you can add your own implementation.
 * @retval None
 */

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
    txcount++;
    HAL_I2C_Slave_Seq_Transmit_IT(I2cHandle, aTxBuffer + txcount, 1,
                                  I2C_NEXT_FRAME);
}

/**
 * @brief  Rx Transfer completed callback.
 *   I2cHandle: I2C handle
 * @note   This example shows a simple way to report end of IT Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
    rxcount++;
    if (rxcount < RxSIZE) {
        if (rxcount == RxSIZE - 1) {
            HAL_I2C_Slave_Seq_Receive_IT(I2cHandle, aRxBuffer + rxcount, 1,
                                         I2C_LAST_FRAME);
        } else {
            HAL_I2C_Slave_Seq_Receive_IT(I2cHandle, aRxBuffer + rxcount, 1,
                                         I2C_NEXT_FRAME);
        }
    }

    if (rxcount == RxSIZE) {

        std::memcpy(&coffeTargetTemp, aRxBuffer, 4);
    }
}

/**
 * @brief   Slave Address Match callback.
 * @param   hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 *   TransferDirection: Master request Transfer Direction (Write/Read), value of
 * @ref I2C_XferOptions_definition AddrMatchCode: Address Match Code
 * @retval None
 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
                          uint16_t AddrMatchCode) {
    if (TransferDirection ==
        I2C_DIRECTION_TRANSMIT) // if the master wants to transmit the data
    {
        rxcount = 0;
        countAddr++;
        // receive using sequential function.
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, aRxBuffer + rxcount, 1,
                                     I2C_FIRST_FRAME);
    } else {
        txcount = 0;
        std::memcpy(aTxBuffer, &buffer, 16);
        if (HAL_I2C_Slave_Seq_Transmit_IT(hi2c, aTxBuffer + txcount, 1,
                                          I2C_NEXT_FRAME) != HAL_OK) {

            Error_Handler();
        }
    }
}

/**
 * @brief   Listen Complete callback.
 * @param   hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_EnableListen_IT(hi2c); // slave is ready again
}

/**
 * @brief  I2C error callbacks.
 *   I2cHandle: I2C handle
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle) {
    /** Error_Handler() function is called when error occurs.
     * 1- When Slave doesn't acknowledge its address, Master restarts
     * communication. 2- When Master doesn't acknowledge the last data
     * transferred, Slave doesn't care in this example.
     */
    uint32_t errorcode = HAL_I2C_GetError(I2cHandle);

    if (errorcode == 4) {
        if (txcount == 0) // error is while slave is receiving
        {
            rxcount = 0; // Reset the rxcount for the next operation
            std::memcpy(&coffeTargetTemp, aRxBuffer, 4);
        } else // error while slave is transmitting
        {
            txcount = 0; // Reset the txcount for the next operation
        }

    }
    /* BERR Error commonly occurs during the Direction switch
     * Here we the software reset bit is set by the HAL error handler
     * Before resetting this bit, we make sure the I2C lines are released and
     * the bus is free I am simply reinitializing the I2C to do so
     */
    else if (errorcode == 1) // BERR Error
    {
        HAL_I2C_DeInit(I2cHandle);
        HAL_I2C_Init(I2cHandle);
        std::memset(aRxBuffer, '\0', RxSIZE); // reset the Rx buffer
        rxcount = 0;                          // reset the count
    }

    else {

        counterror++;
    }

    HAL_I2C_EnableListen_IT(I2cHandle);
}

void reset_gpio_pins() {
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA,
                      WATER_LEVEL_EN_1_Pin | WATER_LEVEL_EN_2_Pin |
                          PUMP_COFFEE_Pin | HEATER_COFFEE_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, HEATER_STEAM_Pin | PUMP_STEAM_Pin, GPIO_PIN_RESET);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    __disable_irq();
    reset_gpio_pins();

    while (1) {
        led1.toggle();
        led2.toggle();
        HAL_Delay(250);
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
