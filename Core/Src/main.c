/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chip8.h"
#include "config.h"
#include "font.h"
#include "ili9341.h"
#include "matrix_keypad_4x4.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"
#include "utils.h"
#include <stdarg.h> //for va_list var arg functions
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
Chip8 vm;
struct ILI9341_t lcd;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  Uart_Init(huart2);
  Keypad_Init(&vm);
  ILI9341_Init(&lcd, (struct ILI9341_Pin_t){LCD_D7},
               (struct ILI9341_Pin_t){LCD_D6}, (struct ILI9341_Pin_t){LCD_D5},
               (struct ILI9341_Pin_t){LCD_D4}, (struct ILI9341_Pin_t){LCD_D3},
               (struct ILI9341_Pin_t){LCD_D2}, (struct ILI9341_Pin_t){LCD_D1},
               (struct ILI9341_Pin_t){LCD_D0},

               (struct ILI9341_Pin_t){LCD_RESET},
               (struct ILI9341_Pin_t){LCD_CS}, (struct ILI9341_Pin_t){LCD_RS},
               (struct ILI9341_Pin_t){LCD_WR}, (struct ILI9341_Pin_t){LCD_RD});
  ILI9341_SendInitializationSequence(&lcd);
  ILI9341_SetOrientation(&lcd, HORIZONTAL);
  ILI9341_FillScreen(&lcd, ILI9341_RgbTo565(255, 0, 0));
  // ========== SD
  Uart_print("\r\n~ SD card demo by kiwih ~\r\n\r\n");

  HAL_Delay(1000); // a short delay is important to let the SD card settle

  // some variables for FatFs
  FATFS FatFs;  // Fatfs handle
  FIL fil;      // File handle
  FRESULT fres; // Result after operations

  // Open the file system
  fres = f_mount(&FatFs, "", 1); // 1=mount now
  if (fres != FR_OK) {
    Uart_print("f_mount error (%i)\r\n", fres);
    while (1)
      ;
  }

  DIR dir;
  char *path;
  char string[128];
  FRESULT res;
  // char *games[60];
  char games[60][13];
  int max_g_size = 0;
  int g_size = 0;

  path = "";

  res = f_opendir(&dir, path);

  if (res == FR_OK) {
    while (1) {
      FILINFO fno;

      res = f_readdir(&dir, &fno);

      if ((res != FR_OK) || (fno.fname[0] == 0))
        break;

      strcpy(games[g_size], fno.fname);
      g_size++;
    }
  }

  char *modes[5] = {"CHIP8", "SCH10", "SCH11"};
  char *freqs[9] = {"  600", " 1200", " 1800", " 2400", " 4800", " 9600", "19200", "38400", "99999"};

  int i_modes = 0;
  int i_freqs = 0;
  int pages = 0;
  int game = -1;

  for (int i = 0; i < 16; i++) {
    c8_release_key(&vm, i);
  }

  while (game == -1) {
    ILI9341_WriteMenu_array(&lcd, 10, 95, &font, 8, 10, g_size, &games, pages,
                            modes[i_modes], freqs[i_freqs]);

    for (int i = 0; i < 16; i++) {
      if (vm.keypad[i]) {
        switch (i) {
        case 0:
          i_modes = (i_modes + 1) % 3;
          break;
        case 7:
          if (pages > 0) {
            pages = pages - 1;
            ILI9341_FillScreen(&lcd, ILI9341_RgbTo565(255, 0, 0));
          }
          break;
        case 8:
          i_freqs = (i_freqs + 1) % 9;
          break;
        case 15:
          if (pages < g_size / 12) {
            pages = pages + 1;
            ILI9341_FillScreen(&lcd, ILI9341_RgbTo565(255, 0, 0));
          }
          break;
        default:
          if (i >= 1 && i <= 6) {
            game = i - 1 + pages * 12;
          } else {
            game = i - 9 + 6 + pages * 12;
          }
          break;
        }
        c8_release_key(&vm, i);
      }
    }
  }

  int emu_freq = atoi(freqs[i_freqs]);
  c8_init(&vm, emu_freq, i_modes, HAL_GetTick());
  c8_set_is_polling(&vm, false);

  fres = f_open(&fil, games[game], FA_READ);
  if (fres != FR_OK) {
    Uart_print("f_open error (%i)\r\n", fres);
    while (1)
      ;
  }
  UINT bytesRead;
  uint16_t size = f_size(&fil);
  unsigned char *data = NULL;
  data = malloc(size);
  FRESULT tmp = f_read(&fil, data, (UINT)size, &bytesRead);

  if (tmp != FR_OK) {
    Uart_print("Error reading file: %i\n", tmp);
    while (1)
      ;
  }
  if (bytesRead == 0) { // EOF
    Uart_print("Error reading file: 0 bytes read\n");
    while (1)
      ;
  }
  if (bytesRead != (UINT)size) {
    Uart_print("Error reading file: %i bytes read instead of %i\n", bytesRead,
               size);
    while (1)
      ;
  }

  ILI9341_FillScreen(&lcd, ILI9341_RgbTo565(0, 255, 0));
  f_close(&fil);

  memcpy(&vm.RAM[PC_OFFSET], data, size);
  free(data);

  int i = 0;
  int c_beeper = 0;

  for (int i = 0; i < KEYPAD_SIZE; i++)
    c8_release_key(&vm, i);

  while (1) {
    uint64_t start = HAL_GetTick();
    c_beeper++;
    if (c8_ended(&vm))
      return 1;
    if (c8_cycle(&vm) != 0)
      goto unknown_opcode;
    c8_decrement_timers(&vm);
    if (c8_screen_updated(&vm)) {
      ILI9341_PrintScreenMetal(&lcd, vm.screen, 2);
    }

    if (c8_sound(&vm)) {
      //      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
      PIN_HIGH_METAL((struct ILI9341_Pin_t){BEEPER});
    } else if (!c8_sound(&vm) && c_beeper == 60) {
      //    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
      PIN_LOW_METAL((struct ILI9341_Pin_t){BEEPER});
      c_beeper = 0;
    }

    uint64_t performance_freq = HAL_RCC_GetHCLKFreq();
    uint64_t end = HAL_GetTick();
    double elapsed_time = ((end - start) * 1000) / performance_freq;
    if (elapsed_time < GAME_LOOP_DELAY) {
      HAL_Delay((uint32_t)(GAME_LOOP_DELAY - elapsed_time + 0.5));
    }
  }

unknown_opcode:
  Uart_print("Unknown opcode: 0x%04X\n", vm.opcode);
  return 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(
      GPIOA, GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10,
      GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB,
                    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_11 |
                        GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_3 | GPIO_PIN_4 |
                        GPIO_PIN_5 | SD_CS_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 PA8 PA9
                           PA10 */
  GPIO_InitStruct.Pin =
      GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB11
                           PB12 PB13 PB3 PB4
                           PB5 SD_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_11 |
                        GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_3 | GPIO_PIN_4 |
                        GPIO_PIN_5 | SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ===== Keypad
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  Interrupt_handle_keypad(GPIO_Pin);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return
   * state
   */
  __disable_irq();
  while (1) {
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
