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
#include "ili9341.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"
#include <stdarg.h> //for va_list var arg functions
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_D0 .port = GPIOA, .pin = GPIO_PIN_9
#define LCD_D1 .port = GPIOC, .pin = GPIO_PIN_7
#define LCD_D2 .port = GPIOA, .pin = GPIO_PIN_10
#define LCD_D3 .port = GPIOB, .pin = GPIO_PIN_3
#define LCD_D4 .port = GPIOB, .pin = GPIO_PIN_5
#define LCD_D5 .port = GPIOB, .pin = GPIO_PIN_4
#define LCD_D6 .port = GPIOB, .pin = GPIO_PIN_10
#define LCD_D7 .port = GPIOA, .pin = GPIO_PIN_8

#define LCD_RD .port = GPIOA, .pin = GPIO_PIN_0 // INPUT
#define LCD_WR .port = GPIOA, .pin = GPIO_PIN_1
#define LCD_RS .port = GPIOA, .pin = GPIO_PIN_4
#define LCD_CS .port = GPIOB, .pin = GPIO_PIN_0
#define LCD_RESET .port = GPIOC, .pin = GPIO_PIN_1

#define BEEPER .port = GPIOB, .pin = GPIO_PIN_13
Chip8 vm;
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
void myprintf(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, -1);
}

// ===== Keypad
GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint8_t pressed_key = 0;

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
  // ========== Display
  struct ILI9341_t lcd;
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
  // ILI9341_FillScreen(&lcd, ILI9341_RgbTo565(50, 50, 50));
  ILI9341_FillScreen(&lcd, ILI9341_RgbTo565(255, 0, 0));
  // ILI9341_FillScreen(&lcd, ILI9341_RgbTo565(0, 255, 0));
  // ILI9341_FillScreen(&lcd, ILI9341_RgbTo565(0, 0, 255));
  // ========== SD
  myprintf("\r\n~ SD card demo by kiwih ~\r\n\r\n");

  HAL_Delay(1000); // a short delay is important to let the SD card settle

  // some variables for FatFs
  FATFS FatFs;  // Fatfs handle
  FIL fil;      // File handle
  FRESULT fres; // Result after operations

  // Open the file system
  fres = f_mount(&FatFs, "", 1); // 1=mount now
  if (fres != FR_OK) {
    myprintf("f_mount error (%i)\r\n", fres);
    while (1)
      ;
  }
  // =========================== LS
  DIR dir;
  char *path;
  char string[128];
  FRESULT res;

  path = ""; // where you want to list

  res = f_opendir(&dir, path);

  if (res == FR_OK) {
    while (1) {
      FILINFO fno;

      res = f_readdir(&dir, &fno);

      if ((res != FR_OK) || (fno.fname[0] == 0))
        break;

      sprintf(string, "%c%c%c%c %10d %s/%s\r\n",
              ((fno.fattrib & AM_DIR) ? 'D' : '-'),
              ((fno.fattrib & AM_RDO) ? 'R' : '-'),
              ((fno.fattrib & AM_SYS) ? 'S' : '-'),
              ((fno.fattrib & AM_HID) ? 'H' : '-'), (int)fno.fsize, path,
              fno.fname);

      myprintf(string);
      // puts(string);
    }
  }
  // =========================================================
  /*
  //Let's get some statistics from the SD card

  DWORD free_clusters, free_sectors, total_sectors;

  FATFS* getFreeFs;

  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK) {
  myprintf("f_getfree error (%i)\r\n", fres);
  while(1);
  }

  //Formula comes from ChaN's documentation
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB
  available.\r\n", total_sectors / 2, free_sectors / 2);
  */

  // Now let's try to open file "test.txt"
  // fres = f_open(&fil, "1-CHIP~1.CH8", FA_READ);
  // fres = f_open(&fil, "6-KEYPAD.CH8", FA_READ);
  // fres = f_open(&fil, "Tetris~1.CH8", FA_READ);
  // fres = f_open(&fil, "SCHIP_~1.CH8", FA_READ);
  fres = f_open(&fil, "SpaceI~1.CH8", FA_READ);

  if (fres != FR_OK) {
    myprintf("f_open error (%i)\r\n");
    while (1)
      ;
  }
  myprintf("I was able to open 'test.txt' for reading!\r\n");

  // We can either use f_read OR f_gets to get data out of files
  // f_gets is a wrapper on f_read that does some string formatting for us
  //	    TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
  //    if(rres != 0) {
  //  	myprintf("Read string from 'test.txt' contents: %s\r\n", readBuf);
  //    } else {
  //  	myprintf("f_gets error (%i)\r\n", fres);
  //    }

  // --------------------------------------------------------
  unsigned char *data = NULL;

  UINT bytesRead;
  uint16_t size = f_size(&fil);
  data = malloc(size);
  FRESULT tmp = f_read(&fil, data, (UINT)size, &bytesRead);

  if (tmp == FR_OK) {
    myprintf("File successfully read!");
    myprintf("%d bytes read", bytesRead);
    myprintf("\r\n");
    for (int i = 0; i < bytesRead; i++) {
      //    		myprintf("data[%d]: 0x%c \r\n", i, data[i]);
      myprintf("%x ", data[i]);
    }

  } else {
    myprintf("f_gets error (%i)\r\n", fres);
  }
  // Be a tidy kiwi - don't forget to close your file!
  f_close(&fil);

  // ========================== HADLE CHIP8
  // ./chip8 10 1200 ./ROMs/games/ALIEN
  myprintf("\n\r========================== HADLE CHIP8\r\n");
  // Chip8 vm;
  int emu_freq = 1200; // 72000;               // 1200;
  c8_init(&vm, emu_freq, P_CHIP8, HAL_GetTick());
  c8_set_is_polling(&vm, false);

  //	for (int i=0; i < size; i++){
  //		vm.RAM[PC_OFFSET+i] = data[i];
  //	}

  memcpy(&vm.RAM[PC_OFFSET], data, size);
  free(data);
  // for (int i = 0; i < MAX_ROM_SIZE; i++) {
  //   myprintf("%x ", vm.RSTM32F303x4-x6-xAM[i]);
  // }

  // int RX = 32;
  // int RY = 52;
  // int CX = SCREEN_WIDTH * 2 - 1 + RX;
  // int CY = SCREEN_HEIGHT * 2 - 1 + RY;
  // ILI9341_SetDrawingArea(&lcd, RX, CX, RY, CY);

  // BEEPER
  // while (1) {
  //   HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
  //   HAL_Delay(100);
  // }

  int i = 0;
  while (1) {
    uint64_t start = HAL_GetTick();

    // HandleKeyPad();
    if (c8_ended(&vm))
      return 1;
    if (c8_cycle(&vm) != 0)
      goto unknown_opcode;
    // UART print
    // if (i == 1000) {
    //   myprintf("\r\n");
    //   for (int i = 0; i < SCREEN_SIZE; i++) {
    //     for (int j = 0; j < 8; j++) {
    //       if (!!((vm.screen[i] << j) & 0x80) == 1)
    //         myprintf("█");
    //       else
    //         myprintf(" ");
    //     }
    //     if ((i + 1) % (SCREEN_WIDTH / 8) == 0)
    //       myprintf("\r\n");
    //   }
    // }
    c8_decrement_timers(&vm);
    if (c8_screen_updated(&vm)) {
      // ILI9341_TestScreen(&lcd, vm.screen);
      // ILI9341_TestScreen2(&lcd, vm.screen, 2);
      ILI9341_TestScreenMetal(&lcd, vm.screen, 2);

      // char oc[16];
      // sprintf(oc, "%x\n\r", vm.opcode);
      // myprintf(oc);

      // if ((vm.opcode & 0xF000) == 0xD000) {
      //   myprintf("NULL");
      //   uint8_t x = (vm.opcode & 0x0F00) >> 8;
      //   uint8_t y = (vm.opcode & 0x00F0) >> 4;
      //   uint8_t n = vm.opcode & 0x000F;
      //   ILI9341_TestScreenArea(&lcd, vm.screen, 2, x, y, n, 8);
      // } else {
      //   ILI9341_TestScreen2(&lcd, vm.screen, 2);
      // }
    }
    if (c8_sound(&vm)) {
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
      PIN_HIGH_METAL((struct ILI9341_Pin_t){BEEPER});
    } else {
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
      PIN_LOW_METAL((struct ILI9341_Pin_t){BEEPER});
    }

    // if (i % 3 == 0) {
    //   for (int x = 0; x < 16; x++) {
    //     c8_release_key(&vm, x);
    //   }
    // }
    uint64_t performance_freq = HAL_RCC_GetHCLKFreq();
    uint64_t end = HAL_GetTick();
    double elapsed_time = ((end - start) * 1000) / performance_freq;
    if (elapsed_time < GAME_LOOP_DELAY) {
      // char oc[16];
      // sprintf(oc, "%lf\n\r", elapsed_time);
      // myprintf(oc);

      HAL_Delay((uint32_t)(GAME_LOOP_DELAY - elapsed_time + 0.5));
    }
    // i++;
  }

unknown_opcode:
  myprintf("Unknown opcode: 0x%04X\n", vm.opcode);
  return 1;

  // ======================================
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

typedef struct Keypad_Pin_t {
  GPIO_TypeDef *port;
  uint16_t pin;
} Keypad_Pin_t;
// ROWS (EXTI)
Keypad_Pin_t R1 = {.port = GPIOA, .pin = GPIO_PIN_12};
Keypad_Pin_t R2 = {.port = GPIOA, .pin = GPIO_PIN_11};
Keypad_Pin_t R3 = {.port = GPIOB, .pin = GPIO_PIN_15};
Keypad_Pin_t R4 = {.port = GPIOB, .pin = GPIO_PIN_14};

Keypad_Pin_t C1 = {.port = GPIOB, .pin = GPIO_PIN_12};
Keypad_Pin_t C2 = {.port = GPIOB, .pin = GPIO_PIN_11};
Keypad_Pin_t C3 = {.port = GPIOB, .pin = GPIO_PIN_2};
Keypad_Pin_t C4 = {.port = GPIOB, .pin = GPIO_PIN_1};

// POLLING row in input and column in output
void HandleKeyPad() {
  int DELAY = 30;

  GPIO_InitStructPrivate.Pin = R1.pin | R2.pin;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(R1.port, &GPIO_InitStructPrivate);

  GPIO_InitStructPrivate.Pin = R3.pin | R4.pin;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(R3.port, &GPIO_InitStructPrivate);

  HAL_GPIO_WritePin(C1.port, C1.pin, 0);
  HAL_GPIO_WritePin(C2.port, C2.pin, 1);
  HAL_GPIO_WritePin(C3.port, C3.pin, 1);
  HAL_GPIO_WritePin(C4.port, C4.pin, 1);

  HAL_Delay(DELAY);

  if (!HAL_GPIO_ReadPin(R1.port, R1.pin)) {
    // myprintf("1");
    c8_press_key(&vm, 1);
  } else {
    c8_release_key(&vm, 1);
  }
  if (!HAL_GPIO_ReadPin(R2.port, R2.pin)) {
    // myprintf("4");
    c8_press_key(&vm, 4);
  } else {
    c8_release_key(&vm, 4);
  }
  if (!HAL_GPIO_ReadPin(R3.port, R3.pin)) {
    // myprintf("7");
    c8_press_key(&vm, 7);
  } else {
    c8_release_key(&vm, 7);
  }
  if (!HAL_GPIO_ReadPin(R4.port, R4.pin)) {
    // myprintf("A");
    c8_press_key(&vm, 10);
  } else {
    c8_release_key(&vm, 10);
  }

  HAL_GPIO_WritePin(C1.port, C1.pin, 1);
  HAL_GPIO_WritePin(C2.port, C2.pin, 0);
  HAL_GPIO_WritePin(C3.port, C3.pin, 1);
  HAL_GPIO_WritePin(C4.port, C4.pin, 1);

  HAL_Delay(DELAY);

  if (!HAL_GPIO_ReadPin(R1.port, R1.pin)) {
    // myprintf("2");
    c8_press_key(&vm, 2);
  } else {
    c8_release_key(&vm, 2);
  }
  if (!HAL_GPIO_ReadPin(R2.port, R2.pin)) {
    // myprintf("5");
    c8_press_key(&vm, 5);
  } else {
    c8_release_key(&vm, 5);
  }
  if (!HAL_GPIO_ReadPin(R3.port, R3.pin)) {
    // myprintf("8");
    c8_press_key(&vm, 8);
  } else {
    c8_release_key(&vm, 8);
  }
  if (!HAL_GPIO_ReadPin(R4.port, R4.pin)) {
    // myprintf("0");
    c8_press_key(&vm, 0);
  } else {
    c8_release_key(&vm, 0);
  }

  HAL_GPIO_WritePin(C1.port, C1.pin, 1);
  HAL_GPIO_WritePin(C2.port, C2.pin, 1);
  HAL_GPIO_WritePin(C3.port, C3.pin, 0);
  HAL_GPIO_WritePin(C4.port, C4.pin, 1);

  HAL_Delay(DELAY);
  if (!HAL_GPIO_ReadPin(R1.port, R1.pin)) {
    // myprintf("3");
    c8_press_key(&vm, 3);
  } else {
    c8_release_key(&vm, 3);
  }
  if (!HAL_GPIO_ReadPin(R2.port, R2.pin)) {
    // myprintf("6");
    c8_press_key(&vm, 6);
  } else {
    c8_release_key(&vm, 6);
  }
  if (!HAL_GPIO_ReadPin(R3.port, R3.pin)) {
    // myprintf("9");
    c8_press_key(&vm, 9);
  } else {
    c8_release_key(&vm, 9);
  }
  if (!HAL_GPIO_ReadPin(R4.port, R4.pin)) {
    // myprintf("B");
    c8_press_key(&vm, 11);
  } else {
    c8_release_key(&vm, 11);
  }

  HAL_GPIO_WritePin(C1.port, C1.pin, 1);
  HAL_GPIO_WritePin(C2.port, C2.pin, 1);
  HAL_GPIO_WritePin(C3.port, C3.pin, 1);
  HAL_GPIO_WritePin(C4.port, C4.pin, 0);

  HAL_Delay(DELAY);

  if (!HAL_GPIO_ReadPin(R1.port, R1.pin)) {
    // myprintf("C");
    c8_press_key(&vm, 12);
  } else {
    c8_release_key(&vm, 12);
  }
  if (!HAL_GPIO_ReadPin(R2.port, R2.pin)) {
    // myprintf("D");
    c8_press_key(&vm, 13);
  } else {
    c8_release_key(&vm, 13);
  }
  if (!HAL_GPIO_ReadPin(R3.port, R3.pin)) {
    // myprintf("E");
    c8_press_key(&vm, 14);
  } else {
    c8_release_key(&vm, 14);
  }
  if (!HAL_GPIO_ReadPin(R4.port, R4.pin)) {
    // myprintf("F");
    c8_press_key(&vm, 15);
  } else {
    c8_release_key(&vm, 15);
  }

  HAL_GPIO_WritePin(C1.port, C1.pin, 0);
  HAL_GPIO_WritePin(C2.port, C2.pin, 0);
  HAL_GPIO_WritePin(C3.port, C3.pin, 0);
  HAL_GPIO_WritePin(C4.port, C4.pin, 0);
}

// COLUMNS
//   5 pb3 -> pb1
//   6 pb4 -> pb15
//   7 pb5 -> pb14
//   8 pb6 -> pb13

//   1 pa6 -> pa12
//   2 pa7 -> pa11
//   3 pa8 -> pb12
//   4 pa9 -> pb11

// ===== Keypad
// INTERRUPT
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  currentMillis = HAL_GetTick();
  if (currentMillis - previousMillis > 100) {
    /*Configure GPIO pins : PB6 PB7 PB8 PB9 to GPIO_INPUT*/
    GPIO_InitStructPrivate.Pin = R1.pin | R2.pin;
    GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
    GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(R1.port, &GPIO_InitStructPrivate);

    GPIO_InitStructPrivate.Pin = R3.pin | R4.pin;
    GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
    GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(R3.port, &GPIO_InitStructPrivate);

    HAL_GPIO_WritePin(C1.port, C1.pin, 1);
    HAL_GPIO_WritePin(C2.port, C2.pin, 0);
    HAL_GPIO_WritePin(C3.port, C3.pin, 0);
    HAL_GPIO_WritePin(C4.port, C4.pin, 0);

    if (GPIO_Pin == R1.pin && HAL_GPIO_ReadPin(R1.port, R1.pin)) {
      pressed_key = 35; // ASCII value of #
      myprintf("1");
      c8_press_key(&vm, 1);
    } else if (GPIO_Pin == R2.pin && HAL_GPIO_ReadPin(R2.port, R2.pin)) {
      pressed_key = 57; // ASCII value of 9
      myprintf("4");
      c8_press_key(&vm, 4);
    } else if (GPIO_Pin == R3.pin && HAL_GPIO_ReadPin(R3.port, R3.pin)) {
      pressed_key = 54; // ASCII value of 6
      myprintf("7");
      c8_press_key(&vm, 7);
    } else if (GPIO_Pin == R4.pin && HAL_GPIO_ReadPin(R4.port, R4.pin)) {
      pressed_key = 51; // ASCII value of 3
      myprintf("A");
      c8_press_key(&vm, 10);
    }

    HAL_GPIO_WritePin(C1.port, C1.pin, 0);
    HAL_GPIO_WritePin(C2.port, C2.pin, 1);
    HAL_GPIO_WritePin(C3.port, C3.pin, 0);
    HAL_GPIO_WritePin(C4.port, C4.pin, 0);

    if (GPIO_Pin == R1.pin && HAL_GPIO_ReadPin(R1.port, R1.pin)) {
      pressed_key = 35; // ASCII value of #
      myprintf("2");
      c8_press_key(&vm, 2);
    } else if (GPIO_Pin == R2.pin && HAL_GPIO_ReadPin(R2.port, R2.pin)) {
      pressed_key = 57; // ASCII value of 9
      myprintf("5");
      c8_press_key(&vm, 5);
    } else if (GPIO_Pin == R3.pin && HAL_GPIO_ReadPin(R3.port, R3.pin)) {
      pressed_key = 54; // ASCII value of 6
      myprintf("8");
      c8_press_key(&vm, 8);
    } else if (GPIO_Pin == R4.pin && HAL_GPIO_ReadPin(R4.port, R4.pin)) {
      pressed_key = 51; // ASCII value of 3
      myprintf("0");
      c8_press_key(&vm, 0);
    }

    HAL_GPIO_WritePin(C1.port, C1.pin, 0);
    HAL_GPIO_WritePin(C2.port, C2.pin, 0);
    HAL_GPIO_WritePin(C3.port, C3.pin, 1);
    HAL_GPIO_WritePin(C4.port, C4.pin, 0);

    if (GPIO_Pin == R1.pin && HAL_GPIO_ReadPin(R1.port, R1.pin)) {
      pressed_key = 35; // ASCII value of #
      myprintf("3");
      c8_press_key(&vm, 3);
    } else if (GPIO_Pin == R2.pin && HAL_GPIO_ReadPin(R2.port, R2.pin)) {
      pressed_key = 57; // ASCII value of 9
      myprintf("6");
      c8_press_key(&vm, 6);
    } else if (GPIO_Pin == R3.pin && HAL_GPIO_ReadPin(R3.port, R3.pin)) {
      pressed_key = 54; // ASCII value of 6
      myprintf("9");
      c8_press_key(&vm, 9);
    } else if (GPIO_Pin == R4.pin && HAL_GPIO_ReadPin(R4.port, R4.pin)) {
      pressed_key = 51; // ASCII value of 3
      myprintf("B");
      c8_press_key(&vm, 11);
    }

    HAL_GPIO_WritePin(C1.port, C1.pin, 0);
    HAL_GPIO_WritePin(C2.port, C2.pin, 0);
    HAL_GPIO_WritePin(C3.port, C3.pin, 0);
    HAL_GPIO_WritePin(C4.port, C4.pin, 1);

    if (GPIO_Pin == R1.pin && HAL_GPIO_ReadPin(R1.port, R1.pin)) {
      pressed_key = 35; // ASCII value of #
      myprintf("C");
      c8_press_key(&vm, 12);
    } else if (GPIO_Pin == R2.pin && HAL_GPIO_ReadPin(R2.port, R2.pin)) {
      pressed_key = 57; // ASCII value of 9
      myprintf("D");
      c8_press_key(&vm, 13);
    } else if (GPIO_Pin == R3.pin && HAL_GPIO_ReadPin(R3.port, R3.pin)) {
      pressed_key = 54; // ASCII value of 6
      myprintf("E");
      c8_press_key(&vm, 14);
    } else if (GPIO_Pin == R4.pin && HAL_GPIO_ReadPin(R4.port, R4.pin)) {
      pressed_key = 51; // ASCII value of 3
      myprintf("F");
      c8_press_key(&vm, 15);
    }

    HAL_GPIO_WritePin(C1.port, C1.pin, 1);
    HAL_GPIO_WritePin(C2.port, C2.pin, 1);
    HAL_GPIO_WritePin(C3.port, C3.pin, 1);
    HAL_GPIO_WritePin(C4.port, C4.pin, 1);

    GPIO_InitStructPrivate.Pin = R1.pin | R2.pin;
    GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructPrivate.Pull = GPIO_PULLDOWN;
    GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(R1.port, &GPIO_InitStructPrivate);

    GPIO_InitStructPrivate.Pin = R3.pin | R4.pin;
    GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructPrivate.Pull = GPIO_PULLDOWN;
    GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(R3.port, &GPIO_InitStructPrivate);

    //		GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING;
    //		GPIO_InitStructPrivate.Pull = GPIO_PULLDOWN;
    //		HAL_GPIO_Init(GPIOA, &GPIO_InitStructPrivate);

    // for (int i = 0; i < 16; i++) {
    //   myprintf("key[%d]: %d \r\n", i, vm.keypad[i]);
    // }

    previousMillis = currentMillis;
  }
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
