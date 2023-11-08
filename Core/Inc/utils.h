#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// void uart_print(const char *fmt, ...);

UART_HandleTypeDef curr_huart2;

void Uart_Init(UART_HandleTypeDef huart2) { curr_huart2 = huart2; }

void Uart_print(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&curr_huart2, (uint8_t *)buffer, len, -1);
}
