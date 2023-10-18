#ifndef ILI9341_H
#define ILI9341_H

#include "stm32f3xx_hal.h"
#include <stdint.h>




#include "chip8.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"
#include <stdarg.h> //for va_list var arg functions
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 240

struct ILI9341_Pin_t {
  GPIO_TypeDef *port;
  uint16_t pin;
};

struct ILI9341_t {
  struct ILI9341_Pin_t DATA[8];
  struct ILI9341_Pin_t RST, // Reset
                       CS, // Chip Select
                       RS, // Register Select (D/CX), it can be Data (1) or Command (0)
                       WR, // Write
                       RD; // Read
                           //
  UART_HandleTypeDef huart2;
};

enum ILI9341_Orientation {
  HORIZONTAL,
  VERTICAL,
  HORIZONTAL_REVERSE,
  VERTICAL_REVERSE
};

enum Commands {
  CMD_SOFTWARE_RESET = 0x01,
  CMD_READ_DISPLAY_IDENTIFICATION_INFORMATION = 0x04,
  CMD_SLEEP_OUT = 0x11,
  CMD_DISPLAY_INVERSION_ON = 0x21,
  CMD_DISPLAY_INVERSION_OFF = 0x22,
  CMD_DISPLAY_OFF = 0x28,
  CMD_DISPLAY_ON = 0x29,
  CMD_COLUMN_ADDRESS_SET = 0x2a,
  CMD_PAGE_ADDRESS_SET = 0x2b,
  CMD_MEMORY_WRITE = 0x2c,
  CMD_MEMORY_ACCESS_CONTROL = 0x36,
  CMD_PIXEL_FORMAT_SET = 0x3a,
  CMD_FRAME_RATE_CONTROL = 0xb1,
  CMD_ENTRY_MODE_SET = 0xb7,
  CMD_POWER_CONTROL_1 = 0xc0,
  CMD_POWER_CONTROL_2 = 0xc1,
  CMD_VCOM_CONTROL_1 = 0xc5,
  CMD_VCOM_CONTROL_2 = 0xc7,
  CMD_READ_ID4 = 0xd3
};

enum CommandParams {
  PARAM_FLAG_MEMORY_ACCESS_CONTROL_ROW_ADDRESS_ORDER = 0x80,
  PARAM_FLAG_MEMORY_ACCESS_CONTROL_COLUMN_ADDRESS_ORDER = 0x40,
  PARAM_FLAG_MEMORY_ACCESS_CONTROL_ROW_COLUMN_EXCHANGE = 0x20,
  PARAM_FLAG_MEMORY_ACCESS_CONTROL_ROW_BGR = 0x08,
  PARAM_PIXEL_FORMAT_RGB_16_BITS_PER_PIXEL = 0x50,
  PARAM_PIXEL_FORMAT_BGR_16_BITS_PER_PIXEL = 0x05,
  PARAM_FLAG_ENTRY_MODE_LOW_VOLTAGE_DETECTION_DISABLED = 0x1,
  PARAM_FLAG_ENTRY_MODE_NORMAL_DISPLAY = 0x6
};
int ILI9341_Init(struct ILI9341_t *ili, struct ILI9341_Pin_t D7,
                 struct ILI9341_Pin_t D6, struct ILI9341_Pin_t D5,
                 struct ILI9341_Pin_t D4, struct ILI9341_Pin_t D3,
                 struct ILI9341_Pin_t D2, struct ILI9341_Pin_t D1,
                 struct ILI9341_Pin_t D0, struct ILI9341_Pin_t RST,
                 struct ILI9341_Pin_t CS, struct ILI9341_Pin_t RS,
                 struct ILI9341_Pin_t WR, struct ILI9341_Pin_t RD);

void ILI9341_SendInitializationSequence(struct ILI9341_t *ili);

uint32_t ILI9341_ReadID(struct ILI9341_t *ili);

void ILI9341_DrawFramebufferScaled(struct ILI9341_t *ili,
                                   uint16_t framebuffer[]);

void ILI9341_SetDrawingArea(struct ILI9341_t *ili, uint16_t x1, uint16_t x2,
                            uint16_t y1, uint16_t y2);

void ILI9341_StepProgressBar(struct ILI9341_t *ili);

void ILI9341_SetOrientation(struct ILI9341_t *ili, enum ILI9341_Orientation o);

void ILI9341_FillScreen(struct ILI9341_t *ili, uint16_t fill_color);

static inline uint16_t ILI9341_RgbTo565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void PIN_LOW_METAL(struct ILI9341_Pin_t p);
void PIN_HIGH_METAL(struct ILI9341_Pin_t p);
void ILI9341_TestScreen(struct ILI9341_t *ili, unsigned char screen[]);
void ILI9341_TestScreen2(struct ILI9341_t *ili, unsigned char screen[], int scale);
void ILI9341_TestScreenArea(struct ILI9341_t *ili, unsigned char screen[], int scale, uint8_t x, uint8_t y, uint8_t n, uint8_t w);
void ILI9341_TestScreenMetal(struct ILI9341_t *ili, unsigned char screen[], int scale);

#endif
