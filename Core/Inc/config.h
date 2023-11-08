#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f3xx_hal.h"

// Pin for LCD screen
#define LCD_D0 .port = GPIOA, .pin = GPIO_PIN_9
#define LCD_D1 .port = GPIOC, .pin = GPIO_PIN_7
#define LCD_D2 .port = GPIOA, .pin = GPIO_PIN_10
#define LCD_D3 .port = GPIOB, .pin = GPIO_PIN_3
#define LCD_D4 .port = GPIOB, .pin = GPIO_PIN_5
#define LCD_D5 .port = GPIOB, .pin = GPIO_PIN_4
#define LCD_D6 .port = GPIOB, .pin = GPIO_PIN_10
#define LCD_D7 .port = GPIOA, .pin = GPIO_PIN_8

#define LCD_RD .port = GPIOA, .pin = GPIO_PIN_0
#define LCD_WR .port = GPIOA, .pin = GPIO_PIN_1
#define LCD_RS .port = GPIOA, .pin = GPIO_PIN_4
#define LCD_CS .port = GPIOB, .pin = GPIO_PIN_0
#define LCD_RESET .port = GPIOC, .pin = GPIO_PIN_1

typedef struct Keypad_Pin_t {
  GPIO_TypeDef *port;
  uint16_t pin;
} Keypad_Pin_t;

#define R1 (struct Keypad_Pin_t){ .port = GPIOA, .pin = GPIO_PIN_12 }
#define R2 (struct Keypad_Pin_t){ .port = GPIOA, .pin = GPIO_PIN_11 }
#define R3 (struct Keypad_Pin_t){ .port = GPIOB, .pin = GPIO_PIN_15 }
#define R4 (struct Keypad_Pin_t){ .port = GPIOB, .pin = GPIO_PIN_14 }
#define C1 (struct Keypad_Pin_t){ .port = GPIOB, .pin = GPIO_PIN_12 }
#define C2 (struct Keypad_Pin_t){ .port = GPIOB, .pin = GPIO_PIN_11 }
#define C3 (struct Keypad_Pin_t){ .port = GPIOB, .pin = GPIO_PIN_2 }
#define C4 (struct Keypad_Pin_t){ .port = GPIOB, .pin = GPIO_PIN_1 }

// Pins for keypad
// #define R1 .port = GPIOA, .pin = GPIO_PIN_12
// #define R2 .port = GPIOA, .pin = GPIO_PIN_11
// #define R3 .port = GPIOB, .pin = GPIO_PIN_15
// #define R4 .port = GPIOB, .pin = GPIO_PIN_14
//
// #define C1 .port = GPIOB, .pin = GPIO_PIN_12
// #define C2 .port = GPIOB, .pin = GPIO_PIN_11
// #define C3 .port = GPIOB, .pin = GPIO_PIN_2
// #define C4 .port = GPIOB, .pin = GPIO_PIN_1

// Pin for beeper
#define BEEPER .port = GPIOB, .pin = GPIO_PIN_13


#endif // ndef CONFIG_H
