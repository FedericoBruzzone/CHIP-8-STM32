#include "matrix_keypad_4x4.h"
#include "stm32f3xx_hal_gpio.h"
#include <config.h>
#include <stdint.h>
#include <stdio.h>

Chip8 *curr_vm;

void Keypad_Init(Chip8 *vm) { curr_vm = vm; }

// COLUMNS
//   5 pb3 -> pb1
//   6 pb4 -> pb15
//   7 pb5 -> pb14
//   8 pb6 -> pb13

//   1 pa6 -> pa12
//   2 pa7 -> pa11
//   3 pa8 -> pb12
//   4 pa9 -> pb11
void Interrupt_handle_keypad(uint16_t GPIO_Pin) {
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  int currentMillis = HAL_GetTick();
  static int previousMillis = 0;
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
      c8_press_key(curr_vm, 1);
    } else if (GPIO_Pin == R2.pin && HAL_GPIO_ReadPin(R2.port, R2.pin)) {
      c8_press_key(curr_vm, 4);
    } else if (GPIO_Pin == R3.pin && HAL_GPIO_ReadPin(R3.port, R3.pin)) {
      c8_press_key(curr_vm, 7);
    } else if (GPIO_Pin == R4.pin && HAL_GPIO_ReadPin(R4.port, R4.pin)) {
      c8_press_key(curr_vm, 10);
    }

    HAL_GPIO_WritePin(C1.port, C1.pin, 0);
    HAL_GPIO_WritePin(C2.port, C2.pin, 1);
    HAL_GPIO_WritePin(C3.port, C3.pin, 0);
    HAL_GPIO_WritePin(C4.port, C4.pin, 0);

    if (GPIO_Pin == R1.pin && HAL_GPIO_ReadPin(R1.port, R1.pin)) {
      c8_press_key(curr_vm, 2);
    } else if (GPIO_Pin == R2.pin && HAL_GPIO_ReadPin(R2.port, R2.pin)) {
      c8_press_key(curr_vm, 5);
    } else if (GPIO_Pin == R3.pin && HAL_GPIO_ReadPin(R3.port, R3.pin)) {
      c8_press_key(curr_vm, 8);
    } else if (GPIO_Pin == R4.pin && HAL_GPIO_ReadPin(R4.port, R4.pin)) {
      c8_press_key(curr_vm, 0);
    }

    HAL_GPIO_WritePin(C1.port, C1.pin, 0);
    HAL_GPIO_WritePin(C2.port, C2.pin, 0);
    HAL_GPIO_WritePin(C3.port, C3.pin, 1);
    HAL_GPIO_WritePin(C4.port, C4.pin, 0);

    if (GPIO_Pin == R1.pin && HAL_GPIO_ReadPin(R1.port, R1.pin)) {
      c8_press_key(curr_vm, 3);
    } else if (GPIO_Pin == R2.pin && HAL_GPIO_ReadPin(R2.port, R2.pin)) {
      c8_press_key(curr_vm, 6);
    } else if (GPIO_Pin == R3.pin && HAL_GPIO_ReadPin(R3.port, R3.pin)) {
      c8_press_key(curr_vm, 9);
    } else if (GPIO_Pin == R4.pin && HAL_GPIO_ReadPin(R4.port, R4.pin)) {
      c8_press_key(curr_vm, 11);
    }

    HAL_GPIO_WritePin(C1.port, C1.pin, 0);
    HAL_GPIO_WritePin(C2.port, C2.pin, 0);
    HAL_GPIO_WritePin(C3.port, C3.pin, 0);
    HAL_GPIO_WritePin(C4.port, C4.pin, 1);

    if (GPIO_Pin == R1.pin && HAL_GPIO_ReadPin(R1.port, R1.pin)) {
      c8_press_key(curr_vm, 12);
    } else if (GPIO_Pin == R2.pin && HAL_GPIO_ReadPin(R2.port, R2.pin)) {
      c8_press_key(curr_vm, 13);
    } else if (GPIO_Pin == R3.pin && HAL_GPIO_ReadPin(R3.port, R3.pin)) {
      c8_press_key(curr_vm, 14);
    } else if (GPIO_Pin == R4.pin && HAL_GPIO_ReadPin(R4.port, R4.pin)) {
      c8_press_key(curr_vm, 15);
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

    previousMillis = currentMillis;
  }
}

void Polling_handle_keypad(uint16_t GPIO_Pin) {
  int DELAY = 30;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};

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
    c8_press_key(curr_vm, 1);
  } else {
    c8_release_key(curr_vm, 1);
  }
  if (!HAL_GPIO_ReadPin(R2.port, R2.pin)) {
    c8_press_key(curr_vm, 4);
  } else {
    c8_release_key(curr_vm, 4);
  }
  if (!HAL_GPIO_ReadPin(R3.port, R3.pin)) {
    c8_press_key(curr_vm, 7);
  } else {
    c8_release_key(curr_vm, 7);
  }
  if (!HAL_GPIO_ReadPin(R4.port, R4.pin)) {
    c8_press_key(curr_vm, 10);
  } else {
    c8_release_key(curr_vm, 10);
  }

  HAL_GPIO_WritePin(C1.port, C1.pin, 1);
  HAL_GPIO_WritePin(C2.port, C2.pin, 0);
  HAL_GPIO_WritePin(C3.port, C3.pin, 1);
  HAL_GPIO_WritePin(C4.port, C4.pin, 1);

  HAL_Delay(DELAY);

  if (!HAL_GPIO_ReadPin(R1.port, R1.pin)) {
    c8_press_key(curr_vm, 2);
  } else {
    c8_release_key(curr_vm, 2);
  }
  if (!HAL_GPIO_ReadPin(R2.port, R2.pin)) {
    c8_press_key(curr_vm, 5);
  } else {
    c8_release_key(curr_vm, 5);
  }
  if (!HAL_GPIO_ReadPin(R3.port, R3.pin)) {
    c8_press_key(curr_vm, 8);
  } else {
    c8_release_key(curr_vm, 8);
  }
  if (!HAL_GPIO_ReadPin(R4.port, R4.pin)) {
    c8_press_key(curr_vm, 0);
  } else {
    c8_release_key(curr_vm, 0);
  }

  HAL_GPIO_WritePin(C1.port, C1.pin, 1);
  HAL_GPIO_WritePin(C2.port, C2.pin, 1);
  HAL_GPIO_WritePin(C3.port, C3.pin, 0);
  HAL_GPIO_WritePin(C4.port, C4.pin, 1);

  HAL_Delay(DELAY);
  if (!HAL_GPIO_ReadPin(R1.port, R1.pin)) {
    c8_press_key(curr_vm, 3);
  } else {
    c8_release_key(curr_vm, 3);
  }
  if (!HAL_GPIO_ReadPin(R2.port, R2.pin)) {
    c8_press_key(curr_vm, 6);
  } else {
    c8_release_key(curr_vm, 6);
  }
  if (!HAL_GPIO_ReadPin(R3.port, R3.pin)) {
    c8_press_key(curr_vm, 9);
  } else {
    c8_release_key(curr_vm, 9);
  }
  if (!HAL_GPIO_ReadPin(R4.port, R4.pin)) {
    c8_press_key(curr_vm, 11);
  } else {
    c8_release_key(curr_vm, 11);
  }

  HAL_GPIO_WritePin(C1.port, C1.pin, 1);
  HAL_GPIO_WritePin(C2.port, C2.pin, 1);
  HAL_GPIO_WritePin(C3.port, C3.pin, 1);
  HAL_GPIO_WritePin(C4.port, C4.pin, 0);

  HAL_Delay(DELAY);

  if (!HAL_GPIO_ReadPin(R1.port, R1.pin)) {
    c8_press_key(curr_vm, 12);
  } else {
    c8_release_key(curr_vm, 12);
  }
  if (!HAL_GPIO_ReadPin(R2.port, R2.pin)) {
    c8_press_key(curr_vm, 13);
  } else {
    c8_release_key(curr_vm, 13);
  }
  if (!HAL_GPIO_ReadPin(R3.port, R3.pin)) {
    c8_press_key(curr_vm, 14);
  } else {
    c8_release_key(curr_vm, 14);
  }
  if (!HAL_GPIO_ReadPin(R4.port, R4.pin)) {
    c8_press_key(curr_vm, 15);
  } else {
    c8_release_key(curr_vm, 15);
  }

  HAL_GPIO_WritePin(C1.port, C1.pin, 0);
  HAL_GPIO_WritePin(C2.port, C2.pin, 0);
  HAL_GPIO_WritePin(C3.port, C3.pin, 0);
  HAL_GPIO_WritePin(C4.port, C4.pin, 0);
}
