#include <chip8.h>
#include <stdint.h>

void Keypad_Init(Chip8 *vm_);
void Interrupt_handle_keypad(uint16_t GPIO_Pin);
void Polling_handle_keypad(uint16_t GPIO_Pin);
