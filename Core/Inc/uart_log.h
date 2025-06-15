#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include "stm32f10x.h"  // Hoặc stm32f4xx.h nếu dùng STM32F4

// Khởi tạo UART
void uart_init(void);

// Gửi chuỗi ký tự (null-terminated string) qua UART
void uart_send(char *s);

#endif