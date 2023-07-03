#ifndef __UTIL_H__
#define __UTIL_H__

#include <stdio.h>
#include "stm32l4xx_hal.h"


// extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;

// void delay_us (uint16_t us);
void debug_print(const char message[]);

template <class T>
extern void debug_print(const char message[], T& x);

template <class T, class U>
void debug_print(const char message[], const T& x, const U& y);



#endif
