#include "util.h"
#include <string>

//void delay_us (uint16_t us)
//{
//	// TODO: config TIM_Handler
//
//	__HAL_TIM_SET_COUNTER(&htim1,0);  			 // set the counter value a 0
//	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
//}

void debug_print(const char message[])
{
	char uart_buf[100];
	int uart_buf_len;

	// TODO: config UART_Handler

	uart_buf_len = sprintf(uart_buf, message);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
}

void debug_print(const char message[], const char x[])
{
    char uart_buf[100];
	int uart_buf_len;

	// TODO: config UART_Handler

	uart_buf_len = sprintf(uart_buf, message, x);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
}


template <class T>
void debug_print(const char message[], const T& x)
{
	char uart_buf[100];
	int uart_buf_len;

	// TODO: config UART_Handler

	uart_buf_len = sprintf(uart_buf, message, x);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
}

template void debug_print<bool>(const char message[], const bool& x);
template void debug_print<int>(const char message[], const int& x);
template void debug_print<unsigned int>(const char message[], const unsigned int& x);
template void debug_print<float>(const char message[], const float& x);
template void debug_print<double>(const char message[], const double& x);
template void debug_print<char>(const char message[], const char& x);
template void debug_print<int8_t>(const char message[], const int8_t& x);
template void debug_print<uint8_t>(const char message[], const uint8_t& x);
template void debug_print<int16_t>(const char message[], const int16_t& x);
template void debug_print<uint16_t>(const char message[], const uint16_t& x);
template void debug_print<int32_t>(const char message[], const int32_t& x);
template void debug_print<uint32_t>(const char message[], const uint32_t& x);
template void debug_print<HAL_StatusTypeDef>(const char message[], const HAL_StatusTypeDef& x);


template <class T, class U>
void debug_print(const char message[], const T& x, const U& y)
{
	char uart_buf[100];
	int uart_buf_len;

	// TODO: config UART_Handler

	uart_buf_len = sprintf(uart_buf, message, x, y);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
}
template void debug_print<int>(const char message[], const int& x, const int& y);
template void debug_print<float>(const char message[], const float& x, const float& y);
template void debug_print<uint8_t>(const char message[], const uint8_t& x, const uint8_t& y);
template void debug_print<uint16_t>(const char message[], const uint16_t& x, const uint16_t& y);


template <class T, class U, class V>
void debug_print(const char message[], const T& x, const U& y, const V& z)
{
	char uart_buf[150];
	int uart_buf_len;

	// TODO: config UART_Handler

	uart_buf_len = sprintf(uart_buf, message, x, y, z);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
}
template void debug_print<int>(const char message[], const int& x, const int& y, const int& z);
template void debug_print<float>(const char message[], const float& x, const float& y, const float& z);
template void debug_print<uint8_t>(const char message[], const uint8_t& x, const uint8_t& y, const uint8_t& z);
template void debug_print<uint16_t>(const char message[], const uint16_t& x, const uint16_t& y, const uint16_t& z);



