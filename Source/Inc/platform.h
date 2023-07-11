#include <stdint.h>
#include <stdbool.h>

// stm32 of NUCLEO-L476RG
#ifdef STM32L476xx
#include "stm32l4xx_hal.h"
#endif

// stm32 of NUCLEO-F446RE
#ifdef STM32F446xx
#include "stm32f4xx_hal.h"
#endif

// stm32 of the separation board
#ifdef STM32F103xB
#include "stm32f1xx_hal.h"
#endif

// stm32 of the formal board

// ...
