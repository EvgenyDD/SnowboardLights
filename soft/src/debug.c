#include "debug.h"
#include "main.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

#define CON_OUT_BUF_SZ 512

void debug(char *format, ...)
{
	static char buffer[CON_OUT_BUF_SZ + 1];
	va_list ap;

	va_start(ap, format);
	vsnprintf(buffer, CON_OUT_BUF_SZ, format, ap);
	va_end(ap);

	HAL_UART_Transmit(&huart3, buffer, strlen(buffer), 200);
}
