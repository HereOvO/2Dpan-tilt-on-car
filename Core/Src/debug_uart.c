#include "debug_uart.h"

#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "usart.h"

static bool DebugUart_IsReady(void)
{
  return (huart1.Instance == USART1);
}

void DebugUart_Write(const uint8_t *data, size_t len)
{
  if ((data == NULL) || (len == 0U) || !DebugUart_IsReady())
  {
    return;
  }

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)data, (uint16_t)len, 100U);
}

void DebugUart_WriteString(const char *str)
{
  if (str == NULL)
  {
    return;
  }

  DebugUart_Write((const uint8_t *)str, strlen(str));
}

void DebugUart_Printf(const char *fmt, ...)
{
  char buffer[256];
  va_list args;
  int len;

  if (fmt == NULL)
  {
    return;
  }

  va_start(args, fmt);
  len = vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  if (len <= 0)
  {
    return;
  }

  if ((size_t)len > sizeof(buffer))
  {
    len = (int)sizeof(buffer);
  }

  DebugUart_Write((const uint8_t *)buffer, (size_t)len);
}

void DebugUart_WriteHex(const uint8_t *data, size_t len)
{
  static const char hex[] = "0123456789ABCDEF";
  size_t i;
  char out[3];

  if (data == NULL)
  {
    return;
  }

  for (i = 0U; i < len; ++i)
  {
    out[0] = hex[(data[i] >> 4) & 0x0F];
    out[1] = hex[data[i] & 0x0F];
    out[2] = ' ';
    DebugUart_Write((const uint8_t *)out, sizeof(out));
  }
}
