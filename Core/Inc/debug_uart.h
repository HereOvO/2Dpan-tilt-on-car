#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

void DebugUart_Write(const uint8_t *data, size_t len);
void DebugUart_WriteString(const char *str);
void DebugUart_Printf(const char *fmt, ...);
void DebugUart_WriteHex(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif
