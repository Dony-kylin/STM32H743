#ifndef __SCOPE_APP_H__
#define __SCOPE_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * USART1 is currently unused.  Keep one compile-time switch so it can be
 * restored for bench diagnostics without touching the acquisition path.
 */
#define SCOPE_APP_UART_ENABLED 0U

void ScopeApp_Init(void);
void ScopeApp_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __SCOPE_APP_H__ */
