#ifndef MCU_TC_APP_H
#define MCU_TC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void App_Init(void);
void App_Timer1ms(void);
void App_CanRxPush(uint32_t id, const uint8_t *data);
void App_CanProcess(void);
void App_SetDPTAdcValues(uint16_t ch1, uint16_t ch2);

#ifdef __cplusplus
}
#endif

#endif /* MCU_TC_APP_H */

