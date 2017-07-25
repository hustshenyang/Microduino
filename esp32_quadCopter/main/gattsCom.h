#ifndef _GATTS_COM_H_
#define _GATTS_COM_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#define GATTS_TAG "GATTS_DEMO"

extern uint16_t bleData[8];
extern uint16_t receiveFlag;
extern SemaphoreHandle_t bleSemaphore;

void gatts_app(void);
void gattsSendData(uint8_t *_data, uint16_t _len);

#endif
