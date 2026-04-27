
#pragma once

#include "Arduino.h"
#include "WInterrupts.h"
#include "ch32v30x_gpio.h"
#include "wiring_constants.h"
#include "freertos_inc.h"

#if defined(CH32_MCU_FAMILY)

inline void attachInterrupt(uint8_t pin, void (*userFunc)(void), int mode) {
    EXTITrigger_TypeDef m = mode == RISING ? EXTI_Trigger_Rising :
                          mode == FALLING ? EXTI_Trigger_Falling :
                          mode == CHANGE ? EXTI_Trigger_Rising_Falling :
                          EXTI_Trigger_Rising_Falling;
    attachInterrupt(pin, GPIO_Mode_IN_FLOATING, userFunc, EXTI_Mode_Interrupt, m);
}

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t pxTaskCode, const char *const pcName, const uint32_t ulStackDepth, void *const pvParameters, UBaseType_t uxPriority, TaskHandle_t *const pxCreatedTask, const BaseType_t xCoreID) {
    (void) xCoreID;
    return xTaskCreate(pxTaskCode, pcName, ulStackDepth, pvParameters, uxPriority, pxCreatedTask);
}

#define IRAM_ATTR

#else

#endif
