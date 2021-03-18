#ifndef __FOC_H
#define __FOC_H

/* _________________________________________________________________________________________________________________*/
/* Include-Files */

#include "stm32l4xx_hal.h"
#include "arm_math.h"
#include "global_config.h"
#include "tim.h"
#include "adc.h"
#include "BSP_SVPWM.h"
#include "BSP_PWM.h"
#include "BSP_LOG_SD.h"
#include "matlab.h"
#include "rtwtypes.h"
#include "BSP_LOG_UART.h"
#include "ErrorHandling.h"
#include "ConfigLE.h"


/* _________________________________________________________________________________________________________________*/
/* Definition von Makros */

/* _________________________________________________________________________________________________________________*/
/* Definition von Konstanten */

/* _________________________________________________________________________________________________________________*/
/* Datentypdefinitionen */

/* _________________________________________________________________________________________________________________*/
/* Funktionsdeklaration */

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);	 

void INIT_FOC(void);
void FOC_loop(uint16_t *ADC_InjectedValues);
void ADC_Injected_Offset_Compensation(uint16_t *ADC_InjectedValues);
void calc_param_controller(float32_t Temp_Motor);
float32_t calc_Motor_Torque(void);

/* _________________________________________________________________________________________________________________*/
/* Definition globaler Variablen */

#endif
