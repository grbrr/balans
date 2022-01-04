/*
 * additives.c
 *
 *  Created on: 3 lis 2021
 *      Author: Damian
 */

#include "additives.h"
//wymagane dla działania getCurrentMicros()
static inline uint32_t LL_SYSTICK_IsActiveCounterFlag(void) {
	return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
			== (SysTick_CTRL_COUNTFLAG_Msk));
}
//funkcja zwracająca liczbę mikrosekund od uruchomiena, pochodzi bodajże z PlatformIO (HAL domyślnie nie posiada odpowiednika)
uint32_t getCurrentMicros(void) {
	/* Ensure COUNTFLAG is reset by reading SysTick control and status register */
	LL_SYSTICK_IsActiveCounterFlag();
	uint32_t m = HAL_GetTick();
	const uint32_t tms = SysTick->LOAD + 1;
	__IO uint32_t u = tms - SysTick->VAL;
	if (LL_SYSTICK_IsActiveCounterFlag()) {
		m = HAL_GetTick();
		u = tms - SysTick->VAL;
	}
	return (m * 1000 + (u * 1000) / tms);
}
//funkcje mapowania wartości
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
		int32_t out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapfloat(float x, float in_min, float in_max, float out_min,
		float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//funkcja przetwarzająca wartość z potencjometru na kąt dla regulatora PID
float potentiometer_value(void) {
	float x = 0.0;
	for (int i = 0; i < 10; i++) {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		x += mapfloat(HAL_ADC_GetValue(&hadc1), 0, 1023, -5, 5);
	}
	x /= 10;
	return x;
}
