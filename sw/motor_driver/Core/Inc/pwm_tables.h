#ifndef PWM_TABLES_H
#define PWM_TABLES_H

#include <inttypes.h>

#define NUM_OF_TABLES 8

#include "pwm_tab.h"


extern const uint32_t TIM_TRIG_FREQ;

const uint8_t *const pwm_table_ptrs[NUM_OF_TABLES] = {
	pwm67,
	pwm41,
	pwm25,
	pwm15,
	pwm9,
	pwm5,
	pwm3,
	pwm1,
};

const uint32_t pwm_max_freq[NUM_OF_TABLES] = {
	  6 * (1ULL << 16),
	 11 * (1ULL << 16),
	 18 * (1ULL << 16),
	 32 * (1ULL << 16),
	 53 * (1ULL << 16),
	 91 * (1ULL << 16),
	143 * (1ULL << 16),
	UINT32_MAX
};

#endif
