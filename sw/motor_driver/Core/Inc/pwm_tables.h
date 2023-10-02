#ifndef PWM_TABLES_H
#define PWM_TABLES_H

#include <inttypes.h>

#define NUM_OF_TABLES 8

#include "pwm_1.txt"
#include "pwm_3.txt"
#include "pwm_5.txt"
#include "pwm_9.txt"
#include "pwm_15.txt"
#include "pwm_27.txt"
#include "pwm_45.txt"
#include "pwm_81.txt"

extern const uint32_t TIM_TRIG_FREQ;

const uint8_t *pwm_table_ptrs[NUM_OF_TABLES] = {
		pwm_81,
		pwm_45,
		pwm_27,
		pwm_15,
		pwm_9,
		pwm_5,
		pwm_3,
		pwm_1,
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
