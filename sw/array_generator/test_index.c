#include <inttypes.h>
#include <stdio.h>

#include "pwm_tab.c"


int main(void) {
	for (uint16_t i = 0; i < 16000; i += 1) {
		if (!(i % PERIOD)) {
			printf("\n");
		}

		const uint32_t invert = i & (PERIOD / 2);
		const uint32_t reverse = i & (PERIOD / 4);

		uint32_t idx = i & (PERIOD / 4 - 1);
		if (reverse) {
			idx = PERIOD / 4 - idx;
		}

		if (invert) {
			printf("-");
		}
		printf("%"PRId32" ", pwm5[idx]);
	}

	printf("\n");

	return 0;
}
