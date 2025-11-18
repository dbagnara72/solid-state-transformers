#include <dsmavgflt.h>

void dsmavgflt_init(volatile DSMAVGFLT *f, volatile float ts) {
	int i;
	f->ts = ts;

	f->sum_value = 0.0;
	f->idx = 0;
	f->length_period = DSMAVGFLT_SIZE_MAX;
	for (i = 0; i < DSMAVGFLT_SIZE_MAX; i++) {
		f->buffer[i] = 0.0;
	}
}

// ------------------------------------------------------------------------------

float dsmavgflt_process(volatile DSMAVGFLT *f, float input, const float period) {

    f->length_period = floorf(period / f->ts);
    if (f->length_period < 1) f->length_period = DSMAVGFLT_SIZE_MAX;
    if (f->length_period > DSMAVGFLT_SIZE_MAX) f->length_period = DSMAVGFLT_SIZE_MAX;

    unsigned int old_idx = (f->idx + DSMAVGFLT_SIZE_MAX - f->length_period) % DSMAVGFLT_SIZE_MAX;
    float xn = f->buffer[old_idx];

    f->sum_value = f->sum_value - xn + input;
    f->buffer[f->idx] = input;

    float filter_output = f->sum_value / f->length_period;

    f->idx = (f->idx + 1) % DSMAVGFLT_SIZE_MAX;

    return filter_output;
}


