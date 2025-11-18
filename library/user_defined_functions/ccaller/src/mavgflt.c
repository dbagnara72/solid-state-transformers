#include <mavgflt.h>

void mavgflt_init(volatile MAVGFLT *f)
{
	f->ts = 0.0;
	mavgflt_reset(f);
}

void mavgflt_ts(volatile MAVGFLT *f, volatile float ts)
{
	f->ts = ts;
}

void mavgflt_reset(volatile MAVGFLT *f)
{
	int i;

	f->pinput = 0.0;
	f->sshort = 0.0;
	f->slong = 0.0;
	f->cbpointer = 0;
	f->cbsample = 0;
	for (i = 0; i < MAVGFLT_SIZE_MAX; i++) {
		f->buffer[i] = 0.0;
	}
}

static unsigned int get_buffer_pointer(const unsigned int cbpointer, const unsigned int cnperiod)
{
	if (cnperiod <= cbpointer) {
		return cbpointer - cnperiod;
	}
	else {
		return cbpointer - cnperiod + MAVGFLT_SIZE_MAX;
	}
}

// ------------------------------------------------------------------------------
static float get_buffer_value(const MAVGFLT * const f, const unsigned int cnperiod)
{
	const unsigned int current_nperiod = get_buffer_pointer(f->cbpointer, cnperiod);
	return f->buffer[current_nperiod];
}

float mavgflt_process(volatile MAVGFLT *f, float input, const float period)
{
	if (period > 0.0) {
		
		float samples_requested_f = period / f->ts;
		
		if (samples_requested_f > MAVGFLT_SIZE_MAX - 1)
		    samples_requested_f = MAVGFLT_SIZE_MAX - 1;
		
		const unsigned int cnbuffer = samples_requested_f + 1;
		
		const float sampling_fraction = samples_requested_f + 1 - cnbuffer;

		if (cnbuffer > f->cbsample) {
			f->sshort += f->pinput;
			f->slong += input;
			f->cbsample++;
		}
		else if (cnbuffer < f->cbsample) {
			const unsigned int decremented_cbsample = f->cbsample - 1;
			const float two_oldest_samples = get_buffer_value((MAVGFLT*) f, f->cbsample)
												+ get_buffer_value((MAVGFLT*) f, decremented_cbsample);

			f->sshort += f->pinput - two_oldest_samples;
			f->slong += input - two_oldest_samples;
					
			f->cbsample = decremented_cbsample;
		}
		else {
			const float oldest_period_sample = get_buffer_value((MAVGFLT*) f, f->cbsample);
			
			f->sshort += f->pinput - oldest_period_sample;
			f->slong += input - oldest_period_sample;
		}
		
		f->buffer[f->cbpointer] = input;

		f->cbpointer++;
		
		f->pinput = input;

		if (f->cbpointer == MAVGFLT_SIZE_MAX) {
			f->cbpointer = 0;
		}

		const float cbsample_short = f->cbsample - 1;
		const float filter_output_short = cbsample_short ? f->sshort / cbsample_short : 0.0;
		const float filter_output_long = f->slong / f->cbsample;
		
		return filter_output_short * (1 - sampling_fraction) + filter_output_long * sampling_fraction;
	}
	else {
		return 0.0;
	}
}

