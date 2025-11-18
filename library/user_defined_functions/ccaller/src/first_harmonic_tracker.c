#include <first_harmonic_tracker.h>

void first_harmonic_tracker_init(volatile FIRST_HARMONIC_TRACKER *f)
{
	f->ts = 0.0F;
	f->first_harmonic_tracker_l1 = FIRST_HARMONIC_TRACKER_L1;
	f->first_harmonic_tracker_l2 = FIRST_HARMONIC_TRACKER_L2;
	f->first_harmonic_tracker_A11 = FIRST_HARMONIC_TRACKER_A11;
	f->first_harmonic_tracker_A12 = FIRST_HARMONIC_TRACKER_A12;
	f->first_harmonic_tracker_A21 = FIRST_HARMONIC_TRACKER_A21;
	f->first_harmonic_tracker_A22 = FIRST_HARMONIC_TRACKER_A22;
	f->first_harmonic_tracker_C1 = FIRST_HARMONIC_TRACKER_C1;
	first_harmonic_tracker_reset(f);
}

void first_harmonic_tracker_reset(volatile FIRST_HARMONIC_TRACKER *f)
{
	f->first_harmonic_tracker_error = 0.0f;
	f->first_harmonic_tracker_state1 = 0.0f;
	f->first_harmonic_tracker_state2 = 0.0f;
	f->first_harmonic_tracker_output = 0.0f;
}

void first_harmonic_tracker_ts(volatile FIRST_HARMONIC_TRACKER *f, volatile float ts)
{
	f->ts = ts;
}

float first_harmonic_tracker_process(volatile FIRST_HARMONIC_TRACKER *f, const float u)
{
	const float first_harmonic_tracker_ld1 = f->first_harmonic_tracker_l1 * f->ts;
	const float first_harmonic_tracker_ld2 = f->first_harmonic_tracker_l2 * f->ts;
	const float first_harmonic_tracker_Ad11 = 1.0F + f->first_harmonic_tracker_A11 * f->ts;
	const float first_harmonic_tracker_Ad12 = f->first_harmonic_tracker_A12 * f->ts;
	const float first_harmonic_tracker_Ad21 = f->first_harmonic_tracker_A21 * f->ts;
	const float first_harmonic_tracker_Ad22 = 1.0F + f->first_harmonic_tracker_A22 * f->ts;

	const float first_harmonic_tracker_output = f->first_harmonic_tracker_output;
	const float error = u - first_harmonic_tracker_output;
	const float stat1 = first_harmonic_tracker_ld1 * error + first_harmonic_tracker_Ad11 * f->first_harmonic_tracker_state1 + first_harmonic_tracker_Ad12 * f->first_harmonic_tracker_state2;
	const float stat2 = first_harmonic_tracker_ld2 * error + first_harmonic_tracker_Ad21 * f->first_harmonic_tracker_state1 + first_harmonic_tracker_Ad22 * f->first_harmonic_tracker_state2;

	f->first_harmonic_tracker_output = f->first_harmonic_tracker_state1 * f->first_harmonic_tracker_C1;
	f->first_harmonic_tracker_state1 = stat1;
	f->first_harmonic_tracker_state2 = stat2;

	return f->first_harmonic_tracker_output;
}

