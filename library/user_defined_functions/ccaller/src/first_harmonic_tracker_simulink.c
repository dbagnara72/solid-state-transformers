
#include <first_harmonic_tracker.h>
#include <first_harmonic_tracker_simulink.h>

// ------------------------------------------------------------------------------
static void init_allfht_instances(FIRST_HARMONIC_TRACKER *const fht_list, const unsigned int fht_num)
{
   /* issue - instance number in simulink at step of two : 0, 2, 4, 6, 8 - double FHTI_N than expected */
	unsigned int i;
    for (i = 0; i < fht_num; ++i) {
        FIRST_HARMONIC_TRACKER *const fht_i = &fht_list[i];
		first_harmonic_tracker_init(fht_i);
		i++;
	}
}

// ------------------------------------------------------------------------------
FIRST_HARMONIC_TRACKER_OUTPUT first_harmonic_tracker_process_simulink(const unsigned char reset, const float u, const float ts, 
	const unsigned char instance)
{

	static FIRST_HARMONIC_TRACKER fht_i[FHTI_N] = {0};
	static unsigned int fht_init = 0;

	if (!fht_init) {
	    init_allfht_instances(fht_i, FHTI_N);
		fht_init = 1;
	}

	const FIRST_HARMONIC_TRACKER* fht_instance = &fht_i[instance];
	
	if (reset) {
		first_harmonic_tracker_ts(fht_instance, ts);
		first_harmonic_tracker_reset(fht_instance);
	}
	
	const float output_value = first_harmonic_tracker_process(fht_instance, u);

	const FIRST_HARMONIC_TRACKER_OUTPUT output = {
//		.output_pu_hat = fht_instance->first_harmonic_tracker_output
		.output_pu_hat = output_value
	};
	return output;
}

