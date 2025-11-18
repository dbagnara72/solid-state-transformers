
#include <mavgflts.h>
#include <mavgflts_simulink.h>

// ------------------------------------------------------------------------------
static void init_allmavgflt_instances(MAVGFLTS *const filter_list, 
	const unsigned int filter_num, const float ts)
{
    unsigned int i;
    for (i = 0; i < filter_num; ++i) {
        MAVGFLTS *const filter = &filter_list[i];
		mavgflts_init(filter, ts);
		i++;
	}
}

// ------------------------------------------------------------------------------
MAVGFLTS_OUTPUT mavgflts_process_simulink(const float input, const float period, 
	const float ts, unsigned char reset, unsigned char instance)
{
	static MAVGFLTS filter_instances[NMAVGFLTS_INSTANCES] = {0};
	static unsigned int filter_initialized = 0;

	if (!filter_initialized) {
	    init_allmavgflt_instances(filter_instances, NMAVGFLTS_INSTANCES, ts);
		filter_initialized = 1;
	}

	if (instance < NMAVGFLTS_INSTANCES) {
		const MAVGFLTS* filter_instance = &filter_instances[instance];


		if(reset) {
			mavgflts_init(filter_instance, ts);
		}

		const float output_value = mavgflts_process(filter_instance, input, period);

		const MAVGFLTS_OUTPUT filter_output = {
			.ts = filter_instance->ts,
			.sum_value = filter_instance->sum_value,
			.pinput = filter_instance->pinput,
			.cbpointer = filter_instance->cbpointer,
			.cbsample = filter_instance->cbsample,
			.mavgs_output = output_value
		};
		return filter_output;
	}
	else {
		const MAVGFLTS_OUTPUT empty_output = {
			0
		};
		return empty_output;
	}
}

