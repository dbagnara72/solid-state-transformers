
#include <dsmavgflt.h>
#include <dsmavgflt_simulink.h>

// ------------------------------------------------------------------------------
static void init_alldsmavgflt_instances(DSMAVGFLT *const filter_list, 
	const unsigned int filter_num, const float ts) {
		
    unsigned int i;
    for (i = 0; i < filter_num; ++i) {
        DSMAVGFLT *const filter = &filter_list[i];
		dsmavgflt_init(filter, ts);
		i++;
	}
}

// ------------------------------------------------------------------------------
DSMAVGFLT_OUTPUT dsmavgflt_process_simulink(const float input, const float period, 
	const float ts, unsigned char reset, unsigned char instance) {

	static DSMAVGFLT filter_instances[NDSMAVGFLT_INSTANCES] = {0};
	static unsigned int filter_initialized = 0;

	if (!filter_initialized) {
	    init_alldsmavgflt_instances(filter_instances, NDSMAVGFLT_INSTANCES, ts);
		filter_initialized = 1;
	}

	if (instance < NDSMAVGFLT_INSTANCES) {
		const DSMAVGFLT* filter_instance = &filter_instances[instance];


		if(reset) {
			dsmavgflt_init(filter_instance, ts);
		}

		const float output_value = dsmavgflt_process(filter_instance, input, period);

		const DSMAVGFLT_OUTPUT filter_output = {
			.dsmavg_output = output_value
		};
		return filter_output;
	}
	else {
		const DSMAVGFLT_OUTPUT empty_output = {
			0
		};
		return empty_output;
	}
}

