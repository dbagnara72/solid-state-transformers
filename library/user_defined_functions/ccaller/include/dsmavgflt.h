#ifndef _DSMAVGFLT_
#define _DSMAVGFLT_

#define DSMAVGFLT_SIZE_MAX	1600

typedef struct dsmavgflt_s {
	float					ts;						    		/* time base */
	float					sum_value;		            		/* sum buffer */
	float					buffer[DSMAVGFLT_SIZE_MAX];   		/* buffer */
	unsigned int			idx;			        			/* buffer array pointer */
	unsigned int			length_period;                  	/* current number of samples for the current period */
} dsmavgflt_t;
#define DSMAVGFLT dsmavgflt_t

void dsmavgflt_init(volatile DSMAVGFLT *f, volatile float ts);

float dsmavgflt_process(volatile DSMAVGFLT *f, float input, const float period);

#endif
