#ifndef _MAVGFLTS_
#define _MAVGFLTS_

#define MAVGFLTS_SIZE_MAX	1600

typedef struct mavgflts_s {
	float					ts;						    /* time base */
	float					sum_value;		            /* sum buffer */
	float					pinput;			            /* step back input value */
	float					buffer[MAVGFLTS_SIZE_MAX];   /* buffer */
	unsigned int			cbpointer;			        /* buffer array pointer */
	unsigned int			cbsample;                  	/* current number of samples for the current period */
} mavgflts_t;
#define MAVGFLTS mavgflts_t

void mavgflts_init(volatile MAVGFLTS *f, volatile float ts);

float mavgflts_process(volatile MAVGFLTS *f, float input, const float period);

#endif
