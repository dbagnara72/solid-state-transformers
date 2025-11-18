#ifndef _MAVGFLT_
#define _MAVGFLT_

#define MAVGFLT_SIZE_MAX	1600

typedef struct mavgflt_s {
	float								ts;						    /* Filter time base. */
	float								sshort;		                /* Integration with floor period length. */
	float 								slong;		                /* Integration with ceil period length. */
	float								pinput;			            /* step back input value */
	float								buffer[MAVGFLT_SIZE_MAX];   /* buffer */
	unsigned int						cbpointer;			        /* buffer array pointer for the newest element to be written */
	unsigned int						cbsample;                  /* current number of samples for the current period */
} mavgflt_t;
#define MAVGFLT mavgflt_t

void mavgflt_init(volatile MAVGFLT *f);

void mavgflt_ts(volatile MAVGFLT *f, volatile float ts);

void mavgflt_reset(volatile MAVGFLT *f);

float mavgflt_process(volatile MAVGFLT *f, float input, const float period);

#endif
