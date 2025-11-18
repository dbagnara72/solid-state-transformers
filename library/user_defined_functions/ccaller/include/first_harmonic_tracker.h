#ifndef _FIRST_HARMONIC_TRACKER_
#define _FIRST_HARMONIC_TRACKER_

//#include <math.h>
#include <math_f.h>

/*
#define FIRST_HARMONIC_TRACKER_L1 		157.08f
#define FIRST_HARMONIC_TRACKER_L2		-94748.2f
#define FIRST_HARMONIC_TRACKER_A11		0.0f
#define FIRST_HARMONIC_TRACKER_A12		1.0f
#define FIRST_HARMONIC_TRACKER_A21		-98696.0f
#define FIRST_HARMONIC_TRACKER_A22		0.0f
#define FIRST_HARMONIC_TRACKER_C1		1.0f
*/

#define FIRST_HARMONIC_TRACKER_L1 		282.743f
#define FIRST_HARMONIC_TRACKER_L2		-91787.3f
#define FIRST_HARMONIC_TRACKER_A11		0.0f
#define FIRST_HARMONIC_TRACKER_A12		1.0f
#define FIRST_HARMONIC_TRACKER_A21		-98696.0f
#define FIRST_HARMONIC_TRACKER_A22		-31.41592f
#define FIRST_HARMONIC_TRACKER_C1		1.0f


typedef struct first_harmonic_tracker_s {
	float			ts;		    						/* sampling time */
	float 			first_harmonic_tracker_l1;			/*  */
	float 			first_harmonic_tracker_l2;			/*  */
	float 			first_harmonic_tracker_A11;			/*  */
	float 			first_harmonic_tracker_A12;			/*  */
	float 			first_harmonic_tracker_A21;			/*  */
	float 			first_harmonic_tracker_A22;			/*  */
	float 			first_harmonic_tracker_C1;			/*  */
	float			first_harmonic_tracker_error;		/*  */
	float			first_harmonic_tracker_state1;		/*  */
	float			first_harmonic_tracker_state2;		/*  */
	float			first_harmonic_tracker_output;		/*  */
} first_harmonic_tracker_t;
#define FIRST_HARMONIC_TRACKER first_harmonic_tracker_t

void first_harmonic_tracker_init(volatile FIRST_HARMONIC_TRACKER *f);

void first_harmonic_tracker_reset(volatile FIRST_HARMONIC_TRACKER *f);

void first_harmonic_tracker_ts(volatile FIRST_HARMONIC_TRACKER *f, volatile float ts);

float first_harmonic_tracker_process(volatile FIRST_HARMONIC_TRACKER *f, const float u);

#endif
