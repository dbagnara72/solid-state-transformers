#ifndef _FIRST_HARMONIC_TRACKER_SIMULINK_
#define _FIRST_HARMONIC_TRACKER_SIMULINK_

#define FHTI_N 128

typedef struct first_harmonic_tracker_output_s {
	float	output_pu_hat;		/* output pu hat */
} first_harmonic_tracker_output_t;
#define FIRST_HARMONIC_TRACKER_OUTPUT first_harmonic_tracker_output_t

// extern FIRST_HARMONIC_TRACKER fht_i[FHTI];
// extern unsigned int fht_init;

FIRST_HARMONIC_TRACKER_OUTPUT first_harmonic_tracker_process_simulink(const unsigned char reset, const float u, const float ts,
const unsigned char instance);
#endif
