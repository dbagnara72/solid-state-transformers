#ifndef _DQVECTOR_PI_
#define _DQVECTOR_PI_

#include <math.h>
#include <math_f.h>


#define DQVECTOR_PI_DESAT	0.001F

typedef struct vector_dq_s
{
	float			d;
	float			q;
} vector_dq_t;
#define VECTORDQ vector_dq_t

typedef struct dqvector_pi_s {
	float			ts;		    				/* sampling time */
	float			ud_i_out;		    		/*  */
	float			ud_i_out_z;					/*  */
	float			uq_i_out;		    		/*  */
	float			uq_i_out_z;		    		/*  */
	unsigned char	clip_active;		    	/*  */
	float			kp_id;		    			/*  */
	float			ki_id;		    			/*  */
	float			kp_iq;		    			/*  */
	float			ki_iq;		    			/*  */
	float			u_lim;		    			/*  */
} dqvector_pi_t;
#define DQVECTOR_PI dqvector_pi_t



extern void dqvector_pi_init(volatile DQVECTOR_PI *dqvpi_ctrl, volatile float kp_id, volatile float ki_id, 
		volatile float kp_iq, volatile float ki_iq, volatile float u_lim);

extern void dqvector_pi_ts(volatile DQVECTOR_PI *dqvpi_ctrl, volatile float ts);

extern void dqvector_pi_reset(volatile DQVECTOR_PI *dqvpi_ctrl);

extern VECTORDQ dqvector_pi_process(volatile DQVECTOR_PI *dqvpi_ctrl, volatile float id_ref, volatile float id, 
		volatile float iq_ref, volatile float iq, volatile float u_dc);

#endif
