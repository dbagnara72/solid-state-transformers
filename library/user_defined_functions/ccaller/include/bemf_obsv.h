#ifndef _BEMF_OBSV_
#define _BEMF_OBSV_

#include <math.h>
#include <math_f.h>

#define BEMF_OBSERVER_UPDATE_TH 0.5
#define BEMF_OBSERVER_UPDATE_HY 0.25
#define	DELAYED_ROTOR_FLUX


typedef struct bemf_obsv_s {
	float			ts;		    					/* sampling time */
	float			psi_s_alpha_hat;		    	/*  */
	float			psi_r_alpha_hat;		    	/*  */
	float			psi_r_alpha_hat_hat;		    /*  */
	float			psi_ls_alpha_hat;		    	/*  */
	float			psi_s_beta_hat;		    		/*  */
	float			psi_r_beta_hat;		    		/*  */
	float			psi_r_beta_hat_hat;	    		/*  */
	float			psi_ls_beta_hat;		    	/*  */
	float			theta_hat;		    			/*  */
	float			omega_hat;		    			/*  */
	float			omega_hat_flt;		    		/*  */
	float			bemf_obsv_fb_p_1;		    	/*  */
	float			bemf_obsv_p_1;		    		/*  */
	float			bemf_obsv_fb_p_2;		    	/*  */
	float			bemf_obsv_p_2;		    		/*  */
	float			motorc_omega_bez;		    	/*  */
	float			motorc_omega_bez_i;		    	/*  */
	float			motorc_m_scale;		    		/*  */
	float			motorc_rs_norm;		    		/*  */
	float			motorc_ls_norm;		    		/*  */
	float			motorc_psi_m_norm;		    	/*  */
	float			bemf_obsv_kalman_omega;			/*  */
	float			bemf_obsv_kalman_theta;			/*  */
	float			phase_compensation_omega;		/*  */
	float			phase_compensation_torque;		/*  */
	float			omega_flt_fcut;					/*  */
	float			omega_flt_g0;					/*  */
	float			omega_flt_g1;					/*  */
} bemf_obsv_t;
#define BEMF_OBSV bemf_obsv_t

void bemf_obsv_init(volatile BEMF_OBSV *f, volatile float bemf_obsv_fb_p_1, volatile float bemf_obsv_p_1, volatile float bemf_obsv_fb_p_2, volatile float bemf_obsv_p_2, 
		volatile float motorc_omega_bez, volatile float motorc_m_scale, volatile float motorc_rs_norm, 
		volatile float motorc_ls_norm, volatile float motorc_psi_m_norm, volatile float bemf_obsv_kalman_omega, 
		volatile float bemf_obsv_kalman_theta, volatile float phase_compensation_omega, volatile float phase_compensation_torque, volatile float omega_flt_fcut);

void bemf_obsv_ts(volatile BEMF_OBSV *f, volatile float ts);

void bemf_obsv_reset(volatile BEMF_OBSV *f);

void bemf_obsv_update(volatile BEMF_OBSV *f,volatile float bemf_obsv_fb_p_1, volatile float bemf_obsv_p_1, volatile float bemf_obsv_fb_p_2, volatile float bemf_obsv_p_2);

float bemf_obsv_process(volatile BEMF_OBSV *f, volatile float u_alpha, volatile float u_beta, volatile float i_alpha, 
	volatile float i_beta, volatile float u_dc);

void obsv_omega_flt_init(volatile BEMF_OBSV *f);

#endif
