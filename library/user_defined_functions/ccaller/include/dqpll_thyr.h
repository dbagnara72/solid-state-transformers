#ifndef _DQPLL_THYR_
#define _DQPLL_THYR_

//#include <math.h>
#include <math_f.h>

#define CABCD_STATE_STOP				        0 /* ricevo un comando di stop o un reset dalla stato di fault, eventuale contattore Ã© aperto, INVERTER e ZVS spenti */
#define CABCD_STATE_FAULT				        1 /* sono in allarme, apro l'eventuale contattore */
#define CABCD_STATE_READY				        2 /* PLL Ã© attivo e nonho allarmi e sono pronto a partire */
#define CABCD_STATE_CONNECTING		            3 /* non so */
#define CABCD_STATE_ZVS_RUN				        4 /* faccio parite lo ZVS e do il consenso all'inverter*/
#define CABCD_STATE_INV_MON_RUN				    5 /* faccio parite l'INVERTER */
#define CABCD_STATE_BUCK_RUN                    6 /* faccio parite il BUCK CONVERTER */
#define CABCD_STATE_DISCONNECTING		        7 /* a fronte di un allarme cerco di spegnere prima l'INVERTER e poi lo ZVS*/

#define DQPLL_THYR_DESAT		0.001f
#define DQPLL_THYR_LIM_OK		0.1f
#define KP_DQPLL_THYR			0.25f
#define KI1_DQPLL_THYR			80.0f
#define OMEGA_THYR_NOM			50.0f
#define OMEGA_THYR_NORM_NOM		1.0f
#define KI2_DQPLL_THYR			314.159f
#define OMEGA_HAT_WINDOW1		0.95f
#define OMEGA_HAT_WINDOW2		0.85f

#define DQPLL_THYR_STATE_STOP			0
#define DQPLL_THYR_STATE_FAULT			1
#define DQPLL_THYR_STATE_START			2
#define DQPLL_THYR_STATE_RUN			3
#define DQPLL_THYR_STATE_CONNECTING		4

#define FAULT_DQPLL_THYR_SYNC			0x0040 // Fault Value : 64

#define LOCK_PLL_COUNTFAULT_WINDOW1		2000
#define LOCK_PLL_COUNTFAULT_WINDOW2		500

typedef struct param_thyr_rect_s
{
	int			cabcd_fault;				/*  */
	int			pll_state;					/*  */
	int			cabcd_global_state;			/*  */
	float		pll_lim_ok;
} param_thyr_rect_t;

extern param_thyr_rect_t param_thyr_rect;
extern int lock_pll_countFault_run;
extern int lock_pll_countFault;

typedef struct dqpll_thyr_s
{
	float			ts;		    				/* sampling time */
	unsigned int	dqpll_state;		    	/*  */
	float			kp_dqpll_thyr;		    	/*  */
	float			ki1_dqpll_thyr;		    	/*  */
	float			ki2_dqpll_thyr;		    	/*  */
	float			u_xi;						/*  */
	float			u_eta;						/*  */
	float			omega_hat;					/*  */
	float			omega_i_hat;				/*  */
	float			gamma_hat;					/*  */
	float			omega_pu_lim;				/*  */
	unsigned char	clip_active;		    	/*  */
} dqpll_thyr_t;

#define DQPLL_THYR dqpll_thyr_t

extern void dqpll_thyr_init(volatile DQPLL_THYR *dqpll_ctrl);

extern void dqpll_thyr_ts(volatile DQPLL_THYR *dqpll_ctrl, volatile float ts);

extern void dqpll_thyr_reset(volatile DQPLL_THYR *dqpll_ctrl);

extern float dqpll_thyr_process(volatile DQPLL_THYR *dqpll_ctrl, volatile float u_phase_r, volatile float u_phase_s, volatile float u_phase_t);

#endif
