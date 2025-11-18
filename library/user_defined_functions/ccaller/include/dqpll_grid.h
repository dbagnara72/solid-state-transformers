#ifndef _DQPLL_GRID_
#define _DQPLL_GRID_

#include <math.h>
#include <math_f.h>

#define CABCD_STATE_STOP				        0 /* ricevo un comando di stop o un reset dalla stato di fault, eventuale contattore Ã© aperto, INVERTER e ZVS spenti */
#define CABCD_STATE_FAULT				        1 /* sono in allarme, apro l'eventuale contattore */
#define CABCD_STATE_READY				        2 /* PLL Ã© attivo e nonho allarmi e sono pronto a partire */
#define CABCD_STATE_CONNECTING		            3 /* non so */
#define CABCD_STATE_ZVS_RUN				        4 /* faccio parite lo ZVS e do il consenso all'inverter*/
#define CABCD_STATE_INV_MON_RUN				    5 /* faccio parite l'INVERTER */
#define CABCD_STATE_BUCK_RUN                    6 /* faccio parite il BUCK CONVERTER */
#define CABCD_STATE_DISCONNECTING		        7 /* a fronte di un allarme cerco di spegnere prima l'INVERTER e poi lo ZVS*/

#define DQPLL_GRID_DESAT		0.001f
#define DQPLL_GRID_LIM_OK		0.2f
#define KP_DQPLL_GRID			1.0f
#define KI1_DQPLL_GRID			80.0f
#define OMEGA_GRID_NOM			314.1592653589793f
#define OMEGA_GRID_NORM_NOM		1.0f
#define KI2_DQPLL_GRID			314.1592653589793f
#define OMEGA_HAT_WINDOW1		0.5f
#define OMEGA_HAT_WINDOW2		0.5f

#define DQPLL_GRID_STATE_STOP			0
#define DQPLL_GRID_STATE_FAULT			1
#define DQPLL_GRID_STATE_START			2
#define DQPLL_GRID_STATE_RUN			3
#define DQPLL_GRID_STATE_CONNECTING		4

#define FAULT_DQPLL_GRID_SYNC			0x0040 // Fault Value : 64

#define AGGANCIO_PLL_CONTAFAULT_WINDOW1		2000
#define AGGANCIO_PLL_CONTAFAULT_WINDOW2		500

typedef struct param_s
{
	int			cabcd_fault;				/*  */
	int			pll_state;					/*  */
	int			cabcd_global_state;			/*  */
	float		pll_lim_ok;
} param_t;

extern param_t param;
extern int aggancio_pll_ContaFault_RUN;
extern int aggancio_pll_ContaFault;

typedef struct dqpll_grid_s
{
	float			ts;		    				/* sampling time */
	unsigned int	dqpll_state;		    	/*  */
	float			kp_dqpll_grid;		    	/*  */
	float			ki1_dqpll_grid;		    	/*  */
	float			ki2_dqpll_grid;		    	/*  */
	float			u_xi;						/*  */
	float			u_eta;						/*  */
	float			omega_hat;					/*  */
	float			omega_i_hat;				/*  */
	float			gamma_hat;					/*  */
	float			omega_pu_lim;				/*  */
	unsigned char	clip_active;		    	/*  */
} dqpll_grid_t;

#define DQPLL_GRID dqpll_grid_t

extern void dqpll_grid_init(volatile DQPLL_GRID *dqpll_ctrl);

extern void dqpll_grid_ts(volatile DQPLL_GRID *dqpll_ctrl, volatile float ts);

extern void dqpll_grid_reset(volatile DQPLL_GRID *dqpll_ctrl);

extern float dqpll_grid_process(volatile DQPLL_GRID *dqpll_ctrl, volatile float u_phase_r, volatile float u_phase_s, volatile float u_phase_t);

#endif
