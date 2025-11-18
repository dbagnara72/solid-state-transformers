#ifndef _RPI_
#define _RPI_

#include <math.h>
#include <math_f.h>


#define RPI_DESAT	0.001F

typedef struct rpi_s {
	float			ts;		    				/* sampling time */
	float			y_out;		    			/*  */
	float			x1;							/*  */
	float			x2;		    				/*  */
	float			kp_rpi;		    			/*  */
	float			ki_rpi;		    			/*  */
	float			a11;		    			/*  */
	float			a12;		    			/*  */
	float			a21;		    			/*  */
	float			a22;		    			/*  */
	float			u_out_lim;	    			/*  */
	unsigned char	clip_active;		    	/*  */
} rpi_t;
#define RPI rpi_t



extern void rpi_init(volatile RPI *rpi_ctrl, volatile float ts, volatile float kp_rpi, volatile float ki_rpi, 
			volatile float omega, volatile float delta, volatile float u_out_lim);

extern void rpi_ts(volatile RPI *rpi_ctrl, volatile float ts);

extern void rpi_reset(volatile RPI *rpi_ctrl);

extern float rpi_process(volatile RPI *rpi_ctrl, volatile float i_ref, volatile float i, 
	volatile float u_dc);

#endif
