/* Dual six pulses rectifier */

#ifndef _DSPRCTRL_
#define _DSPRCTRL_

#define MATH_PI_6		0.523598775598299
#define MATH_PI_3		1.0471975511966
#define MATH_PI_2		1.5707963267949
#define MATH_PI			3.14159265358979
#define MATH_2PI		6.28318530717959
#define MATH_1_2		0.5
#define MATH_SQRT3_2	0.866025403784439
#define MATH_RAD_DEG	57.2957795130823
#define PWIDTH			110


typedef struct dspr_ctrl_s {
    float ramp_d_1;
    float ramp_d_2;
    float ramp_d_3;
    float ramp_d_4;
    float ramp_d_5;
    float ramp_d_6;  
    float ramp_y_1;
    float ramp_y_2;
    float ramp_y_3;
    float ramp_y_4;
    float ramp_y_5;
    float ramp_y_6;  
	int synch_dA1;
	int synch_dA2;
	int synch_dA3;
	int synch_dA4;
	int synch_dA5;
	int synch_dA6;	
	int synch_dB1;
	int synch_dB2;
	int synch_dB3;
	int synch_dB4;
	int synch_dB5;
	int synch_dB6;
	int synch_d1;
	int synch_d2;
	int synch_d3;
	int synch_d4;
	int synch_d5;
	int synch_d6;
	
	int synch_yA1;
	int synch_yA2;
	int synch_yA3;
	int synch_yA4;
	int synch_yA5;
	int synch_yA6;	
	int synch_yB1;
	int synch_yB2;
	int synch_yB3;
	int synch_yB4;
	int synch_yB5;
	int synch_yB6;
	int synch_y1;
	int synch_y2;
	int synch_y3;
	int synch_y4;
	int synch_y5;
	int synch_y6;
} dspr_ctrl_t;
#define DSPRCTRL dspr_ctrl_t

typedef struct dspr_pd_pos_s {
	int pd_pos_1;
	int pd_pos_2;
	int pd_pos_3;
	int pd_pos_4;
	int pd_pos_5;
	int pd_pos_6;
}  dspr_pd_pos_t;
#define DSPR_PD_POS dspr_pd_pos_t

typedef struct dspr_pd_neg_s {
	int pd_neg_1;
	int pd_neg_2;
	int pd_neg_3;
	int pd_neg_4;
	int pd_neg_5;
	int pd_neg_6;
}  dspr_pd_neg_t;
#define DSPR_PD_NEG dspr_pd_neg_t

float dsprProcess(DSPRCTRL *dspr, float wt, float alpha, int block, DSPR_PD_POS *pd_pos, DSPR_PD_NEG *pd_neg);

void twvprProcessSimulink(float wt, float alpha, int block, TWVPR_PY *py, TWVPR_PD *pd);

#endif
