/* Twelve pulses rectifier */

#ifndef _TWVPRCTRL_
#define _TWVPRCTRL_

#define MATH_PI_6		0.523598775598299f
#define MATH_5PI_3		5.2360f
#define MATH_4PI_3		4.1888f
#define MATH_2PI_3		2.0944f
#define MATH_PI_3		1.0471975511966f
#define MATH_PI_2		1.5707963267949f
#define MATH_PI			3.14159265358979f
#define MATH_2PI		6.28318530717959f
#define MATH_1_2		0.5f
#define MATH_SQRT3_2	0.866025403784439f
#define MATH_RAD_DEG	57.2957795130823f
#define PWIDTH			1.9f


typedef struct twvprctrl_s {
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
} twvprctrl_t;
#define TWVPRCTRL twvprctrl_t

typedef struct twvpr_pd_s {
	int pd_1;
	int pd_2;
	int pd_3;
	int pd_4;
	int pd_5;
	int pd_6;
}  twvpr_pd_t;
#define TWVPR_PD twvpr_pd_t

typedef struct twvpr_py_s {
	int py_1;
	int py_2;
	int py_3;
	int py_4;
	int py_5;
	int py_6;
} twvpr_py_t;
#define TWVPR_PY twvpr_py_t

float twvprProcess(TWVPRCTRL *twvpr, const float wt, const float alpha, const int block, TWVPR_PY *py, TWVPR_PD *pd);

void twvprProcessSimulink(const float wt, const float alpha, const int block, TWVPR_PY *py, TWVPR_PD *pd);

#endif
