/* six pulses rectifier */

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


typedef struct sprc_s {
    float ramp_1; float ramp_2; float ramp_3; 
	float ramp_4; float ramp_5; float ramp_6;  
	
	int synch_A1; int synch_A2; int synch_A3; 
	int synch_A4; int synch_A5; int synch_A6;	
	
	int synch_B1; int synch_B2; int synch_B3; 
	int synch_B4; int synch_B5; int synch_B6;

	int synch_1; int synch_2; int synch_3;
	int synch_4; int synch_5; int synch_6;
} sprc_t;

typedef struct spr_p_s {
	int p1;
	int p2;
	int p3;
	int p4;
	int p5;
	int p6;
} spr_p_t;

float sprcProcess(sprc_t *spr, const float wt, const float alpha, const int block, spr_p_t *p);

void sprcProcessSimulink(const float wt, const float alpha, const int block, spr_p_t *p);
