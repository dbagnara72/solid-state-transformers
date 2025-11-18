/* space vector modulation */

#include <sv_pwm.h>

void sv_pwm_process(SVPWM *c)
{
	float t1 = MATH_NULL;
	float t2 = MATH_NULL;
	float t0 = MATH_NULL;
	float angle = MATH_NULL;
	
	const float tbpwm = MATH_1;

	if (c->enable)
	{
    	angle = atan2f(c->ubeta,c->ualpha);

	    if ((angle >= MATH_NULL) && (angle <= MATH_PI_3)) {
	    	/* sector1 (counterclockwise) */
	    	t2 = c->ubeta * MATH_2_SQRT3; // v2
			t1 = c->ualpha - c->ubeta * MATH_1_SQRT3; // v1
			t0 = MATH_1 - t2 - t1;
			/* management of the overmudulation */
			if (t0 < 0) {
				t1 = MATH_1 - t2;
				c->da = MATH_NULL;
				c->db = t1;
				c->dc = MATH_1;
				}
        	else {
				c->da = t0 * tbpwm * MATH_HALF;
				c->db = t1 * tbpwm + c->da;
				c->dc = t2 * tbpwm + c->db;
			}

		}

	    else if ((angle >= MATH_PI_3) && (angle <= MATH_2PI_3)) {
	        /* sector2 (clockwise) */
	        t1 = -c->ualpha + c->ubeta * MATH_1_SQRT3; // v3
	        t2 = c->ualpha + c->ubeta * MATH_1_SQRT3; // v2
	        t0 = MATH_1 - t2 - t1;
			/* management of the overmudulation */
			if (t0 < 0) {
				t1 = MATH_1 - t2;
				c->db = MATH_NULL;
				c->da = t1;
				c->dc = MATH_1;
				}
        	else {
				c->db = t0 * tbpwm * MATH_HALF;
				c->da = t1 * tbpwm + c->db;
				c->dc = t2 * tbpwm + c->da;
			}
	    }

	    else if ((angle >= MATH_2PI_3) && (angle <= MATH_PI)) {
	        /* sector3 (counterclockwise) */
	        t2 = -c->ubeta * MATH_1_SQRT3 - c->ualpha; // v4
	        t1 = c->ubeta * MATH_2_SQRT3; //v3
	        t0 = MATH_1 - t2 - t1;
			/* management of the overmudulation */
			if (t0 < 0) {
				t1 = MATH_1 - t2;
				c->db = MATH_NULL;
				c->dc = t1;
				c->da = MATH_1;
				}
        	else {
				c->db = t0 * tbpwm * MATH_HALF;
				c->dc = t1 * tbpwm + c->db;
				c->da = t2 * tbpwm + c->dc;
			}
	    }

	    else if ((angle >= -MATH_PI) && (angle <= -MATH_2PI_3)) {
	        /* sector4 (clockwise) */
	        t1 = -MATH_2_SQRT3 * c->ubeta; //v5
	        t2 = c->ubeta * MATH_1_SQRT3 - c->ualpha; //v4
	        t0 = MATH_1 - t2 - t1;
			/* management of the overmudulation */
			if (t0 < 0) {
				t1 = MATH_1 - t2;
				c->dc = MATH_NULL;
				c->db = t1;
				c->da = MATH_1;
				}
        	else {
				c->dc = t0 * tbpwm * MATH_HALF;
				c->db = t1 * tbpwm + c->dc;
				c->da = t2 * tbpwm + c->db;
			}
	    }

	    else if ((angle >= -MATH_2PI_3) && (angle <= -MATH_PI_3)) {
	        /* sector5 (counterclockwise) */
	        t2 = c->ualpha - c->ubeta * MATH_1_SQRT3; // v6
	        t1 = -c->ualpha - c->ubeta * MATH_1_SQRT3; // v5
	        t0 = MATH_1 - t2 - t1;
			/* management of the overmudulation */
			if (t0 < 0) {
				t1 = MATH_1 - t2;
				c->dc = MATH_NULL;
				c->da = t1;
				c->db = MATH_1;
				}
        	else {
				c->dc = t0 * tbpwm * MATH_HALF;
				c->da = t1 * tbpwm + c->dc;
				c->db = t2 * tbpwm + c->da;
			}
	    }

	    /* else ((angle >= -MATH_PI_3) && (angle <= MATH_NULL)) { */
	    else  {
	        /* sector6 (clockwise) */
	        t1 = c->ualpha + c->ubeta * MATH_1_SQRT3; // v1
	        t2 = -c->ubeta * MATH_2_SQRT3; // v6
	        t0 = MATH_1 - t2 - t1;
			/* management of the overmudulation */
			if (t0 < 0) {
				t1 = MATH_1 - t2;
				c->da = MATH_NULL;
				c->dc = t1;
				c->db = MATH_1;
				}
        	else {
				c->da = t0 * tbpwm * MATH_HALF;
				c->dc = t1 * tbpwm + c->da;
				c->db = t2 * tbpwm + c->dc;
			}
	    }
	}
	else
	{
		c->da = MATH_HALF * tbpwm;
		c->db = MATH_HALF * tbpwm;
		c->dc = MATH_HALF * tbpwm;
	}

}

	