#include <dqvector_pi.h>

void dqvector_pi_init(volatile DQVECTOR_PI *dqvpi_ctrl, volatile float kp_id, volatile float ki_id, 
		volatile float kp_iq, volatile float ki_iq, volatile float u_lim)
{
	dqvpi_ctrl->ts = 0.0;
	dqvpi_ctrl->kp_id = kp_id;
	dqvpi_ctrl->ki_id = ki_id;
	dqvpi_ctrl->kp_iq = kp_iq;
	dqvpi_ctrl->ki_iq = ki_iq;
	dqvpi_ctrl->u_lim = u_lim;
	dqvector_pi_reset(dqvpi_ctrl);
}

void dqvector_pi_ts(volatile DQVECTOR_PI *dqvpi_ctrl, volatile float ts)
{
	dqvpi_ctrl->ts = ts;
}

void dqvector_pi_reset(volatile DQVECTOR_PI *dqvpi_ctrl)
{
	dqvpi_ctrl->ud_i_out = 0.0;
	dqvpi_ctrl->ud_i_out_z = 0.0;
	dqvpi_ctrl->uq_i_out = 0.0;
	dqvpi_ctrl->uq_i_out_z = 0.0;
	dqvpi_ctrl->clip_active = 0;
}

VECTORDQ dqvector_pi_process(volatile DQVECTOR_PI *dqvpi_ctrl, volatile float id_ref, volatile float id, 
	volatile float iq_ref, volatile float iq, volatile float u_dc)
{
	const float id_tilde = id_ref - id;
	const float iq_tilde = iq_ref - iq;
	const float ud_p_out = id_tilde * dqvpi_ctrl->kp_id;
	const float uq_p_out = iq_tilde * dqvpi_ctrl->kp_iq;

	float ud_i_out_z = 0.0;
	float uq_i_out_z = 0.0;
	
	if (!dqvpi_ctrl->clip_active) {
		ud_i_out_z = id_tilde * dqvpi_ctrl->ki_id * dqvpi_ctrl->ts + dqvpi_ctrl->ud_i_out;
		uq_i_out_z = iq_tilde * dqvpi_ctrl->ki_iq * dqvpi_ctrl->ts + dqvpi_ctrl->uq_i_out;
	}
	else {
		ud_i_out_z = dqvpi_ctrl->ud_i_out - dqvpi_ctrl->ud_i_out * dqvpi_ctrl->ts * DQVECTOR_PI_DESAT;
		uq_i_out_z = dqvpi_ctrl->uq_i_out - dqvpi_ctrl->uq_i_out * dqvpi_ctrl->ts * DQVECTOR_PI_DESAT;
	}

	dqvpi_ctrl->ud_i_out_z = ud_i_out_z;
	dqvpi_ctrl->uq_i_out_z = uq_i_out_z;

	dqvpi_ctrl->ud_i_out = dqvpi_ctrl->ud_i_out_z;
	dqvpi_ctrl->uq_i_out = dqvpi_ctrl->uq_i_out_z;

	VECTORDQ udq_out = {
		.d = dqvpi_ctrl->ud_i_out + ud_p_out,
		.q = dqvpi_ctrl->uq_i_out + uq_p_out
	};

	const float u_out = sqrtf(udq_out.d * udq_out.d + udq_out.q * udq_out.q);
	const float u_out_reciprocal = 1/u_out;

	if (u_out >= dqvpi_ctrl->u_lim) {
		dqvpi_ctrl->clip_active = 1;
		udq_out.d = udq_out.d * u_out_reciprocal;
		udq_out.q = udq_out.q * u_out_reciprocal;
	} 
	else {
		dqvpi_ctrl->clip_active = 0;
	}

	return udq_out;
}

