/*
 * Copyright (C) 2013-2022 by LeitDrive. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <derivative_observer.h>

/***************************************************************/
void DerivativeObserver_Init(
    derivative_obs_t *const observer, const derivative_obs_deps_t *const dependencies,
    const derivative_obs_config_t *const config
)
{
    observer->config       = *config;
    observer->dependencies = *dependencies;
    DerivativeObserver_Reset(observer);
}

/***************************************************************/
void DerivativeObserver_Reset(derivative_obs_t *const observer)
{
    observer->x1            = 0;
    observer->x2            = 0;
    observer->x_hat         = 0;
    observer->x_dot_hat     = 0;
    observer->x_dot_dot_hat = 0;
}

/***************************************************************/
derivative_obs_out_t DerivativeObserver_Process(derivative_obs_t *const observer, const float input)
{
    /* Short names */
    const float l_x         = *observer->config.l_x;
    const float l_x_dot     = *observer->config.l_x_dot;
    const float l_x_dot_dot = *observer->config.l_x_dot_dot;
    const float omega_nom   = *observer->config.f_nom * MATH_2PI;
    const float tb          = *observer->dependencies.timebase;

    /* Observer error */
    const float x_tilde = input - observer->x_hat;

    /* Observer equations */
    observer->x1            = observer->x1 + x_tilde * l_x_dot_dot;
    observer->x_dot_dot_hat = observer->x1 * omega_nom * omega_nom;

    observer->x2        = observer->x2 + x_tilde * l_x_dot + observer->x1 * omega_nom * tb;
    observer->x_dot_hat = observer->x2 * omega_nom;

    observer->x_hat = observer->x_hat + x_tilde * l_x + observer->x_dot_hat * tb;

    /* Write output structure */
    derivative_obs_out_t output = {
        .dot_hat     = observer->x_dot_hat,
        .dot_dot_hat = observer->x_dot_dot_hat,
    };

    return output;
}
